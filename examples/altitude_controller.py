#!/usr/bin/env python

import argparse
import os
import sys
import time
import matplotlib.pyplot as plt
import numpy as np
import pyproj

import gym
from gym.envs.registration import register
from gymfc_nf.envs.base import BaseEnv


def state_degrees_error_deltaerror(local):
    error_delta = local.measured_error - local.last_measured_error
    return np.concatenate([local.measured_error, error_delta])


register(
    id="gymfc_quadrotor_env_v1",
    entry_point='altitude_controller:StepEnv',
    kwargs={
        "state_fn": state_degrees_error_deltaerror,
    }
)


class StepEnv(BaseEnv):
    def __init__(self, pulse_width=1, max_rate=100, state_fn=None, max_sim_time=1):
        super().__init__(max_sim_time=max_sim_time, state_fn=state_fn)

    def reset(self, seed=None, options=None):
        return super().reset(seed, options)

    def sample_target(self):
        """Sample a random angular velocity setpoint """
        return self.np_random.normal(0, self.max_rate, size=3)


class SimplePID(object):

    def __init__(self, kp, ki, kd, output_limits=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        if output_limits is None:
            self.output_limits = (-float("inf"), float("inf"))
        else:
            output_limits = output_limits
        self.reset()

    def _clamp(self, value, limits):
        lower, upper = limits
        if value is None:
            return None
        elif (upper is not None) and (value > upper):
            return upper
        elif (lower is not None) and (value < lower):
            return lower
        return value

    def update(self, t, e):
        # Most environments have a short execution time the controller doesn't have much time to wind up
        dt = t - self.last_t
        self.last_t = t
        p_term = self.kp * e
        self.accum += e * dt
        i_term = self.ki * self.accum
        de = e - self.last_e
        self.last_e = e
        d_term = self.kd * de / dt if dt > 0 else 0
        return self._clamp(p_term + i_term + d_term, self.output_limits)

    def reset(self):
        self.last_t = 0
        self.last_e = 0
        self.accum = 0


if __name__ == '__main__':
    default_model = "../gazebo/models/iris/iris_altitude.sdf"
    # default_model = "../gazebo/models/nf1/model.sdf"
    default_config = "iris_gymfc/gymfc.ini"
    parser = argparse.ArgumentParser("Evaluate each algorithm")
    parser.add_argument('--model', default=default_model,
                        help="File path of the NF1 twin/model SDF.")
    parser.add_argument('--gymfc-config', default=default_config,
                        help="Option to override default GymFC configuration location.")
    parser.add_argument('--gym-id', default="gymfc_nf-step-v1")
    parser.add_argument('--seed', help='RNG seed', type=int, default=0)
    args = parser.parse_args()

    verbose = True
    render = True
    current_altitude = 487.9

    # Need to set the aircraft model after we create the environment because
    # the model is unique and can't be hardcoded in the gym init.
    # env = gym.make(args.gym_id)
    env = gym.make("gymfc_quadrotor_env_v1")
    env.setup(args.model, args.gymfc_config)
    env.seed(args.seed)
    env.reset()
    if render:
        env.render()
    env.verbose = verbose

    # 3DR Iris Vehicle
    pid = SimplePID(kp=1e-4, ki=0.000000, kd=1e-2)
    motor_rpm_sp = 790
    #  NF 1 Vehicle
    # pid = SimplePID(kp=1e-6, ki=0.000000, kd=1e-10)
    # motor_rpm_sp = 850

    pid.output_limits = (-1000, 1000)

    # let the quad settle onto the ground
    while env.sim_time < 4:
        ob, reward, done, _ = env.step_basic([motor_rpm_sp] * 4)
    utm_zone = int((np.floor((ob.gt_wgs84_pos[1] + 180) / 6) % 60) + 1)
    proj_wgs842utm = pyproj.Proj("+proj=utm +zone={} +datum=WGS84 +units=m +no_defs".format(utm_zone))

    utm_initial = np.array([*proj_wgs842utm(ob.gt_wgs84_pos[1], ob.gt_wgs84_pos[0]), ob.gt_wgs84_pos[2]])

    current_step = 0
    elevation_set_point = current_altitude + 4.7

    # spin up the motors
    while env.sim_time < 60:
        error = elevation_set_point - current_altitude
        delta_motor_rpm_sp = pid.update(env.sim_time, error)
        # delta_motor_rpm_sp = pid(current_altitude, env.stepsize)
        motor_rpm_sp += delta_motor_rpm_sp
        ob, reward, done, _ = env.step_basic([motor_rpm_sp] * 4)
        # utm_current = proj_wgs84(ob.gt_wgs84_pos[1], ob.gt_wgs84_pos[0])
        motor_rpm_actual = 10 * np.array(ob.esc_motor_angular_velocity[0:4])
        utm_current = np.array([*proj_wgs842utm(ob.gt_wgs84_pos[1], ob.gt_wgs84_pos[0]), ob.gt_wgs84_pos[2]])
        xyz_relative = utm_current - utm_initial

        if current_step % 1000 == 0:
            print("step={} height_sp={:.2f} h={:.2f} ".format(current_step, elevation_set_point, current_altitude) +
                  "set_point_rpm={:.2f} actual_rpm={:.2f} ".format(motor_rpm_sp, motor_rpm_actual[0]) +
                  "delta_rpm_sp={:.2f} rpm ".format(delta_motor_rpm_sp / env.stepsize) +
                  "(x,y,z)=({:.3f}, {:.3f}, {:.3f})".format(*xyz_relative))
        # if done:
        #     break
        current_altitude = ob.gt_wgs84_pos[2]
        current_step += 1

    env.close()

    # plt.plot(x, y, label='measured')
    # plt.plot(x, setpoint, label='target')
    # plt.xlabel('time')
    # plt.ylabel('temperature')
    # plt.legend()
    # if os.getenv('NO_DISPLAY'):
    #     # If run in CI the plot is saved to file instead of shown to the user
    #     plt.savefig(f"result-py{'.'.join([str(x) for x in sys.version_info[:2]])}.png")
    # else:
    #     plt.show()
