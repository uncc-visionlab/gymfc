#!/usr/bin/env python

import argparse
import os
import sys
import time
import matplotlib.pyplot as plt
import numpy as np
import pyproj
from scipy.spatial.transform import Rotation

import gym
from base import BaseEnv


def state_fn_error(local):
    state_error = np.zeros(9, dtype=np.float32)
    return state_error


class StepEnv(BaseEnv):
    def __init__(self, pulse_width=1, max_rate=100, state_fn=None, max_sim_time=1):
        super().__init__(max_sim_time=max_sim_time, state_fn=state_fn)

    def reset(self, seed=None, options=None):
        return super().reset(seed, options)

    def sample_target(self):
        """Sample a random angular velocity setpoint """
        return self.np_random.normal(0, self.max_rate, size=3)


class PhysicalModel:
    def __init__(self, mass):
        self.num_motors = 4  # number of motors on the vehicle
        self.gravity = 9.8  # acceleration due to gravity, m/s^2
        # self.kt = 1e-7  # constant to convert motor rotational speed into thrust (T=kt*omega^2), N/(rpm)^2
        # self.kt = mass * self.gravity / (self.num_motors * 800)
        self.kt = 1e-06
        # self.b_prop = 1e-9  # constant to convert motor speed to torque (torque = b*omega^2), (N*m)/(rpm)^2
        # self.b_prop = 0.06
        self.b_prop = 1e-6 * 0.000175  # motor constant * rotor drag coefficient
        # Air density at 25 deg C is about 1.1839 kg/m^3 At 20 °C and 101.325 kPa, dry air has density 1.2041 kg/m3
        # self.density = 1.225  # air density, kg/m^3
        self.density = 1.19  # air density, kg/m^3
        self.mass = mass  # total mass of the vehicle, kg
        # self.Ixx = 8.11858e-5  # mass-moment of inertial about x-axis, kg-m^2
        # self.Iyy = 8.11858e-5  # mass-moment of inertial about y-axis, kg-m^2
        # self.Izz = 6.12233e-5  # mass-moment of inertial about z-axis, kg-m^2
        self.Ixx = 0.029125  # mass-moment of inertial about x-axis, kg-m^2
        self.Iyy = 0.029125  # mass-moment of inertial about y-axis, kg-m^2
        self.Izz = 0.055225  # mass-moment of inertial about z-axis, kg-m^2
        self.A_ref = 0.02  # reference area for drag calcs, m^2
        # self.L = 0.2  # length from body center to prop center, m
        self.L = 0.25554  # length from body center to prop center, m
        self.Cd = 1  # drag coefficient
        self.thrust = mass * self.gravity
        # initial speeds of motors
        # self.speeds = np.ones(self.num_motors * ((mass * self.gravity) / (self.kt * self.num_motors)))
        self.speeds = np.zeros((4, 1))
        self.tau = np.zeros(3)
        self.MAX_RPM = 1100
        self.MIN_RPM = 0
        # self.maxT = 16.5  # max thrust from any single motor, N
        # self.minT = .5  # min thrust from any single motor, N
        # self.max_angle = np.pi / 12  # radians, max angle allowed at any time step
        self.max_angle = np.pi / 36  # radians, max angle allowed at any time step

        self.I = np.array([[self.Ixx, 0, 0], [0, self.Iyy, 0], [0, 0, self.Izz]])
        self.g = np.array([0, 0, -self.gravity])


if __name__ == '__main__':
    default_model = "../gazebo/models/primitive_obj/sphere.sdf"
    # default_model = "sphere.sdf"
    default_config = "physics_testbench.ini"
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

    SIM_DURATION = 60

    # Need to set the aircraft model after we create the environment because
    # the model is unique and can't be hardcoded in the gym init.
    # env = gym.make(args.gym_id)
    gym.envs.register(
        id="gymfc_physics_testbench_env",
        entry_point='physics_testbench:StepEnv',
        kwargs={
            "state_fn": state_fn_error,
        }
    )
    env = gym.make("gymfc_physics_testbench_env")
    env.setup(args.model, args.gymfc_config)
    env.seed(args.seed)
    env.reset()
    if render:
        env.render()
    env.verbose = verbose
    env.max_sim_time = SIM_DURATION

    # motor_rpm_sp = np.array([START_MOTOR_RPM] * 4)
    motor_rpm_sp = []

    # let the quad settle onto the ground and spin up the motors
    while env.sim_time < 2:
        ob, reward, done, _ = env.step_basic(motor_rpm_sp)

    utm_zone = int((np.floor((ob.gt_wgs84_pos[1] + 180) / 6) % 60) + 1)
    proj_wgs842utm = pyproj.Proj("+proj=utm +zone={} +datum=WGS84 +units=m +no_defs".format(utm_zone))

    utm_initial = np.array([*proj_wgs842utm(ob.gt_wgs84_pos[1], ob.gt_wgs84_pos[0]), ob.gt_wgs84_pos[2]])
    utm_current = utm_initial

    current_step = 0

    orientation_rpy_ob = np.array([0, 0, 0])
    orientation_rpy_ob = Rotation.from_quat(ob.gt_attitude_quat).as_euler("xyz", degrees=False)
    orientation_rpy_previous = orientation_rpy_ob
    velocity_rpy_previous = np.array([0, 0, 0])
    acceleration_rpy_previous = np.array([0, 0, 0])

    position_xyz_initial = utm_current - utm_initial
    position_xyz_ob = position_xyz_initial
    position_xyz_previous = position_xyz_ob
    velocity_xyz_previous = np.array([0, 0, 0])
    acceleration_xyz_previous = np.array([0, 0, 0])

    Quadcopter = PhysicalModel(mass=1.5)

    # start of PID control loop
    while env.sim_time < SIM_DURATION:
        ob, reward, done, _ = env.step_basic(motor_rpm_sp)

        utm_current = np.array([*proj_wgs842utm(ob.gt_wgs84_pos[1], ob.gt_wgs84_pos[0]), ob.gt_wgs84_pos[2]])

        position_xyz_ob = utm_current - utm_initial
        velocity_xyz_ob = (position_xyz_ob - position_xyz_previous) / env.stepsize
        acceleration_xyz_ob = (velocity_xyz_ob - velocity_xyz_previous) / env.stepsize

        # xyz returns roll, pitch, yaw to assign the rpy vector
        orientation_rpy_ob = Rotation.from_quat(ob.gt_attitude_quat).as_euler("xyz", degrees=False)
        velocity_rpy_ob = (orientation_rpy_ob - orientation_rpy_previous) / env.stepsize
        acceleration_rpy_ob = (velocity_rpy_ob - velocity_rpy_previous) / env.stepsize

        position_xyz_previous = position_xyz_ob
        velocity_xyz_previous = velocity_xyz_ob
        orientation_rpy_previous = orientation_rpy_ob
        velocity_rpy_previous = velocity_rpy_ob

        if current_step % 1000 == 0:
            print("step={} ".format(current_step) +
                  "(x,y,z)_ob=({:.2f}, {:.2f}, {:.2f}) ".format(*position_xyz_ob) +
                  "(x',y',z')_ob=({:.2f}, {:.2f}, {:.2f}) ".format(*velocity_xyz_ob) +
                  "(x'',y'',z'')_ob=({:.2f}, {:.2f}, {:.2f}) ".format(*acceleration_xyz_ob) +
                  "(r,p,y)_ob=({:.1f}, {:.1f}, {:.1f})".format(*[av * 180 / np.pi for av in orientation_rpy_ob]) +
                  "(r',p',y')_ob=({:.1f}, {:.1f}, {:.1f})".format(*[av * 180 / np.pi for av in velocity_rpy_ob]) +
                  "(r'',p'',y'')_ob=({:.1f}, {:.1f}, {:.1f})".format(
                      *[av * 180 / np.pi for av in acceleration_rpy_ob_rpy_ob]))
        # if done:
        #     break

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
