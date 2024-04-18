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
import asyncio
from rover import Rover


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


if __name__ == '__main__':
    # default_model = "../gazebo/models/iris/model.sdf"
    default_model = "../../../gazebo/models/mc_simple/uav.sdf"
    default_config = "gymfc_mc_simple.ini"
    parser = argparse.ArgumentParser("Test bench for gazebo physics engines")
    parser.add_argument('--model', default=default_model,
                        help="File path of the NF1 twin/model SDF.")
    parser.add_argument('--gymfc-config', default=default_config,
                        help="Option to override default GymFC configuration location.")
    parser.add_argument('--gym-id', default="gymfc_nf-step-v1")
    parser.add_argument('--seed', help='RNG seed', type=int, default=0)
    args = parser.parse_args()

    verbose = True
    render = True

    SIM_DURATION = 15

    # Need to set the aircraft model after we create the environment because
    # the model is unique and can't be hardcoded in the gym init.
    # env = gym.make(args.gym_id)
    gym.envs.register(
        id="gymfc_mc_simple_env",
        entry_point='gymfc_mc_simple:StepEnv',
        kwargs={
            "state_fn": state_fn_error,
        }
    )
    env = gym.make("gymfc_mc_simple_env")
    env.setup(args.model, args.gymfc_config)
    env.seed(args.seed)
    env.reset()
    if render:
        env.render()
    env.verbose = verbose
    env.max_sim_time = SIM_DURATION

    force_and_moment = [0.0] * 6

    quadcopter = Rover()
    quadcopter.motor_on = True
    quadcopter.mode = 0  # Idle

    # let the quad settle onto the ground and spin up the motors
    # let GPS, IMU and other messages populate into the observation message at their configured rate
    while env.sim_time < 1:
        ob, reward, done, _ = env.step_basic(force_and_moment)

    utm_zone = int((np.floor((ob.gt_wgs84_pos[1] + 180) / 6) % 60) + 1)
    proj_wgs842utm = pyproj.Proj("+proj=utm +zone={} +datum=WGS84 +units=m +no_defs".format(utm_zone))
    utm_initial_gt = np.array([*proj_wgs842utm(ob.gt_wgs84_pos[1], ob.gt_wgs84_pos[0]), ob.gt_wgs84_pos[2]])
    utm_initial_gps = np.array([*proj_wgs842utm(ob.gps_wgs84_pos[1], ob.gps_wgs84_pos[0]), ob.gps_wgs84_pos[2]])

    orientation_rpy_ob = Rotation.from_quat(ob.gt_attitude_quat).as_euler("xyz", degrees=False)
    orientation_rpy_previous = orientation_rpy_ob

    imu_update_time_previous = env.sim_time
    gps_update_time_previous = env.sim_time
    control_update_time_previous = env.sim_time

    position_xyz_gt_previous = np.zeros(3)
    position_xyz_gps_previous = np.zeros(3)
    current_step = 0

    imu_ang_vel_avg = np.zeros(3)
    imu_lin_acc_avg = np.zeros(3)
    num_imu_readings = 0

    dt_control = 0
    dt_gps = 0
    dt_imu = 0

    while env.sim_time < SIM_DURATION:
        if 1 < env.sim_time < 3:
            quadcopter.mode = 1  # Warm-up
        elif 3 <= env.sim_time < 5:
            quadcopter.mode = 2  # Take-off
        elif 5 <= env.sim_time < 15:
            quadcopter.mode = 5  # Land
        # if quadcopter.trajectory.manual_mode:
        #     quadcopter.mode = 5
        # print("Commanding force and moment {}".format(force_and_moment))
        ob, reward, done, _ = env.step_basic(force_and_moment)

        dt_imu = env.sim_time - imu_update_time_previous
        dt_gps = env.sim_time - gps_update_time_previous
        dt_control = env.sim_time - control_update_time_previous
        current_step += 1

        # convert ground truth wgs84 coordinate to a UTM coordinate
        utm_current_gt = np.array([*proj_wgs842utm(ob.gt_wgs84_pos[1], ob.gt_wgs84_pos[0]), ob.gt_wgs84_pos[2]])
        utm_current_gps = np.array([*proj_wgs842utm(ob.gps_wgs84_pos[1], ob.gps_wgs84_pos[0]), ob.gps_wgs84_pos[2]])
        # convert the UTM coordinate to a local relative coordinate
        position_xyz_gt = utm_current_gt - utm_initial_gt
        position_xyz_gps = utm_current_gps - utm_initial_gps
        # position_xyz_gt = np.array(ob.gt_wgs84_pos)

        # velocity_xyz_gt = (position_xyz_gt - position_xyz_gt_previous) / env.stepsize
        # position_xyz_gt_previous = position_xyz_gt

        # xyz returns roll, pitch, yaw to assign the rpy vector
        orientation_rpy_ob = Rotation.from_quat(ob.gt_attitude_quat).as_euler("xyz", degrees=False)
        velocity_rpy_ob = (orientation_rpy_ob - orientation_rpy_previous) / env.stepsize

        # if current_step % 1000 == 0:
        #     print("step={} ".format(current_step) +
        #           "fm_sp=({:.1f},{:.1f},{:.1f},{:.1f}) ".format(*force_and_moment[2:6]) +
        #           "(x,y,z)=({:.2f}, {:.2f}, {:.2f}) ".format(*position_xyz_gt) +
        #           "(x',y',z')=({:.2f}, {:.2f}, {:.2f}) ".format(*velocity_xyz_gt) +
        #           "(r,p,y)=({:.2f}, {:.2f}, {:.2f}) ".format(*[av * 180 / np.pi for av in orientation_rpy_ob]) +
        #           "(r',p',y')=({:.1f}, {:.1f}, {:.1f})".format(*[av * 180 / np.pi for av in velocity_rpy_ob]))

        if dt_imu > 0.01:
            # print("Got a IMU update at time {}".format(env.sim_time))
            quadcopter.gymfc_imu_callback(ob.imu_orientation_quat, ob.imu_angular_velocity_rpy,
                                          ob.imu_linear_acceleration_xyz, dt_imu)
            imu_update_time_previous = env.sim_time

        if dt_gps > 0.1:
            # print("Got a GPS update at time {}.".format(env.sim_time))
            velocity_xyz_gt = (position_xyz_gt - position_xyz_gt_previous) / dt_gps
            position_xyz_gt_previous = position_xyz_gt
            quadcopter.gymfc_gps_callback(position_xyz_gt, velocity_xyz_gt)
            # quadcopter.gymfc_gps_callback(ob.gt_wgs84_pos, ob.gt_enu_vel)
            gps_update_time_previous = env.sim_time

        if dt_control > 0.005:
            fM = quadcopter.run_controller(env.sim_time, dt_control)
            fM = fM.flatten()
            if (not quadcopter.motor_on) or (quadcopter.mode < 2):
                force_and_moment = [0.0] * 6
            else:
                force_and_moment = [0.0, 0.0, fM[0], fM[1], fM[2], fM[3]]
            control_update_time_previous = env.sim_time

    env.close()
