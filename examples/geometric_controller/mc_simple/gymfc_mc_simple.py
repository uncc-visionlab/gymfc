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

    SIM_DURATION = 60

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

    # motor_rpm_sp = np.array([START_MOTOR_RPM] * 4)
    force_and_moment = [0.0] * 6

    quadcopter = Rover()
    quadcopter.motor_on = True
    quadcopter.mode = 0  # Idle

    # let the quad settle onto the ground and spin up the motors
    # let GPS, IMU and other messages to populate into the observation message per their configured rate
    while env.sim_time < 1:
        ob, reward, done, _ = env.step_basic(force_and_moment)

    utm_zone = int((np.floor((ob.gt_wgs84_pos[1] + 180) / 6) % 60) + 1)
    proj_wgs842utm = pyproj.Proj("+proj=utm +zone={} +datum=WGS84 +units=m +no_defs".format(utm_zone))
    utm_initial_gt = np.array([*proj_wgs842utm(ob.gt_wgs84_pos[1], ob.gt_wgs84_pos[0]), ob.gt_wgs84_pos[2]])
    utm_initial_gps = np.array([*proj_wgs842utm(ob.gps_wgs84_pos[1], ob.gps_wgs84_pos[0]), ob.gps_wgs84_pos[2]])

    imu_update_time = 0.0
    imu_update_time_previous = 0.0
    gps_update_time = 0.0
    gps_update_time_previous = 0.0

    position_xyz_gt_previous = np.zeros(3)
    position_xyz_gps_previous = np.zeros(3)

    while env.sim_time < 20:
        if 1 < env.sim_time < 3:
            quadcopter.mode = 1  # Warm-up
        elif 3 <= env.sim_time < 7:
            quadcopter.mode = 2  # Take-off
        elif 7 <= env.sim_time < 12:
            quadcopter.mode = 3  # Land
        ob, reward, done, _ = env.step_basic(force_and_moment)

        # convert ground truth wgs84 coordinate to a UTM coordinate
        utm_current_gt = np.array([*proj_wgs842utm(ob.gt_wgs84_pos[1], ob.gt_wgs84_pos[0]), ob.gt_wgs84_pos[2]])
        utm_current_gps = np.array([*proj_wgs842utm(ob.gps_wgs84_pos[1], ob.gps_wgs84_pos[0]), ob.gps_wgs84_pos[2]])
        # convert the UTM coordinate to a local relative coordinate
        position_xyz_gt = utm_current_gt - utm_initial_gt
        position_xyz_gps = utm_current_gps - utm_initial_gps

        # detect a GPS update
        gps_update = np.linalg.norm(position_xyz_gps - position_xyz_gps_previous) > 1.0e-4
        imu_update = True  # imu updates come on each GymFC iteration

        velocity_xyz_gt = (position_xyz_gt - position_xyz_gt_previous) / env.stepsize
        velocity_xyz_gps = (position_xyz_gps - position_xyz_gps_previous) / env.stepsize

        position_xyz_gt_previous = position_xyz_gt
        position_xyz_gps_previous = position_xyz_gps

        if imu_update:
            imu_update_time = env.sim_time
            dt_imu = imu_update_time - imu_update_time_previous
            if dt_imu > 0 and imu_update_time_previous > 0:
                quadcopter.gymfc_imu_callback(ob.imu_orientation_quat, ob.imu_angular_velocity_rpy,
                                              ob.imu_linear_acceleration_xyz, dt_imu)
            imu_update_time_previous = imu_update_time

        # if gps_update:
        #     gps_update_time = env.sim_time
        #     dt_gps = gps_update_time - gps_update_time_previous
        #     if dt_gps > 0 and gps_update_time_previous > 0:
        #         quadcopter.gymfc_gps_callback(position_xyz_gt, velocity_xyz_gt)
        #     gps_update_time_previous = gps_update_time

        fM = quadcopter.run_controller()
        fM = fM.flatten()
        if (not quadcopter.motor_on) or (quadcopter.mode < 2):
            force_and_moment = [0.0] * 6
        else:
            force_and_moment = [0.0, 0.0, fM[0], fM[1], fM[2], fM[3]]

    env.close()
