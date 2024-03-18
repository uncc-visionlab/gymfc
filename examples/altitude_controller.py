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
from gym.envs.registration import register
from gymfc_nf.envs.base import BaseEnv


def c(x): return np.cos(x)  # ''' cosine of angle in radians'''


def s(x): return np.sin(x)  # ''' sine of angle in radians'''


def t(x): return np.tan(x)  # ''' tangent of angle in radians'''


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


class Quadcopter():
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


def _clamp(value, limits):
    lower, upper = limits
    if value is None:
        return None
    elif (upper is not None) and (value > upper):
        return upper
    elif (lower is not None) and (value < lower):
        return lower
    return value


class SimplePID(object):

    def __init__(self, kp, ki, kd, ki_sat=None):
        self.dimension = len(kp) if hasattr(kp, "__len__") else 1
        self.kp = np.array(kp).flatten()
        self.ki = np.array(ki).flatten()
        self.kd = np.array(kd).flatten()
        if ki_sat is None:
            self.ki_sat = np.array([[-float("inf"), float("inf")] * self.dimension])
        else:
            self.ki_sat = ki_sat
        self.last_t = 0
        self.last_e = 0
        self.accumulator = [0] * self.dimension
        self.reset()

    def update(self, delta_t, e):
        # Most environments have a short execution time the controller doesn't have much time to wind up
        e = np.array(e).flatten()
        p_term = self.kp * e
        self.accumulator += e * delta_t
        i_terms = self.ki * self.accumulator
        i_term = [_clamp(i_terms[index], sat) for index, sat in enumerate(self.ki_sat)]
        de = e - self.last_e
        self.last_e = e
        d_term = self.kd * de / delta_t if delta_t > 0 else 0
        return p_term + i_term + d_term

    def reset(self):
        self.last_t = 0
        self.last_e = 0
        self.accumulator = [0] * self.dimension


if __name__ == '__main__':
    # default_model = "../gazebo/models/iris/iris_altitude.sdf"
    default_model = "../gazebo/models/iris/model.sdf"
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

    SDF_SLOWDOWN_PARAM = 10
    SIM_DURATION = 60
    YAW_SET_POINT = 0  # + np.pi / 2
    START_MOTOR_RPM = 790
    USE_WAYPOINTS = True

    position_xyz_sp = np.array([10, 10, 1.7])

    if USE_WAYPOINTS:
        waypoint_list = np.array([[10, 0, 1.7], [10, 10, 2.7], [-10, -10, 0.7], [0, 0, 0.1]])
        transition_times = np.arange(0, SIM_DURATION, SIM_DURATION / waypoint_list.shape[0])
        current_waypoint_index = 0
        position_xyz_sp = waypoint_list[current_waypoint_index, :]


    def update_waypoint(td_env, td_transition_times, td_waypoint_list):
        global current_waypoint_index, position_xyz_sp
        if (current_waypoint_index < td_waypoint_list.shape[0] and
                td_env.sim_time > td_transition_times[current_waypoint_index]):
            position_xyz_sp = td_waypoint_list[current_waypoint_index, :]
            print("New position set point assigned (x,y,z)_sp=({:.2f}, {:.2f}, {:.2f}) ".format(*position_xyz_sp))
            current_waypoint_index += 1


    # Need to set the aircraft model after we create the environment because
    # the model is unique and can't be hardcoded in the gym init.
    # env = gym.make(args.gym_id)
    gym.envs.register(
        id="gymfc_quadrotor_env_v1",
        entry_point='altitude_controller:StepEnv',
        kwargs={
            "state_fn": state_fn_error,
        }
    )
    env = gym.make("gymfc_quadrotor_env_v1")
    env.setup(args.model, args.gymfc_config)
    env.seed(args.seed)
    env.reset()
    if render:
        env.render()
    env.verbose = verbose
    env.max_sim_time = SIM_DURATION

    # 3DR Iris Vehicle

    # Gains for position controller
    Kp_pos = [0.5, 0.5, 0.5]  # proportional [x,y,z]
    Kd_pos = [0, 0, 0]  # derivative [x,y,z]
    Ki_pos = [0.25, 0.25, 0.25]  # integral [x,y,z]
    Ki_sat_pos = np.tile(np.array([-1.1, 1.1]), (3, 1))  # saturation for integral controller (prevent windup) [x,y,z]

    # Gains for angle controller
    Kp_ang = [4.0e-2, 4.0e-2, 4.0e-5]  # proportional [x,y,z]
    Kd_ang = [0, 0, 0]  # derivative [x,y,z]
    Ki_ang = [12.0e-3, 12.0e-3, 12.0e-6]  # integral [x,y,z]
    Ki_sat_ang = np.tile(np.array([[-0.1, 0.1]]), (3, 1))  # saturation for integral controller (prevent windup) [x,y,z]

    position_xyz_controller = SimplePID(Kp_pos, Kd_pos, Ki_pos, Ki_sat_pos)
    orientation_rpy_controller = SimplePID(Kp_ang, Kd_ang, Ki_ang, Ki_sat_ang)

    motor_rpm_sp = np.array([START_MOTOR_RPM] * 4)
    #  NF 1 Vehicle
    # pid = SimplePID(kp=1e-6, ki=0.000000, kd=1e-10)

    # let the quad settle onto the ground and spin up the motors
    while env.sim_time < 2:
        ob, reward, done, _ = env.step_basic(motor_rpm_sp)
        if USE_WAYPOINTS:
            update_waypoint(env, transition_times, waypoint_list)

    utm_zone = int((np.floor((ob.gt_wgs84_pos[1] + 180) / 6) % 60) + 1)
    proj_wgs842utm = pyproj.Proj("+proj=utm +zone={} +datum=WGS84 +units=m +no_defs".format(utm_zone))

    utm_initial = np.array([*proj_wgs842utm(ob.gt_wgs84_pos[1], ob.gt_wgs84_pos[0]), ob.gt_wgs84_pos[2]])
    utm_current = utm_initial

    current_step = 0

    orientation_rpy_ob = np.array([0, 0, 0])
    orientation_rpy_ob = Rotation.from_quat(ob.gt_attitude_quat).as_euler("xyz", degrees=False)
    orientation_rpy_previous = orientation_rpy_ob

    position_xyz_initial = utm_current - utm_initial
    position_xyz_ob = position_xyz_initial
    position_xyz_previous = position_xyz_ob

    orientation_rpy_sp = np.array([0, 0, 0])
    velocity_xyz_sp = np.array([0, 0, 0])
    # velocity_rpy_sp = np.array([0, 0, 0])

    Quadcopter = Quadcopter(mass=1.5)

    # start of PID control loop
    while env.sim_time < SIM_DURATION:
        ob, reward, done, _ = env.step_basic(motor_rpm_sp)
        if USE_WAYPOINTS:
            update_waypoint(env, transition_times, waypoint_list)

        motor_rpm_ob = SDF_SLOWDOWN_PARAM * np.array(ob.esc_motor_angular_velocity[0:4])

        utm_current = np.array([*proj_wgs842utm(ob.gt_wgs84_pos[1], ob.gt_wgs84_pos[0]), ob.gt_wgs84_pos[2]])
        position_xyz_ob = utm_current - utm_initial
        # xyz returns roll, pitch, yaw to assign the rpy vector
        orientation_rpy_ob = Rotation.from_quat(ob.gt_attitude_quat).as_euler("xyz", degrees=False)

        velocity_xyz_ob = (position_xyz_ob - position_xyz_previous) / env.stepsize
        velocity_rpy_ob = (orientation_rpy_ob - orientation_rpy_previous) / env.stepsize
        position_xyz_previous = position_xyz_ob
        orientation_rpy_previous = orientation_rpy_ob

        position_xyz_error = position_xyz_sp - position_xyz_ob
        # velocity_xyz_error = velocity_xyz_sp - velocity_xyz_current

        #  Calculate Quadcopter dynamics
        thrust = Quadcopter.kt * np.sum(motor_rpm_ob)
        R_b2i = Rotation.from_quat(ob.gt_attitude_quat).as_matrix()  # changes from body frame to inertial frame
        R_i2b = R_b2i.T  # changes from inertial frame to body frame

        # Linear and angular accelerations in inertial frame
        # find_lin_acc()
        thrust_body = np.array([0, 0, thrust])
        thrust_inertial = R_b2i @ thrust_body
        velocity_body = R_i2b @ velocity_xyz_ob
        drag_body = -Quadcopter.Cd * 0.5 * Quadcopter.density * Quadcopter.A_ref * velocity_body ** 2
        drag_inertial = R_b2i @ drag_body
        # weight = Quadcopter.mass * Quadcopter.g
        # acceleration_xyz_ob = (thrust_inertial + drag_inertial + weight) / Quadcopter.mass

        # torque on body from motor thrust differential
        # find_body_torque()
        L = Quadcopter.L
        kt = Quadcopter.kt
        b_prop = Quadcopter.b_prop

        tau_ob = np.array([(L * kt * (-motor_rpm_ob[0] + motor_rpm_ob[1] + motor_rpm_ob[2] - motor_rpm_ob[3])),
                           (L * kt * (-motor_rpm_ob[0] + motor_rpm_ob[1] - motor_rpm_ob[2] + motor_rpm_ob[3])),
                           (b_prop * (-motor_rpm_ob[0] - motor_rpm_ob[1] + motor_rpm_ob[2] + motor_rpm_ob[3]))])

        # Angular acceleration in inertial frame
        # omega = self.thetadot2omega()  #angles in inertial frame
        # omega_dot = self.find_omegadot(omega) #angular acceleration in inertial frame
        R = np.array([[1, 0, -s(orientation_rpy_ob[1])],
                      [0, c(orientation_rpy_ob[0]), c(orientation_rpy_ob[1]) * s(orientation_rpy_ob[0])],
                      [0, -s(orientation_rpy_ob[0]), c(orientation_rpy_ob[1]) * c(orientation_rpy_ob[0])]])
        omega = R @ velocity_rpy_ob
        Imat = Quadcopter.I
        omega_dot = np.linalg.inv(Imat).dot(tau_ob - np.cross(omega, Imat @ omega))

        # Angular acceleration in body frame
        # self.omegadot2Edot(omega_dot)
        R = np.array([[1, s(orientation_rpy_ob[0]) * t(orientation_rpy_ob[1]),
                       c(orientation_rpy_ob[0]) * t(orientation_rpy_ob[1])],
                      [0, c(orientation_rpy_ob[0]), -s(orientation_rpy_ob[0])],
                      [0, s(orientation_rpy_ob[0]) / c(orientation_rpy_ob[1]),
                       c(orientation_rpy_ob[0]) / c(orientation_rpy_ob[1])]])

        E_dot = R @ omega_dot
        acceleration_rpy_current = E_dot

        if current_step % 1000 == 0:
            print("step={} ".format(current_step) +
                  "rpm_sp=({:.1f},{:.1f},{:.1f},{:.1f}) ".format(*motor_rpm_sp) +
                  "rpm_ob=({:.1f},{:.1f},{:.1f},{:.1f}) ".format(*motor_rpm_ob) +
                  "(x,y,z)_sp=({:.2f}, {:.2f}, {:.2f}) ".format(*position_xyz_sp) +
                  "(x,y,z)_ob=({:.2f}, {:.2f}, {:.2f}) ".format(*position_xyz_ob) +
                  "(r,p,y)_sp=({:.2f}, {:.2f}, {:.2f}) ".format(*[av * 180 / np.pi for av in orientation_rpy_sp]) +
                  "(r,p,y)_ob=({:.1f}, {:.1f}, {:.1f})".format(*[av * 180 / np.pi for av in orientation_rpy_ob]))
        # if done:
        #     break

        # Compute position control
        acceleration_xyz_sp = position_xyz_controller.update(env.stepsize, position_xyz_error)

        # Modify z gain to include thrust required to hover
        # acceleration_xyz_sp[2] = (Quadcopter.gravity +
        #                           acceleration_xyz_sp[2]) / (c(orientation_rpy_ob[0]) * c(orientation_rpy_ob[1]))

        # Calculate thrust needed
        thrust_sp = Quadcopter.mass * acceleration_xyz_sp[2]

        # Check if needed acceleration is not zero. if zero, set to one to prevent divide by zero below
        acceleration_xyz_magnitude = np.linalg.norm(acceleration_xyz_sp)
        if acceleration_xyz_magnitude == 0:
            acceleration_xyz_magnitude = 1

        # roll changes Y coordinate, pitch changes X coordinate, yaw stays at 0
        # use desired acceleration to find desired angles since the quad can only move via changing angles
        sin_roll_angle_sp = -acceleration_xyz_sp[1] / acceleration_xyz_magnitude / np.cos(orientation_rpy_ob[1])
        if np.abs(sin_roll_angle_sp) > 1:
            sin_roll_angle_sp /= np.abs(sin_roll_angle_sp)
        orientation_rpy_sp = np.array([
            np.arcsin(sin_roll_angle_sp),
            np.arcsin(acceleration_xyz_sp[0] / acceleration_xyz_magnitude), YAW_SET_POINT])

        # check if exceeds max angle
        orientation_rpy_magnitude = np.linalg.norm(orientation_rpy_sp[:2])
        if orientation_rpy_magnitude > Quadcopter.max_angle:
            orientation_rpy_sp[:2] = (orientation_rpy_sp[:2] / orientation_rpy_magnitude) * Quadcopter.max_angle

        # call angle controller
        orientation_rpy_error = orientation_rpy_sp - orientation_rpy_ob
        if orientation_rpy_error[2] > Quadcopter.max_angle / 10:
            orientation_rpy_error[2] = Quadcopter.max_angle / 10
        # velocity_rpy_error = velocity_rpy_sp - velocity_rpy_current

        tau_sp = orientation_rpy_controller.update(env.stepsize, orientation_rpy_error)

        # Find motor speeds needed to achieve desired linear and angular accelerations
        # quadcopter.des2speeds(thrust_needed, tau_needed)
        # ''' finds speeds of motors to achieve a desired thrust and torque '''
        # Needed torque on body (e1,e2,e3) = torque on (x,y,z) axes = (roll, pitch, yaw)
        e1 = tau_sp[0] * Quadcopter.Ixx
        e2 = tau_sp[1] * Quadcopter.Iyy
        e3 = tau_sp[2] * Quadcopter.Izz
        # less typing
        n = Quadcopter.num_motors
        # Thrust desired converted into motor speeds
        weight_speed = thrust_sp / (n * Quadcopter.kt)
        # Allocate thrust to the motors to achieve needed thrust and torque on body
        motor_rpm_sp = [
            # THRUST     # ROLL CONTROL COLUMN # PITCH CONTROL COLUMN # YAW CONTROL COLUMN
            weight_speed - (e1 / (n * kt * L)) - (e2 / (n * kt * L)) - (e3 / (n * b_prop)),
            weight_speed + (e1 / (n * kt * L)) + (e2 / (n * kt * L)) - (e3 / (n * b_prop)),
            weight_speed + (e1 / (n * kt * L)) - (e2 / (n * kt * L)) + (e3 / (n * b_prop)),
            weight_speed - (e1 / (n * kt * L)) + (e2 / (n * kt * L)) + (e3 / (n * b_prop))]

        if (np.any(np.isnan(motor_rpm_sp)) is True
                or (max(motor_rpm_sp) - min(motor_rpm_sp)) / max(motor_rpm_sp) > 0.1):
            test_condition = True
        MAX_RPM = Quadcopter.MAX_RPM
        MIN_RPM = Quadcopter.MIN_RPM
        # Ensure that desired speed is within overall min and max of all motors
        motor_rpm_sp = [_clamp(motor_speed_rpm, [MIN_RPM, MAX_RPM]) for motor_speed_rpm in motor_rpm_sp]
        thrust_all = np.array(motor_rpm_sp) * kt
        total_thrust = np.sum(thrust_all)

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
