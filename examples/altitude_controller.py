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
        self.kt = 1e-7  # constant to convert motor rotational speed into thrust (T=kt*omega^2), N/(rpm)^2
        self.kt = mass * self.gravity / (self.num_motors * 795)
        self.b_prop = 1e-9  # constant to convert motor speed to torque (torque = b*omega^2), (N*m)/(rpm)^2
        self.density = 1.225  # air density, kg/m^3
        self.mass = mass  # total mass of the vehicle, kg
        self.Ixx = 8.11858e-5  # mass-moment of inertial about x-axis, kg-m^2
        self.Iyy = 8.11858e-5  # mass-moment of inertial about y-axis, kg-m^2
        self.Izz = 6.12233e-5  # mass-moment of inertial about z-axis, kg-m^2
        self.A_ref = 0.02  # reference area for drag calcs, m^2
        self.L = 0.2  # length from body center to prop center, m
        self.Cd = 1  # drag coefficient
        self.thrust = mass * self.gravity
        # initial speeds of motors
        # self.speeds = np.ones(self.num_motors * ((mass * self.gravity) / (self.kt * self.num_motors)))
        self.speeds = np.zeros((4, 1))
        self.tau = np.zeros(3)

        self.maxT = 16.5  # max thrust from any single motor, N
        self.minT = .5  # min thrust from any single motor, N
        self.max_angle = np.pi / 12  # radians, max angle allowed at any time step

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
    SDF_SLOWDOWN_PARAM = 10
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

    # 3DR Iris Vehicle
    pid = SimplePID(kp=1e-4, ki=0.000000, kd=1e-2)

    # Gains for position controller
    # Kp_pos = [.95, .95, 15.]  # proportional [x,y,z]
    # Kd_pos = [1.8, 1.8, 15.]  # derivative [x,y,z]
    # Ki_pos = [0.2, 0.2, 1.0]  # integral [x,y,z]
    Kp_pos = [.95, .95, 15.]  # proportional [x,y,z]
    Kd_pos = [0, 0, 0]  # derivative [x,y,z]
    Ki_pos = [0, 0, 0]  # integral [x,y,z]
    Ki_sat_pos = np.tile(np.array([-1.1, 1.1]), (3, 1))  # saturation for integral controller (prevent windup) [x,y,z]

    # Gains for angle controller
    Kp_ang = [6.9, 6.9, 25.]  # proportional [x,y,z]
    Kd_ang = [0, 0, 0]  # derivative [x,y,z]
    Ki_ang = [0, 0, 0]  # integral [x,y,z]
    Ki_sat_ang = np.tile(np.array([[-0.1, 0.1]]), (3, 1))  # saturation for integral controller (prevent windup) [x,y,z]

    position_xyz_controller = SimplePID(Kp_pos, Kd_pos, Ki_pos, Ki_sat_pos)
    orientation_rpy_controller = SimplePID(Kp_ang, Kd_ang, Ki_ang, Ki_sat_ang)

    motor_rpm_sp_list = [790] * 4
    #  NF 1 Vehicle
    # pid = SimplePID(kp=1e-6, ki=0.000000, kd=1e-10)
    # motor_rpm_sp = 850

    pid.output_limits = (-1000, 1000)

    # let the quad settle onto the ground
    while env.sim_time < 4:
        ob, reward, done, _ = env.step_basic(motor_rpm_sp_list)
    utm_zone = int((np.floor((ob.gt_wgs84_pos[1] + 180) / 6) % 60) + 1)
    proj_wgs842utm = pyproj.Proj("+proj=utm +zone={} +datum=WGS84 +units=m +no_defs".format(utm_zone))

    utm_initial = np.array([*proj_wgs842utm(ob.gt_wgs84_pos[1], ob.gt_wgs84_pos[0]), ob.gt_wgs84_pos[2]])
    utm_current = utm_initial

    current_step = 0

    elevation_set_point = current_altitude + 4.7

    orientation_rpy_initial = Rotation.from_quat(ob.gt_attitude_quat).as_euler("xyz")
    orientation_rpy_current = orientation_rpy_initial
    orientation_rpy_previous = orientation_rpy_current

    position_xyz_initial = utm_current - utm_initial
    position_xyz_current = position_xyz_initial
    position_xyz_previous = position_xyz_current

    position_xyz_sp = np.array([0, 0, 4.7])
    orientation_rpy_sp = np.array([0, 0, 0])
    velocity_xyz_sp = np.array([0, 0, 0])
    velocity_rpy_sp = np.array([0, 0, 0])

    Quadcopter = Quadcopter(mass=1.5)

    # spin up the motors
    while env.sim_time < 60:
        # error = elevation_set_point - current_altitude
        # delta_motor_rpm_sp = pid.update(env.sim_time, error)
        # delta_motor_rpm_sp = pid(current_altitude, env.stepsize)
        # motor_rpm_sp_list = [motor_rpm_sp + delta_motor_rpm_sp[0] for motor_rpm_sp in motor_rpm_sp_list]
        ob, reward, done, _ = env.step_basic(motor_rpm_sp_list)

        motor_rpm_actual = SDF_SLOWDOWN_PARAM * np.array(ob.esc_motor_angular_velocity[0:4])

        utm_current = np.array([*proj_wgs842utm(ob.gt_wgs84_pos[1], ob.gt_wgs84_pos[0]), ob.gt_wgs84_pos[2]])
        position_xyz_current = utm_current - utm_initial
        orientation_rpy_current = Rotation.from_quat(ob.gt_attitude_quat).as_euler("xyz")

        velocity_xyz_current = (position_xyz_current - position_xyz_previous) / env.stepsize
        velocity_rpy_current = (orientation_rpy_current - orientation_rpy_previous) / env.stepsize

        position_xyz_error = position_xyz_sp - position_xyz_current
        # velocity_xyz_error = velocity_xyz_sp - velocity_xyz_current

        #  Calculate Quadcopter dynamics
        thrust = Quadcopter.kt * np.sum(motor_rpm_actual)
        R_b2i = Rotation.from_quat(ob.gt_attitude_quat).as_matrix()  # changes from body frame to inertial frame
        R_i2b = R_b2i.T  # changes from inertial frame to body frame

        # Linear and angular accelerations in inertial frame
        # find_lin_acc()
        thrust_body = np.array([0, 0, thrust])
        thrust_inertial = R_b2i @ thrust_body
        velocity_body = R_i2b @ velocity_xyz_current
        drag_body = -Quadcopter.Cd * 0.5 * Quadcopter.density * Quadcopter.A_ref * velocity_body ** 2
        drag_inertial = R_b2i @ drag_body
        weight = Quadcopter.mass * Quadcopter.g
        acceleration_inertial = (thrust_inertial + drag_inertial + weight) / Quadcopter.mass

        # torque on body from motor thrust differential
        # find_body_torque()
        L = Quadcopter.L
        kt = Quadcopter.kt
        b_prop = Quadcopter.b_prop
        tau = np.array([(L * kt * (motor_rpm_actual[3] - motor_rpm_actual[1])),
                        (L * kt * (motor_rpm_actual[2] - motor_rpm_actual[0])),
                        (b_prop * (-motor_rpm_actual[0] + motor_rpm_actual[1] -
                                   motor_rpm_actual[2] + motor_rpm_actual[3]))])

        # Angular acceleration in inertial frame
        # omega = self.thetadot2omega()  #angles in inertial frame
        # omega_dot = self.find_omegadot(omega) #angular acceleration in inertial frame
        R = np.array([[1, 0, -s(orientation_rpy_current[1])],
                      [0, c(orientation_rpy_current[0]),
                       c(orientation_rpy_current[1]) * s(orientation_rpy_current[0])],
                      [0, -s(orientation_rpy_current[0]),
                       c(orientation_rpy_current[1]) * c(orientation_rpy_current[0])]])
        omega = R @ velocity_rpy_current
        I = Quadcopter.I
        omega_dot = np.linalg.inv(I).dot(tau - np.cross(omega, I @ omega))

        # Angular acceleration in body frame
        # self.omegadot2Edot(omega_dot)
        R = np.array([[1, s(orientation_rpy_current[0]) * t(orientation_rpy_current[1]),
                       c(orientation_rpy_current[0]) * t(orientation_rpy_current[1])],
                      [0, c(orientation_rpy_current[0]), -s(orientation_rpy_current[0])],
                      [0, s(orientation_rpy_current[0]) / c(orientation_rpy_current[1]),
                       c(orientation_rpy_current[0]) / c(orientation_rpy_current[1])]])

        E_dot = R @ omega_dot
        acceleration_rpy_current = E_dot

        if current_step % 1000 == 0:
            print("step={} height_sp={:.2f} h={:.2f} ".format(current_step, elevation_set_point, current_altitude) +
                  "set_point_rpm=({:.1f},{:.1f},{:.1f},{:.1f}) ".format(*motor_rpm_sp_list) +
                  "actual_rpm=({:.1f},{:.1f},{:.1f},{:.12f}) ".format(*motor_rpm_actual) +
                  # "delta_rpm_sp={:.1f} rpm ".format(delta_motor_rpm_sp[0] / env.stepsize) +
                  "(x,y,z)=({:.2f}, {:.2f}, {:.2f})".format(*position_xyz_current) +
                  "(r,p,y)=({:.1f}, {:.1f}, {:.1f})".format(*[arad * 180 / np.pi for arad in orientation_rpy_current]))
            test = True
        # if done:
        #     break

        # Compute control
        acceleration_xyz_sp = position_xyz_controller.update(env.stepsize, position_xyz_error)

        # Modify z gain to include thrust required to hover
        # acceleration_xyz_sp[2] = (gravity + des_acc[2]) / (math.cos(quadcopter.angle[0]) * math.cos(quadcopter.angle[1]))

        # calculate thrust needed
        thrust_sp = Quadcopter.mass * acceleration_xyz_sp[2]

        # Check if needed acceleration is not zero. if zero, set to one to prevent divide by zero below
        acceleration_xyz_magnitude = np.linalg.norm(acceleration_xyz_sp)
        if acceleration_xyz_magnitude == 0:
            acceleration_xyz_magnitude = 1

        # use desired acceleration to find desired angles since the quad can only move via changing angles
        orientation_rpy_sp = [
            np.arcsin(-acceleration_xyz_sp[1] / acceleration_xyz_magnitude / np.cos(orientation_rpy_current[1])),
            np.arcsin(acceleration_xyz_sp[0] / acceleration_xyz_magnitude), 0]

        # check if exceeds max angle
        orientation_rpy_magnitude = np.linalg.norm(orientation_rpy_sp)
        if orientation_rpy_magnitude > Quadcopter.max_angle:
            orientation_rpy_sp = (orientation_rpy_sp / orientation_rpy_magnitude) * Quadcopter.max_angle

        # call angle controller
        orientation_rpy_error = orientation_rpy_sp - orientation_rpy_current
        # velocity_rpy_error = velocity_rpy_sp - velocity_rpy_current

        tau_sp = orientation_rpy_controller.update(env.stepsize, orientation_rpy_error)

        # Find motor speeds needed to achieve desired linear and angular accelerations
        # quadcopter.des2speeds(thrust_needed, tau_needed)
        # ''' finds speeds of motors to achieve a desired thrust and torque '''
        # Needed torque on body
        e1 = tau_sp[0] * Quadcopter.Ixx
        e2 = tau_sp[1] * Quadcopter.Iyy
        e3 = tau_sp[2] * Quadcopter.Izz
        # less typing
        n = Quadcopter.num_motors
        # Thrust desired converted into motor speeds
        weight_speed = thrust_sp / (n * Quadcopter.kt)

        # Thrust differene in each motor to achieve needed torque on body
        motor_speeds_rpm_sp = []
        motor_speeds_rpm_sp.append(weight_speed - (e2 / ((n / 2) * kt * L)) - (e3 / (n * b_prop)))
        motor_speeds_rpm_sp.append(weight_speed - (e1 / ((n / 2) * kt * L)) + (e3 / (n * b_prop)))
        motor_speeds_rpm_sp.append(weight_speed + (e2 / ((n / 2) * kt * L)) - (e3 / (n * b_prop)))
        motor_speeds_rpm_sp.append(weight_speed + (e1 / ((n / 2) * kt * L)) + (e3 / (n * b_prop)))

        motor_speeds_rpm_sp = [_clamp(motor_speed_rpm, [0, 1000]) for motor_speed_rpm in motor_speeds_rpm_sp]
        motor_rpm_sp_list = motor_speeds_rpm_sp
        # Ensure that desired thrust is within overall min and max of all motors
        thrust_all = np.array(motor_speeds_rpm_sp) * kt
        total_thrust = np.sum(thrust_all)
        # over_max = np.argwhere(thrust_all > Quadcopter.maxT)
        # under_min = np.argwhere(thrust_all < Quadcopter.minT)

        # if over_max.size != 0:
        #     for i in range(over_max.size):
        #         motor_speeds[over_max[i][0]] = Quadcopter.maxT / kt
        # if under_min.size != 0:
        #     for i in range(under_min.size):
        #         motor_speeds[under_min[i][0]] = Quadcopter.minT /kt
        # self.speeds = motor_speeds

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
