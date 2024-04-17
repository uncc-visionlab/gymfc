from uav_simulator.matrix_utils import hat, vee, q_to_R
from uav_simulator.control import Control
from uav_simulator.estimator import Estimator
from uav_simulator.trajectory import Trajectory

import datetime
import numpy as np


class Rover:
    def __init__(self):
        self.on = True
        self.motor_on = False
        self.save_on = False
        # self.modes = ['Idle', 'Warm-up', 'Take-off', 'Land', 'Stay', 'Circle']
        self.mode = 0

        self.t0 = datetime.datetime.now()
        self.t = 0.0
        self.t_pre = 0.0
        self.freq_imu = 0.0
        self.freq_gps = 0.0
        self.freq_control = 0.0
        self.freq_log = 0.0

        self.x = np.zeros(3)
        self.v = np.zeros(3)
        self.a = np.zeros(3)
        self.R = np.identity(3)
        self.W = np.zeros(3)

        self.x_offset = np.zeros(3)
        self.yaw_offset = 0.0

        self.g = 9.81
        self.ge3 = np.array([0.0, 0.0, self.g])

        # Gazebo uses ENU frame, but NED frame is used in FDCL.
        # Note that ENU and the NED here refer to their direction order.
        # ENU: E - axis 1, N - axis 2, U - axis 3
        # NED: N - axis 1, E - axis 2, D - axis 3
        self.R_fg = np.array([
            [1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, -1.0]
        ])

        self.V_R_imu = np.diag([0.01, 0.01, 0.01])
        self.V_x_gps = np.diag([0.01, 0.01, 0.01])
        self.V_v_gps = np.diag([0.01, 0.01, 0.01])

        self.control = Control()
        self.control.use_integral = True  # Enable integral control

        self.estimator = Estimator()
        self.trajectory = Trajectory()

    def run_controller(self):
        states = self.estimator.get_states()
        desired = self.trajectory.get_desired(self.mode, states,
                                              self.x_offset, self.yaw_offset)
        fM = self.control.run(states, desired)

        self.x, self.v, self.a, self.R, self.W = states
        return fM

    def gymfc_imu_callback(self, imu_quat, imu_vel_rpy, imu_linear_acc_xyz, dt_imu):
        q = np.roll(np.array(imu_quat), -1)  # move to x,y,z,w order

        R_gi = q_to_R(q)  # IMU to Gazebo frame
        R_fi = self.R_fg.dot(R_gi)  # IMU to FDCL frame (NED freme)

        # FDCL-UAV expects IMU accelerations without gravity.
        a_i = np.array(imu_linear_acc_xyz)
        a_i = R_gi.T.dot(R_gi.dot(a_i) - self.ge3)

        W_i = np.array(imu_vel_rpy)

        self.estimator.prediction(a_i, W_i, dt_imu)
        self.estimator.imu_correction(R_fi, self.V_R_imu)

    def gymfc_gps_callback(self, gps_enu_pos, gps_enu_vel):
        # Gazebo uses ENU frame, but NED frame is used in FDCL.
        x_g = np.array([gps_enu_pos[0], -gps_enu_pos[1], -gps_enu_pos[2]])
        v_g = np.array([gps_enu_vel[0], -gps_enu_vel[1], -gps_enu_vel[2]])
        self.estimator.gps_correction(x_g, v_g, self.V_x_gps, self.V_v_gps)


# def reset_uav():
#     # rospy.wait_for_service('/gazebo/set_model_state')
#
#     init_position = Point(x=0.0, y=0.0, z=0.2)
#     init_attitude = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
#     init_pose = Pose(position=init_position, orientation=init_attitude)
#
#     zero_motion = Vector3(x=0.0, y=0.0, z=0.0)
#     init_velocity = Twist(linear=zero_motion, angular=zero_motion)
#
#     model_state = ModelState(model_name='uav', reference_frame='world', \
#                              pose=init_pose, twist=init_velocity)
#
#     # reset_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
#     # reset_state(model_state)
#
#     print('Resetting UAV successful ..')
