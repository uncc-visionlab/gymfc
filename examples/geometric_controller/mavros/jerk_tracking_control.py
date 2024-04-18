from common import quatMultiplication, quat2RotMatrix
from control import Control
import numpy as np


class JerkTrackingControl(Control):
    # Jerk feedforward control
    # Based on: Lopez, Brett Thomas. Low-latency trajectory planning for high-speed navigation in unknown environments.
    # Diss. Massachusetts Institute of Technology, 2016.
    # Feedforward control from Lopez(2016)
    def __init__(self):
        super(JerkTrackingControl, self).__init__()
        self.last_ref_acc_ = np.zeros(3)

    def Update(self, curr_att, ref_att, ref_acc, ref_jerk):
        dt_ = 0.01
        # Numerical differentiation to calculate jerk_fb
        jerk_fb = (ref_acc - self.last_ref_acc_) / dt_
        jerk_des = ref_jerk + jerk_fb
        R = quat2RotMatrix(curr_att)
        zb = R[:, 2]
        jerk_vector = jerk_des / np.linalg.norm(ref_acc) \
                      - ref_acc * np.dot(ref_acc, jerk_des) / np.linalg.norm(ref_acc) ** 3
        jerk_vector4d = np.array([0.0, jerk_vector[0], jerk_vector[1], jerk_vector[2]])
        inverse = np.array([1.0, -1.0, -1.0, -1.0])
        q_inv = np.diag(inverse) @ curr_att
        qd = quatMultiplication(q_inv, ref_att)
        qd_star = np.array([qd[0], -qd[1], -qd[2], -qd[3]])
        ratecmd_pre = quatMultiplication(quatMultiplication(qd_star, jerk_vector4d), qd)
        self.desired_rate_[0] = ratecmd_pre[2]  # TODO: Are the coordinate systems consistent?
        self.desired_rate_[1] = (-1.0) * ratecmd_pre[1]
        self.desired_rate_[2] = 0.0
        self.desired_thrust_[0] = 0.0
        self.desired_thrust_[1] = 0.0
        self.desired_thrust_[2] = np.dot(ref_acc, zb)  # Calculate thrust
        self.last_ref_acc_ = ref_acc
