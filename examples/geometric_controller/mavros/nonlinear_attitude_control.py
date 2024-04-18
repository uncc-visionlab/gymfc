from common import quatMultiplication, quat2RotMatrix
from control import Control
import numpy as np


class NonlinearAttitudeControl(Control):
    # Geometric attitude controller
    # Attitude error is defined as in Brescianini, Dario, Markus Hehn, and Raffaello D'Andrea. Nonlinear quadrocopter
    # attitude control: Technical report. ETH Zurich, 2013
    def __init__(self, attctrl_tau):
        super(NonlinearAttitudeControl, self).__init__()
        self.attctrl_tau_ = attctrl_tau

    def Update(self, curr_att, ref_att, ref_acc, ref_jerk):
        inverse = np.array([1.0, -1.0, -1.0, -1.0])
        q_inv = np.diag(inverse) @ curr_att
        qe = quatMultiplication(q_inv, ref_att)
        self.desired_rate_[0] = (2.0 / self.attctrl_tau_) * np.copysign(1.0, qe[0]) * qe[1]
        self.desired_rate_[1] = (2.0 / self.attctrl_tau_) * np.copysign(1.0, qe[0]) * qe[2]
        self.desired_rate_[2] = (2.0 / self.attctrl_tau_) * np.copysign(1.0, qe[0]) * qe[3]
        rotmat = quat2RotMatrix(curr_att)
        zb = rotmat[:, 2]
        self.desired_thrust_[0] = 0.0
        self.desired_thrust_[1] = 0.0
        self.desired_thrust_[2] = np.dot(ref_acc, zb)
