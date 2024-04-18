from common import quat2RotMatrix, matrix_hat_inv
from control import Control
import numpy as np


class NonlinearGeometricControl(Control):
    # Geometric attitude controller
    # Attitude error is defined as in Lee, Taeyoung, Melvin Leok, and N. Harris McClamroch. "Geometric tracking control
    # of a quadrotor UAV on SE (3)." 49th IEEE conference on decision and control (CDC). IEEE, 2010.
    # The original paper inputs moment commands, but for offboard control, angular rate commands are sent
    def __init__(self, attctrl_tau=1.0):
        super(NonlinearGeometricControl, self).__init__()
        self.attctrl_tau_ = attctrl_tau

    def Update(self, curr_att, ref_att, ref_acc, ref_jerk):
        rotmat = quat2RotMatrix(curr_att)
        rotmat_d = quat2RotMatrix(ref_att)

        error_att = 0.5 * matrix_hat_inv(rotmat_d.T @ rotmat - rotmat.T @ rotmat_d)
        self.desired_rate_ = (2.0 / self.attctrl_tau_) * error_att
        zb = rotmat[:, 2]
        self.desired_thrust_[0] = 0.0
        self.desired_thrust_[1] = 0.0
        self.desired_thrust_[2] = np.dot(ref_acc, zb)
