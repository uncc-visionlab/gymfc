import numpy as np


class Control(object):
    def __init__(self):
        self.desired_rate_ = np.zeros(3)
        self.desired_thrust_ = np.zeros(3)

    def Update(self, curr_att, ref_att, ref_acc, ref_jerk):
        print("Control::Update() NOT IMPLEMENTED\n")

    def getDesiredThrust(self):
        return self.desired_thrust_

    def getDesiredRate(self):
        return self.desired_rate_
