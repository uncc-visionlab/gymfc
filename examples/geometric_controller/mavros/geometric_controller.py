from common import quat2RotMatrix, rot2Quaternion
from jerk_tracking_control import JerkTrackingControl
from nonlinear_geometric_control import NonlinearGeometricControl
from nonlinear_attitude_control import NonlinearAttitudeControl
from control import Control
import numpy as np


class MAV_STATE:
    MAV_STATE_UNINIT = 1
    MAV_STATE_BOOT = 2
    MAV_STATE_CALIBRATIN = 3
    MAV_STATE_STANDBY = 4
    MAV_STATE_ACTIVE = 5
    MAV_STATE_CRITICAL = 6
    MAV_STATE_EMERGENCY = 7
    MAV_STATE_POWEROFF = 8
    MAV_STATE_FLIGHT_TERMINATION = 8


class ControlMode:
    ERROR_QUATERNION = 1
    ERROR_GEOMETRIC = 2


class FlightState:
    WAITING_FOR_HOME_POSE = 1
    MISSION_EXECUTION = 2
    LANDING = 3
    LANDED = 4


class GeometricController:

    # bool fail_detec_{false};
    # bool feedthrough_enable_{false};
    # bool ctrl_enable_{true};
    # bool landing_commanded_{false};
    # double kp_rot_, kd_rot_;
    # double reference_request_dt_;
    # double norm_thrust_const_, norm_thrust_offset_;
    # double max_fb_acc_;
    #
    # mavros_msgs::State current_state_;
    # mavros_msgs::CommandBool arm_cmd_;
    # std::vector<geometry_msgs::PoseStamped> posehistory_vector_;
    # MAV_STATE companion_state_ = MAV_STATE::MAV_STATE_ACTIVE;
    #
    # Eigen::Vector3d targetPos_, targetVel_, targetAcc_, targetJerk_, targetSnap_, targetPos_prev_, targetVel_prev_;

    def __init__(self):
        self.node_state = FlightState.MISSION_EXECUTION
        self.fail_detec_ = False
        self.feedthrough_enable_ = False
        self.ctrl_enable_ = True
        self.landing_commanded_ = False
        self.gravity_ = np.array([0.0, 0.0, -9.8])
        self.mavAtt_ = np.zeros(4)
        self.q_des = np.zeros(4)

        self.mav_name_ = "iris"
        self.ctrl_mode_ = ControlMode.ERROR_QUATERNION
        self.sim_enable_ = True
        self.velocity_yaw_ = False  # velocity yaw param
        self.max_fb_acc_ = 9.0  # max_acc param
        self.mavYaw_ = 0  # yaw_heading param
        drag_dx = 0
        drag_dy = 0
        drag_dz = 0
        self.D_ = np.array([drag_dx, drag_dy, drag_dz])
        attctrl_tau = 0.1
        self.norm_thrust_const_ = 0.05
        self.norm_thrust_offset_ = 0.1
        self.Kpos_x_ = 8.0
        self.Kpos_y_ = 8.0
        self.Kpos_z_ = 10.0
        self.Kvel_x_ = 1.5
        self.Kvel_y_ = 1.5
        self.Kvel_z_ = 3.3
        self.posehistory_window_ = 200
        initTargetPos_x_ = 0
        initTargetPos_y_ = 0
        initTargetPos_z_ = 2.0
        self.targetPos_ = np.array([initTargetPos_x_, initTargetPos_y_, initTargetPos_z_])
        self.targetVel_ = np.array([0, 0, 0])
        self.mavPos_ = np.array([0, 0, 0])
        self.mavVel_ = np.array([0, 0, 0])
        self.Kpos_ = np.array([-self.Kpos_x_, -self.Kpos_y_, -self.Kpos_z_])
        self.Kvel_ = np.array([-self.Kvel_x_, -self.Kvel_y_, -self.Kvel_z_])
        jerk_enabled = False
        self.controller = Control()
        if not jerk_enabled:
            if self.ctrl_mode_ == ControlMode.ERROR_GEOMETRIC:
                self.controller_ = NonlinearGeometricControl(attctrl_tau)
            else:
                self.controller_ = NonlinearAttitudeControl(attctrl_tau)
        else:
            self.controller_ = JerkTrackingControl()

        self.cmdBodyRate_ = np.zeros(4)  # {wx, wy, wz, Thrust}
        self.mavRate_ = np.zeros(3)
        self.targetAcc_ = np.zeros(3)
        self.targetJerk_ = np.zeros(3)
        self.targetSnap_ = np.zeros(3)
        self.targetPos_prev_ = np.zeros(3)
        self.targetVel_prev_ = np.zeros(3)

    def getStates(self, pos, att, vel, angvel):
        pos = self.mavPos_
        att = self.mavAtt_
        vel = self.mavVel_
        angvel = self.mavRate_

    def getErrors(self, pos, vel):
        pos = self.mavPos_ - self.targetPos_
        vel = self.mavVel_ - self.targetVel_

# void geometricCtrl::pubReferencePose(const Eigen::Vector3d &target_position, const Eigen::Vector4d &target_attitude) {
#   geometry_msgs::PoseStamped msg;
#
#   msg.header.stamp = ros::Time::now();
#   msg.header.frame_id = "map";
#   msg.pose.position.x = target_position(0);
#   msg.pose.position.y = target_position(1);
#   msg.pose.position.z = target_position(2);
#   msg.pose.orientation.w = target_attitude(0);
#   msg.pose.orientation.x = target_attitude(1);
#   msg.pose.orientation.y = target_attitude(2);
#   msg.pose.orientation.z = target_attitude(3);
#   referencePosePub_.publish(msg);
# }
#
# void geometricCtrl::pubRateCommands(const Eigen::Vector4d &cmd, const Eigen::Vector4d &target_attitude) {
#   mavros_msgs::AttitudeTarget msg;
#
#   msg.header.stamp = ros::Time::now();
#   msg.header.frame_id = "map";
#   msg.body_rate.x = cmd(0);
#   msg.body_rate.y = cmd(1);
#   msg.body_rate.z = cmd(2);
#   msg.type_mask = 128;  // Ignore orientation messages
#   msg.orientation.w = target_attitude(0);
#   msg.orientation.x = target_attitude(1);
#   msg.orientation.y = target_attitude(2);
#   msg.orientation.z = target_attitude(3);
#   msg.thrust = cmd(3);
#
#   angularVelPub_.publish(msg);
# }
#
# void geometricCtrl::pubPoseHistory() {
#   nav_msgs::Path msg;
#
#   msg.header.stamp = ros::Time::now();
#   msg.header.frame_id = "map";
#   msg.poses = posehistory_vector_;
#
#   posehistoryPub_.publish(msg);
# }
#
# void geometricCtrl::pubSystemStatus() {
#   mavros_msgs::CompanionProcessStatus msg;
#
#   msg.header.stamp = ros::Time::now();
#   msg.component = 196;  // MAV_COMPONENT_ID_AVOIDANCE
#   msg.state = (int)companion_state_;
#
#   systemstatusPub_.publish(msg);
# }
#
# void geometricCtrl::appendPoseHistory() {
#   posehistory_vector_.insert(posehistory_vector_.begin(), vector3d2PoseStampedMsg(mavPos_, mavAtt_));
#   if (posehistory_vector_.size() > posehistory_window_) {
#     posehistory_vector_.pop_back();
#   }
# }
    def controlPosition(self, target_pos, target_vel, target_acc):
        # Compute BodyRate commands using differential flatness
        # Controller based on Faessler 2017
        a_ref = target_acc
        if self.velocity_yaw_ != 0:
            self.mavYaw_ = self.getVelocityYaw(self.mavVel_)

        q_ref = self.acc2quaternion(a_ref - self.gravity_, self.mavYaw_)
        R_ref = quat2RotMatrix(q_ref)

        pos_error = self.mavPos_ - target_pos
        vel_error = self.mavVel_ - target_vel

        # Position Controller
        a_fb = self.poscontroller(pos_error, vel_error)

        # Rotor Drag compensation
        a_rd = R_ref @ np.diag(self.D_) @ R_ref.T @ target_vel  # Rotor drag

        # Reference acceleration
        a_des = a_fb + a_ref - a_rd - self.gravity_
        return a_des

    def computeBodyRateCmd(self, bodyrate_cmd, a_des):
        # Reference attitude
        q_des = self.acc2quaternion(a_des, self.mavYaw_)

        self.controller_.Update(self.mavAtt_, q_des, a_des, self.targetJerk_)  # Calculate BodyRate
        bodyrate_cmd[:3] = self.controller_.getDesiredRate()
        thrust_command = self.controller_.getDesiredThrust()[2]
        # Calculate thrustcontroller_->getDesiredThrust()(3);
        bodyrate_cmd[3] = np.max(0.0, np.min(1.0, self.norm_thrust_const_ * thrust_command + self.norm_thrust_offset_))

    def poscontroller(self, pos_error, vel_error):
        # feedforward term for trajectory error
        a_fb = np.diag(self.Kpos_) @ pos_error + np.diag(self.Kvel_) @ vel_error

        if np.linalg.norm(a_fb) > self.max_fb_acc_:
            a_fb = (self.max_fb_acc_ / np.linalg.norm(a_fb)) * a_fb  # Clip acceleration if reference is too large
        return a_fb

    def acc2quaternion(self, vector_acc, yaw):
        proj_xb_des = np.array([np.cos(yaw), np.sin(yaw), 0.0])

        zb_des = vector_acc / np.linalg.norm(vector_acc)
        yb_des = np.cross(zb_des, proj_xb_des) / np.linalg.norm(np.cross(zb_des, proj_xb_des))
        xb_des = np.cross(yb_des, zb_des) / np.linalg.norm(np.cross(yb_des, zb_des))

        rotmat = np.array([[xb_des[0], yb_des[0], zb_des[0]],
                           [xb_des[1], yb_des[1], zb_des[1]],
                           [xb_des[2], yb_des[2], zb_des[2]]])
        quat = rot2Quaternion(rotmat)
        return quat

    def getVelocityYaw(self, velocity):
        return np.atan2(velocity[1], velocity[0])

    def cmdloopCallback(self, event):
        if self.node_state == FlightState.WAITING_FOR_HOME_POSE:
            # waitForPredicate(&received_home_pose, "Waiting for home pose...")
            # ROS_INFO("Got pose! Drone Ready to be armed.")
            self.node_state = FlightState.MISSION_EXECUTION
        elif self.node_state == FlightState.MISSION_EXECUTION:
            desired_acc = np.zeros(3)
            if self.feedthrough_enable_:
                desired_acc = self.targetAcc_
            else:
                desired_acc = self.controlPosition(self.targetPos_, self.targetVel_, self.targetAcc_)

            self.computeBodyRateCmd(self.cmdBodyRate_, desired_acc)
            # pubReferencePose(targetPos_, q_des)
            # pubRateCommands(cmdBodyRate_, q_des)
            # appendPoseHistory()
            # pubPoseHistory()
        elif self.node_state == FlightState.LANDING:
            # geometry_msgs::PoseStamped landingmsg;
            # landingmsg.header.stamp = ros::Time::now();
            # landingmsg.pose = home_pose_;
            # landingmsg.pose.position.z = landingmsg.pose.position.z + 1.0;
            # target_pose_pub_.publish(landingmsg);
            self.node_state = FlightState.LANDED;
            # ros::spinOnce();
        elif self.node_state == FlightState.LANDED:
            # ROS_INFO("Landed. Please set to position control and disarm.");
            # cmdloop_timer_.stop();
            xxx = 1
