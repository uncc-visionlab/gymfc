//
// Created by arwillis on 2/20/24.
//
#include "gymfc_gazebo_groundtruth_plugin.h"

namespace gazebo {
    void GymFCGroundtruthPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
        //gzdbg << "Loading ground truth plugin\n";
        GroundtruthPlugin::Load(_model, _sdf);
        resetEvent_ = event::Events::ConnectTimeReset(boost::bind(&GymFCGroundtruthPlugin::OnTimeReset, this));
        gymfc_gt_pub_ = node_handle_->Advertise<sensor_msgs::msgs::Groundtruth>(gymfc_gt_pub_topic_);
        gzdbg << "GymFC groundtruth publishes to " << gymfc_gt_pub_topic_ << std::endl;
        updateConnection_ = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&GymFCGroundtruthPlugin::OnUpdate, this, _1));
    }

    void GymFCGroundtruthPlugin::OnTimeReset() {

    }

    void GymFCGroundtruthPlugin::OnUpdate(const common::UpdateInfo &info) {
        GroundtruthPlugin::OnUpdate(info);
//#if GAZEBO_MAJOR_VERSION >= 9
//        ignition::math::Vector3d vpos = this->model_->WorldLinearVel();
//        ignition::math::Vector3d veul = this->model_->WorldAngularVel();
//        ignition::math::Pose3d pose = model_->WorldPose();
//#else
//        ignition::math::Pose3d pose = ignitionFromGazeboMath(model_->GetWorldPose());
//#endif
//        // Use the models world position and attitude for groundtruth
//        ignition::math::Vector3d& pos = pose.Pos();
//        ignition::math::Quaterniond& vatt = pose.Rot();
//        sensor_msgs::msgs::Groundtruth gymfc_gt_msg;
//        gymfc_gt_msg.set_time_usec(groundtruth_msg.time_usec());
//        gymfc_gt_msg.set_attitude_q_w(vatt.W());
//        gymfc_gt_msg.set_attitude_q_x(vatt.X());
//        gymfc_gt_msg.set_attitude_q_y(vatt.Y());
//        gymfc_gt_msg.set_attitude_q_z(vatt.Z());
//        gymfc_gt_msg.set_velocity_east(vpos.X());
//        gymfc_gt_msg.set_velocity_north(vpos.Y());
//        gymfc_gt_msg.set_velocity_up(vpos.Z());
//        gymfc_gt_msg.set_longitude_rad(pos.X());
//        gymfc_gt_msg.set_latitude_rad(pos.Y());
//        gymfc_gt_msg.set_altitude(pos.Z());
        //gymfc_gt_msg = groundtruth_msg;
        //gymfc_gt_pub_->Publish(gymfc_gt_msg);
        gymfc_gt_pub_->Publish(groundtruth_msg);
    }

    GZ_REGISTER_MODEL_PLUGIN(GymFCGroundtruthPlugin);
}