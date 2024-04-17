//
// Created by arwillis on 2/20/24.
//
#include "gymfc_gazebo_imu_plugin.h"

namespace gazebo {
    void GymFCGazeboImuPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
        //gzdbg << "Loading IMU sensor\n";
        GazeboImuPlugin::Load(_model, _sdf);
        resetEvent_ = event::Events::ConnectTimeReset(boost::bind(&GymFCGazeboImuPlugin::OnTimeReset, this));
        gzdbg << "GymFC imu publishes to " << gymfc_imu_pub_topic_ << std::endl;
        gymfc_imu_pub_ = node_handle_->Advertise<sensor_msgs::msgs::Imu>(gymfc_imu_pub_topic_);
        updateConnection_ = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&GymFCGazeboImuPlugin::OnUpdate, this, _1));
    }

    void GymFCGazeboImuPlugin::OnTimeReset() {
#if GAZEBO_MAJOR_VERSION >= 9
        last_time_ = world_->SimTime();
#else
        last_time_ = world_->GetSimTime();
#endif
    }

    void GymFCGazeboImuPlugin::OnUpdate(const common::UpdateInfo &info) {
        GazeboImuPlugin::OnUpdate(info);
        sensor_msgs::msgs::Imu gymfc_imu_msg;
        ignition::math::Pose3d pose = link_->WorldPose();
        ignition::math::Quaterniond orientation = pose.Rot();
        ignition::math::Vector3d linear_acc = link_->WorldLinearAccel();
        ignition::math::Vector3d angular_vel = link_->WorldAngularVel();
        gazebo::msgs::Quaternion* orientation_msg = new gazebo::msgs::Quaternion;
        gazebo::msgs::Vector3d* angular_vel_msg = new gazebo::msgs::Vector3d;
        gazebo::msgs::Vector3d* linear_acc_msg = new gazebo::msgs::Vector3d;
        orientation_msg->set_w(orientation.W());
        orientation_msg->set_x(orientation.X());
        orientation_msg->set_y(orientation.Y());
        orientation_msg->set_z(orientation.Z());
        angular_vel_msg->set_x(angular_vel.X());
        angular_vel_msg->set_y(angular_vel.Y());
        angular_vel_msg->set_z(angular_vel.Z());
        linear_acc_msg->set_x(linear_acc.X());
        linear_acc_msg->set_y(linear_acc.Y());
        linear_acc_msg->set_z(linear_acc.Z());
        gymfc_imu_msg.set_time_usec(imu_message_.time_usec());
        gymfc_imu_msg.set_seq(imu_message_.seq());
        gymfc_imu_msg.set_allocated_orientation(orientation_msg);
        gymfc_imu_msg.set_allocated_angular_velocity(angular_vel_msg);
        gymfc_imu_msg.set_allocated_linear_acceleration(linear_acc_msg);
        //gymfc_imu_msg = imu_message_;
//        gymfc_imu_pub_->Publish(imu_message_);
//        sensor_msgs::msgs::Imu gymfc_imu_message_;
//        gymfc_imu_message_.CopyFrom(imu_message_);
        gymfc_imu_pub_->Publish(gymfc_imu_msg);
    }

    GZ_REGISTER_MODEL_PLUGIN(GymFCGazeboImuPlugin);
}