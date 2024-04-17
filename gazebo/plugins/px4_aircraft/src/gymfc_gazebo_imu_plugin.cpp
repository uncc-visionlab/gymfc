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
        //sensor_msgs::msgs::Imu gymfc_imu_msg;
        //gymfc_imu_msg = imu_message_;
        gymfc_imu_pub_->Publish(imu_message_);
//        sensor_msgs::msgs::Imu gymfc_imu_message_;
//        gymfc_imu_message_.CopyFrom(imu_message_);
//        gymfc_imu_pub_->Publish(gymfc_imu_message_);
    }

    GZ_REGISTER_MODEL_PLUGIN(GymFCGazeboImuPlugin);
}