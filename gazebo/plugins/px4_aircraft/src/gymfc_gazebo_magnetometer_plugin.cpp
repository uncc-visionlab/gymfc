//
// Created by arwillis on 2/20/24.
//
#include "gymfc_gazebo_magnetometer_plugin.h"

namespace gazebo {
    void GymFCMagnetometerPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
        //gzdbg << "Loading Magnetometer sensor\n";
        MagnetometerPlugin::Load(_model, _sdf);
        resetEvent_ = event::Events::ConnectTimeReset(boost::bind(&GymFCMagnetometerPlugin::OnTimeReset, this));
        gymfc_magneto_pub_ = node_handle_->Advertise<sensor_msgs::msgs::MagneticField>(gymfc_magneto_pub_topic_);
        gzdbg << "GymFC magnetometer publishes to " << gymfc_magneto_pub_topic_ << std::endl;
        update_connection_ = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&GymFCMagnetometerPlugin::OnUpdate, this, _1));
    }

    void GymFCMagnetometerPlugin::OnTimeReset() {
#if GAZEBO_MAJOR_VERSION >= 9
        last_time_ = world_->SimTime();
#else
        last_time_ = world_->GetSimTime();
#endif
    }

    void GymFCMagnetometerPlugin::OnUpdate(const common::UpdateInfo &info) {
        MagnetometerPlugin::OnUpdate(info);
        if (new_msg_published) {
            //gzdbg << "published magnetometer message." << std::endl;
            gymfc_magneto_pub_->Publish(mag_message_);
            new_msg_published = false;
        }
    }

    GZ_REGISTER_MODEL_PLUGIN(GymFCMagnetometerPlugin);
}