//
// Created by arwillis on 2/20/24.
//
#include "gymfc_gazebo_barometer_plugin.h"

namespace gazebo {
    void GymFCBarometerPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
        //gzdbg << "Loading Barometer sensor\n";
        BarometerPlugin::Load(_model, _sdf);
        this->resetEvent_ = event::Events::ConnectTimeReset(boost::bind(&GymFCBarometerPlugin::OnTimeReset, this));
        gymfc_baro_pub_ = node_handle_->Advertise<sensor_msgs::msgs::Pressure>(gymfc_baro_pub_topic_);
        gzdbg << "GymFC barometer publishes to " << gymfc_baro_pub_topic_ << std::endl;
    }

    void GymFCBarometerPlugin::OnTimeReset() {
#if GAZEBO_MAJOR_VERSION >= 9
        last_time_ = world_->SimTime();
#else
        last_time_ = world_->GetSimTime();
#endif
    }

    void GymFCBarometerPlugin::OnUpdate(const common::UpdateInfo &info) {
        BarometerPlugin::OnUpdate(info);
        if (new_msg_published) {
            //gzdbg << "published barometer message." << std::endl;
            gymfc_baro_pub_->Publish(baro_msg_);
            new_msg_published = false;
        }
    }

    GZ_REGISTER_MODEL_PLUGIN(GymFCBarometerPlugin);
}