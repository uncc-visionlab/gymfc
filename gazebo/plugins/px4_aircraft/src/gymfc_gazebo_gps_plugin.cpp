//
// Created by arwillis on 2/20/24.
//
#include "gymfc_gazebo_gps_plugin.h"

namespace gazebo {
    void GymFCGazeboGpsPlugin::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf) {
        //gzdbg << "Loading GPS sensor\n";
        GpsPlugin::Load(_parent, _sdf);
        resetEvent_ = event::Events::ConnectTimeReset(boost::bind(&GymFCGazeboGpsPlugin::OnTimeReset, this));
        gymfc_gps_pub_ = node_handle_->Advertise<sensor_msgs::msgs::SITLGps>(gymfc_gps_pub_topic_);
        gzdbg << "GymFC GPS publishes to " << gymfc_gps_pub_topic_ << std::endl;
        updateWorldConnection_ = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&GymFCGazeboGpsPlugin::OnWorldUpdate, this, _1));
    }

    void GymFCGazeboGpsPlugin::OnTimeReset() {
#if GAZEBO_MAJOR_VERSION >= 9
        last_time_ = world_->SimTime();
#else
        last_time_ = world_->GetSimTime();
#endif
    }

    void GymFCGazeboGpsPlugin::OnWorldUpdate(const common::UpdateInfo &_info) {
        GpsPlugin::OnWorldUpdate(_info);
    }

    void GymFCGazeboGpsPlugin::OnSensorUpdate() {
        GpsPlugin::OnSensorUpdate();
        if (new_msg_published) {
            //gzdbg << "published Gps message." << std::endl;
            gymfc_gps_pub_->Publish(gps_msg);
            new_msg_published = false;
        }
    }

    GZ_REGISTER_SENSOR_PLUGIN(GymFCGazeboGpsPlugin);
}