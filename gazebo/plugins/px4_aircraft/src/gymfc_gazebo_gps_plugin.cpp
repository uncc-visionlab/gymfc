//
// Created by arwillis on 2/20/24.
//
#include "gymfc_gazebo_gps_plugin.h"

namespace gazebo {
    void GymFCGazeboGpsPlugin::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf) {
        gzdbg << "Loading GPS sensor\n";
        GpsPlugin::Load(_parent, _sdf);
        this->resetEvent_ = event::Events::ConnectTimeReset(
                boost::bind(&GymFCGazeboGpsPlugin::OnTimeReset, this));
        //imu_pub_->Fini();
        // Specify queue limit and rate, make this configurable
        //imu_pub_ = node_handle_->Advertise<sensor_msgs::msgs::Imu>("/aircraft/sensor/imu");
    }

    void GymFCGazeboGpsPlugin::OnTimeReset() {
#if GAZEBO_MAJOR_VERSION >= 9
        last_time_ = world_->SimTime();
#else
        last_time_ = world_->GetSimTime();
#endif
    }

    GZ_REGISTER_SENSOR_PLUGIN(GymFCGazeboGpsPlugin);
}