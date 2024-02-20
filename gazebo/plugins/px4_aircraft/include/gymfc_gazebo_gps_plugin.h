//
// Created by arwillis on 2/20/24.
//

#ifndef GYMFC_PLUGINS_GYMFC_GAZEBO_GPS_PLUGIN_H
#define GYMFC_PLUGINS_GYMFC_GAZEBO_GPS_PLUGIN_H

#include "gazebo_gps_plugin.h"

namespace gazebo {
    class GymFCGazeboGpsPlugin : public GpsPlugin {
    public:
        GymFCGazeboGpsPlugin() {

        }

        ~GymFCGazeboGpsPlugin() {

        }
    protected:
        void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);
        void OnTimeReset();
    private:
        //physics::WorldPtr world_;
        //transport::PublisherPtr imu_pub_;
        event::ConnectionPtr resetEvent_;

        //common::Time last_time_;
    };
}
#endif //GYMFC_PLUGINS_GYMFC_GAZEBO_GPS_PLUGIN_H
