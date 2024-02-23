//
// Created by arwillis on 2/20/24.
//

#ifndef GYMFC_PLUGINS_GYMFC_GAZEBO_GPS_PLUGIN_H
#define GYMFC_PLUGINS_GYMFC_GAZEBO_GPS_PLUGIN_H

#include "gazebo_gps_plugin.h"

namespace gazebo {
    static const std::string kGymFCDefaultGPSPubTopic = "/aircraft/sensor/gps";

    class GymFCGazeboGpsPlugin : public GpsPlugin {
    public:
        GymFCGazeboGpsPlugin() {

        }

        ~GymFCGazeboGpsPlugin() {

        }
    protected:
        virtual void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);
        virtual void OnWorldUpdate(const common::UpdateInfo& /*_info*/);
        virtual void OnSensorUpdate();
        virtual void OnTimeReset();
    private:
        event::ConnectionPtr resetEvent_;
        std::string gymfc_gps_pub_topic_{kGymFCDefaultGPSPubTopic};
        transport::PublisherPtr gymfc_gps_pub_;
    };
}
#endif //GYMFC_PLUGINS_GYMFC_GAZEBO_GPS_PLUGIN_H
