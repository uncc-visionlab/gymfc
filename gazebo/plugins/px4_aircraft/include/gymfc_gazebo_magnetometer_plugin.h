//
// Created by arwillis on 2/20/24.
//

#ifndef GYMFC_PLUGINS_GYMFC_GAZEBO_MAGNETOMETER_PLUGIN_H
#define GYMFC_PLUGINS_GYMFC_GAZEBO_MAGNETOMETER_PLUGIN_H

#include "gazebo_magnetometer_plugin.h"

namespace gazebo {
    static const std::string kGymFCDefaultMagnetometerPubTopic = "/aircraft/sensor/magneto";

    class GymFCMagnetometerPlugin : public MagnetometerPlugin {
    public:
        GymFCMagnetometerPlugin() {

        }

        ~GymFCMagnetometerPlugin() {

        }
    protected:
        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
        virtual void OnUpdate(const common::UpdateInfo& /*_info*/);
        virtual void OnTimeReset();
    private:
        event::ConnectionPtr resetEvent_;
        std::string gymfc_magneto_pub_topic_{kGymFCDefaultMagnetometerPubTopic};
        transport::PublisherPtr gymfc_magneto_pub_;
    };
}

#endif //GYMFC_PLUGINS_GYMFC_GAZEBO_MAGNETOMETER_PLUGIN_H
