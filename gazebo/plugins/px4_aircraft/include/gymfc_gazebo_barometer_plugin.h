//
// Created by arwillis on 2/20/24.
//

#ifndef GYMFC_PLUGINS_GYMFC_GAZEBO_BAROMETER_PLUGIN_H
#define GYMFC_PLUGINS_GYMFC_GAZEBO_BAROMETER_PLUGIN_H

#include "gazebo_barometer_plugin.h"

namespace gazebo {
    static const std::string kGymFCDefaultBarometerPubTopic = "/aircraft/sensor/baro";

    class GymFCBarometerPlugin : public BarometerPlugin {
    public:
        GymFCBarometerPlugin() {

        }

        ~GymFCBarometerPlugin() {

        }
    protected:
        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
        virtual void OnUpdate(const common::UpdateInfo& /*_info*/);
        virtual void OnTimeReset();
    private:
        event::ConnectionPtr resetEvent_;
        std::string gymfc_baro_pub_topic_{kGymFCDefaultBarometerPubTopic};
        transport::PublisherPtr gymfc_baro_pub_;
    };
}
#endif //GYMFC_PLUGINS_GYMFC_GAZEBO_BAROMETER_PLUGIN_H
