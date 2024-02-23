//
// Created by arwillis on 2/20/24.
//

#ifndef GYMFC_PLUGINS_GYMFC_GAZEBO_GROUNDTRUTH_PLUGIN_H
#define GYMFC_PLUGINS_GYMFC_GAZEBO_GROUNDTRUTH_PLUGIN_H

#include "gazebo_groundtruth_plugin.h"

namespace gazebo {
    static const std::string kGymFCDefaultGroundtruthPubTopic = "/aircraft/sensor/groundtruth";

    class GymFCGroundtruthPlugin : public GroundtruthPlugin {
    public:
        GymFCGroundtruthPlugin() {

        }

        ~GymFCGroundtruthPlugin() {

        }
    protected:
        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
        virtual void OnUpdate(const common::UpdateInfo& /*_info*/);
        virtual void OnTimeReset();
    private:
        event::ConnectionPtr resetEvent_;
        std::string gymfc_gt_pub_topic_{kGymFCDefaultGroundtruthPubTopic};
        transport::PublisherPtr gymfc_gt_pub_;
    };
}
#endif //GYMFC_PLUGINS_GYMFC_GAZEBO_GROUNDTRUTH_PLUGIN_H
