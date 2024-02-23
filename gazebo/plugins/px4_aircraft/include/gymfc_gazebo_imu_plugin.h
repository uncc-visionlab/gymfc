//
// Created by arwillis on 2/20/24.
//

#ifndef GYMFC_PLUGINS_GYMFC_GAZEBO_IMU_PLUGIN_H
#define GYMFC_PLUGINS_GYMFC_GAZEBO_IMU_PLUGIN_H

#include "gazebo_imu_plugin.h"

namespace gazebo {
    static const std::string kGymFCDefaultImuPubTopic = "/aircraft/sensor/imu";

    class GymFCGazeboImuPlugin : public GazeboImuPlugin {
    public:
        GymFCGazeboImuPlugin() {

        }

        ~GymFCGazeboImuPlugin() {

        }
        void Publish();
    protected:
        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
        virtual void OnUpdate(const common::UpdateInfo& /*_info*/);
        virtual void OnTimeReset();
    private:
        event::ConnectionPtr resetEvent_;
        std::string gymfc_imu_pub_topic_{kGymFCDefaultImuPubTopic};
        transport::PublisherPtr gymfc_imu_pub_;
    };
}

#endif //GYMFC_PLUGINS_GYMFC_GAZEBO_IMU_PLUGIN_H
