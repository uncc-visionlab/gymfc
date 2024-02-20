//
// Created by arwillis on 2/20/24.
//

#ifndef GYMFC_PLUGINS_GYMFC_GAZEBO_MAGNETOMETER_PLUGIN_H
#define GYMFC_PLUGINS_GYMFC_GAZEBO_MAGNETOMETER_PLUGIN_H

#include "gazebo_magnetometer_plugin.h"

namespace gazebo {
    class GymFCMagnetometerPlugin : public MagnetometerPlugin {
    public:
        GymFCMagnetometerPlugin() {

        }

        ~GymFCMagnetometerPlugin() {

        }
    protected:
        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
        void OnTimeReset();
    private:
        //physics::WorldPtr world_;
        //transport::PublisherPtr imu_pub_;
        event::ConnectionPtr resetEvent_;

        //common::Time last_time_;
    };
}

#endif //GYMFC_PLUGINS_GYMFC_GAZEBO_MAGNETOMETER_PLUGIN_H
