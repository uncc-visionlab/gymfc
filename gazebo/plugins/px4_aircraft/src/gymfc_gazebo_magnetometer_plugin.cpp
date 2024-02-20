//
// Created by arwillis on 2/20/24.
//
#include "gymfc_gazebo_magnetometer_plugin.h"

namespace gazebo {
    void GymFCMagnetometerPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
        gzdbg << "Loading Magnetometer sensor\n";
        MagnetometerPlugin::Load(_model, _sdf);
        this->resetEvent_ = event::Events::ConnectTimeReset(
                boost::bind(&GymFCMagnetometerPlugin::OnTimeReset, this));
    }

    void GymFCMagnetometerPlugin::OnTimeReset() {
#if GAZEBO_MAJOR_VERSION >= 9
        last_time_ = world_->SimTime();
#else
        last_time_ = world_->GetSimTime();
#endif
    }

    GZ_REGISTER_MODEL_PLUGIN(GymFCMagnetometerPlugin);
}