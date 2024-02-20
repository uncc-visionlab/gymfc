//
// Created by arwillis on 2/20/24.
//
#include "gymfc_gazebo_barometer_plugin.h"

namespace gazebo {
    void GymFCBarometerPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
        gzdbg << "Loading Barometer sensor\n";
        BarometerPlugin::Load(_model, _sdf);
        this->resetEvent_ = event::Events::ConnectTimeReset(
                boost::bind(&GymFCBarometerPlugin::OnTimeReset, this));
    }

    void GymFCBarometerPlugin::OnTimeReset() {
#if GAZEBO_MAJOR_VERSION >= 9
        last_time_ = world_->SimTime();
#else
        last_time_ = world_->GetSimTime();
#endif
    }

    GZ_REGISTER_MODEL_PLUGIN(GymFCBarometerPlugin);
}