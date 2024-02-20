//
// Created by arwillis on 2/20/24.
//
#include "gymfc_gazebo_groundtruth_plugin.h"

namespace gazebo {
    void GymFCGroundtruthPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
        gzdbg << "Loading ground truth plugin\n";
        GroundtruthPlugin::Load(_model, _sdf);

        this->resetEvent_ = event::Events::ConnectTimeReset(
                boost::bind(&GymFCGroundtruthPlugin::OnTimeReset, this));
    }

    void GymFCGroundtruthPlugin::OnTimeReset() {

    }

    GZ_REGISTER_MODEL_PLUGIN(GymFCGroundtruthPlugin);
}