//
// Created by arwillis on 4/7/24.
//

#ifndef GYMFC_PLUGINS_GYMFC_GAZEBO_BLAST3D_MODEL_PLUGIN_H
#define GYMFC_PLUGINS_GYMFC_GAZEBO_BLAST3D_MODEL_PLUGIN_H

#include "gazebo_blast3d_model_plugin.h"

namespace gazebo {
    static const std::string kGymFCDefaultBlast3dPubTopic = "/aircraft/sensor/blast3d";

    class GymFCGazeboBlast3DModelPlugin : public GazeboBlast3DModelPlugin {
    public:
        GymFCGazeboBlast3DModelPlugin() {

        }

        ~GymFCGazeboBlast3DModelPlugin() {

        }
    protected:
        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
        virtual void OnUpdate(const common::UpdateInfo& /*_info*/);
        virtual void OnTimeReset();
        virtual void Blast3DCallback(Blast3dMsgPtr &blast3d_msg);
    private:
        event::ConnectionPtr resetEvent_;
        std::string gymfc_blast3d_pub_topic_{kGymFCDefaultBlast3dPubTopic};
        transport::PublisherPtr gymfc_blast3d_pub_;
    };
}

#endif //GYMFC_PLUGINS_GYMFC_GAZEBO_BLAST3D_MODEL_PLUGIN_H
