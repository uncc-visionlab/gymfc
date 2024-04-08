//
// Created by arwillis on 4/7/24.
//
#include "gymfc_gazebo_blast3d_model_plugin.h"

namespace gazebo {
    void GymFCGazeboBlast3DModelPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
        //gzdbg << "Loading gymFC Blast3d model plugin.\n";
        GazeboBlast3DModelPlugin::Load(_model, _sdf);
        resetEvent_ = event::Events::ConnectTimeReset(boost::bind(&GymFCGazeboBlast3DModelPlugin::OnTimeReset, this));
        gymfc_blast3d_pub_ = node_handle_->Advertise<blast3d_msgs::msgs::Blast3d>(gymfc_blast3d_pub_topic_);
        gzdbg << "GymFC blast3d events publish to " << gymfc_blast3d_pub_topic_ << std::endl;
    }

    void GymFCGazeboBlast3DModelPlugin::OnTimeReset() {

    }

    void GymFCGazeboBlast3DModelPlugin::OnUpdate(const common::UpdateInfo &info) {
        GazeboBlast3DModelPlugin::OnUpdate(info);
    }

    void GymFCGazeboBlast3DModelPlugin::Blast3DCallback(Blast3dMsgPtr &blast3d_msg) {
        GazeboBlast3DModelPlugin::Blast3DCallback(blast3d_msg);
        gymfc_blast3d_pub_->Publish(*blast3d_msg);
    }

    GZ_REGISTER_MODEL_PLUGIN(GymFCGazeboBlast3DModelPlugin);
}