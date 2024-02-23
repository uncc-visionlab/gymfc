//
// Created by arwillis on 2/20/24.
//
#include "gymfc_gazebo_groundtruth_plugin.h"

namespace gazebo {
    void GymFCGroundtruthPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
        //gzdbg << "Loading ground truth plugin\n";
        GroundtruthPlugin::Load(_model, _sdf);
        this->resetEvent_ = event::Events::ConnectTimeReset(boost::bind(&GymFCGroundtruthPlugin::OnTimeReset, this));
        gymfc_gt_pub_ = node_handle_->Advertise<sensor_msgs::msgs::Groundtruth>(gymfc_gt_pub_topic_);
        gzdbg << "GymFC groundtruth publishes to " << gymfc_gt_pub_topic_ << std::endl;
    }

    void GymFCGroundtruthPlugin::OnTimeReset() {

    }

    void GymFCGroundtruthPlugin::OnUpdate(const common::UpdateInfo &info) {
        GroundtruthPlugin::OnUpdate(info);
        sensor_msgs::msgs::Groundtruth gymfc_gt_msg;
        gymfc_gt_msg = groundtruth_msg;
        gymfc_gt_pub_->Publish(gymfc_gt_msg);
    }

    GZ_REGISTER_MODEL_PLUGIN(GymFCGroundtruthPlugin);
}