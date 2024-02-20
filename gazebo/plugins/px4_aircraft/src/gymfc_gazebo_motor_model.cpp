//
// Created by arwillis on 2/20/24.
//
#include "gymfc_gazebo_motor_model.h"

namespace gazebo {
    void GymFCGazeboMotorModel::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
        gzdbg << "Loading motor model\n";
        GazeboMotorModel::Load(_model, _sdf);

        this->resetEvent_ = event::Events::ConnectTimeReset(
                boost::bind(&GymFCGazeboMotorModel::OnTimeReset, this));

//        esc_sensor_pub_ = node_handle_->Advertise<sensor_msgs::msgs::EscSensor>(
//                motor_speed_pub_topic_ + "/" + std::to_string(motor_number_));

        gzdbg << "Loading Motor number=" << motor_number_ << " Subscribed to " << command_sub_topic_ << std::endl;
    }

    void GymFCGazeboMotorModel::OnTimeReset() {
        gzdbg << "Resetting motor " << motor_number_ << std::endl;
        joint_->Reset();
        joint_->SetVelocity(0, 0);
        motor_rot_vel_ = 0;
        ref_motor_rot_vel_ = 0;
        prev_sim_time_ = 0;
        sampling_time_ = 0.01;
        rotor_velocity_filter_.reset(
                new FirstOrderFilter<double>(time_constant_up_, time_constant_down_, ref_motor_rot_vel_));
        pid_.SetCmd(0.0);
        pid_.Reset();
    }

    GZ_REGISTER_MODEL_PLUGIN(GymFCGazeboMotorModel);
}