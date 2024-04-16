//
// Created by arwillis on 2/20/24.
//
#include "gymfc_gazebo_motor_model.h"

namespace gazebo {
    void GymFCGazeboMotorModel::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
//        gzdbg << "Loading motor model\n";
        GazeboMotorModel::Load(_model, _sdf);
        resetEvent_ = event::Events::ConnectTimeReset(boost::bind(&GymFCGazeboMotorModel::OnTimeReset, this));
        gymfc_command_sub_ = node_handle_->Subscribe<mav_msgs::msgs::CommandMotorSpeed>(
                gymfc_command_sub_topic_, &GymFCGazeboMotorModel::VelocityCallback, this);
        std::string esc_pub_topic_ = gymfc_esc_pub_topic_ + "/" + std::to_string(motor_number_);
        gymfc_esc_pub_ = node_handle_->Advertise<sensor_msgs::msgs::EscSensor>(esc_pub_topic_);
        gzdbg << "GymFC motor number=" << motor_number_ << " subscribed to " << gymfc_command_sub_topic_
              << " publishes esc to " << esc_pub_topic_ << std::endl;
        updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GymFCGazeboMotorModel::OnUpdate, this, _1));
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

    void GymFCGazeboMotorModel::VelocityCallback(CommandMotorSpeedPtr &rot_velocities) {
        GazeboMotorModel::VelocityCallback(rot_velocities);
        //gzdbg << "Velocity motor " << motor_number_ << " = " << rot_velocities->motor_speed(motor_number_) << std::endl;
    }

    void GymFCGazeboMotorModel::OnUpdate(const common::UpdateInfo &_info) {
        GazeboMotorModel::OnUpdate(_info);
        //Publish();
    }

    void GymFCGazeboMotorModel::Publish() {
        GazeboMotorModel::Publish();
//        if (rotor_velocity_units_.compare(rotor_velocity_units::RPM) == 0) {
//            double rpms = std::abs(joint_->GetVelocity(0) * 9.5493);
//            //gzdbg << "Publishing ESC sensors " << motor_number_ << " Velocity " << rpms << std::endl;
//            sensor.set_motor_speed(rpms);
//        } else {
        gymfc_esc_sensor_msg_.set_motor_speed(std::abs(joint_->GetVelocity(0)));
//        }


        // XXX When id was of type int64 motor speed will be reset
        // after id is set.
        gymfc_esc_sensor_msg_.set_id(motor_number_);

        // TODO develop model for these
        gymfc_esc_sensor_msg_.set_current(0);
        gymfc_esc_sensor_msg_.set_temperature(0);
        gymfc_esc_sensor_msg_.set_voltage(0);

        gymfc_esc_sensor_msg_.set_force(current_force_);
        gymfc_esc_sensor_msg_.set_torque(current_torque_);

        gymfc_esc_pub_->Publish(gymfc_esc_sensor_msg_);
        //gzdbg << "Sending esc sensor for motor " << motor_number_ << std::endl;
    }

    GZ_REGISTER_MODEL_PLUGIN(GymFCGazeboMotorModel);
}