//
// Created by arwillis on 4/7/24.
//
#include "gymfc_gazebo_wind3d_motor_model_plugin.h"

namespace gazebo {
    void GymFCWind3DGazeboMotorModel::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
        gzdbg << "Loading gymFC wind3d motor model plugin\n";
        GazeboWind3DMotorModel::Load(_model, _sdf);
        this->resetEvent_ = event::Events::ConnectTimeReset(boost::bind(&GymFCWind3DGazeboMotorModel::OnTimeReset, this));
        gymfc_command_sub_ = node_handle_->Subscribe<mav_msgs::msgs::CommandMotorSpeed>(
                gymfc_command_sub_topic_ , &GymFCWind3DGazeboMotorModel::VelocityCallback, this);
        std::string esc_pub_topic_ = gymfc_esc_pub_topic_ + "/" + std::to_string(motor_number_);
        std::string wind_pub_topic_ = gymfc_wind_pub_topic_ + "/" + std::to_string(motor_number_);
        gymfc_esc_pub_ = node_handle_->Advertise<sensor_msgs::msgs::EscSensor>(esc_pub_topic_);
        gymfc_wind_sub_ = node_handle_->Subscribe(wind_server_link_wind_topic_, &GymFCWind3DGazeboMotorModel::WindVelocityCallback, this);
        gymfc_wind_pub_ = node_handle_->Advertise<physics_msgs::msgs::Wind>(wind_pub_topic_);
        gzdbg << "GymFC motor number=" << motor_number_ << " subscribed to " << gymfc_command_sub_topic_
              << " publishes esc to " << esc_pub_topic_ << " published wind to " << wind_pub_topic_ << std::endl;
    }

    void GymFCWind3DGazeboMotorModel::OnTimeReset() {
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

    void GymFCWind3DGazeboMotorModel::VelocityCallback(CommandMotorSpeedPtr &rot_velocities) {
        GazeboWind3DMotorModel::VelocityCallback(rot_velocities);
        //gzdbg << "Velocity motor " << motor_number_ << " = " << rot_velocities->motor_speed(motor_number_) << std::endl;
    }

    void GymFCWind3DGazeboMotorModel::WindVelocityCallback(WindPtr & msg) {
        if (kPrintOnMsgCallback) {
            ignition::math::Vector3d position = link_->WorldPose().Pos();
            gzdbg << __FUNCTION__ << "() motor " << motor_number_ << " pos (x,y,z)= (" <<
                  position.X() << ", " << position.Y() << ", " << position.Z() << ") " <<
                  "wind speed (x,y,z)=(" <<
                  msg->velocity().x() << ", " <<
                  msg->velocity().y() << ", " <<
                  msg->velocity().z() << ")" << std::endl;
        }
        physics_msgs::msgs::Wind gymfc_msg;
        gymfc_msg.set_time_usec(msg->time_usec());
        gymfc_msg.set_frame_id(std::to_string(motor_number_));
        gazebo::msgs::Vector3d* wind_v_ptr = new gazebo::msgs::Vector3d();
        *wind_v_ptr = msg->velocity();
        gymfc_msg.set_allocated_velocity(wind_v_ptr);
        gymfc_wind_pub_->Publish(gymfc_msg);
        // TODO: Publish to FlightControllerPlugin for inclusion in observation vector
//        wind_vel_ = ignition::math::Vector3d(msg->velocity().x(),
//                                             msg->velocity().y(),
//                                             msg->velocity().z());
    }

    void GymFCWind3DGazeboMotorModel::InitializeParams() {
    }

    void GymFCWind3DGazeboMotorModel::Publish() {
        GazeboWind3DMotorModel::Publish();
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

        // TODO: develop model for these
        gymfc_esc_sensor_msg_.set_current(0);
        gymfc_esc_sensor_msg_.set_temperature(0);
        gymfc_esc_sensor_msg_.set_voltage(0);

        gymfc_esc_sensor_msg_.set_force(current_force_);
        gymfc_esc_sensor_msg_.set_torque(current_torque_);

        gymfc_esc_pub_->Publish(gymfc_esc_sensor_msg_);
        //gzdbg << "Sending esc sensor for motor " << motor_number_ << std::endl;
    }

    GZ_REGISTER_MODEL_PLUGIN(GymFCWind3DGazeboMotorModel);
}