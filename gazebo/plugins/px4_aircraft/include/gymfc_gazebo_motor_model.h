//
// Created by arwillis on 2/20/24.
//

#ifndef GYMFC_PLUGINS_GYMFC_GAZEBO_MOTOR_MODEL_H
#define GYMFC_PLUGINS_GYMFC_GAZEBO_MOTOR_MODEL_H

#include "gazebo_motor_model.h"
#include "CommandMotorSpeed.pb.h"
#include "EscSensor.pb.h"
#include "gazebo/transport/transport.hh"
//#include "gazebo/msgs/msgs.hh"
//#include "MotorSpeed.pb.h"

namespace gazebo {
    static const std::string kGymFCDefaultCommandSubTopic = "/aircraft/command/motor";
    static const std::string kGymFCDefaultESCPubTopic = "/aircraft/sensor/esc";

    class GymFCGazeboMotorModel : public GazeboMotorModel {
    public:
        GymFCGazeboMotorModel() {

        }

        ~GymFCGazeboMotorModel() {

        }
        virtual void Publish();
    protected:
        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
        virtual void OnTimeReset();
        virtual void OnUpdate(const common::UpdateInfo &_info);

    private:
        event::ConnectionPtr resetEvent_;
        void VelocityCallback(CommandMotorSpeedPtr &rot_velocities);
        sensor_msgs::msgs::EscSensor gymfc_esc_sensor_msg_;
        std::string gymfc_command_sub_topic_{kGymFCDefaultCommandSubTopic};
        transport::SubscriberPtr gymfc_command_sub_;
        std::string gymfc_esc_pub_topic_{kGymFCDefaultESCPubTopic};
        transport::PublisherPtr gymfc_esc_pub_;
        float current_force_{0};
        float current_torque_{0};
    };
}
#endif //GYMFC_PLUGINS_GYMFC_GAZEBO_MOTOR_MODEL_H
