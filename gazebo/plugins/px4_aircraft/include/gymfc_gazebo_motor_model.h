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
    //static const std::string kGymFCDefaultMotorFailureNumSubTopic = "/gazebo/motor_failure_num";
    static const std::string kGymFCDefaultMotorVelocityPubTopic = "/aircraft/sensor/esc";

    class GymFCGazeboMotorModel : public GazeboMotorModel {
    public:
        GymFCGazeboMotorModel() {

        }

        ~GymFCGazeboMotorModel() {

        }
        virtual void Publish();
    protected:
        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
        void OnTimeReset();
    private:
        void VelocityCallback(CommandMotorSpeedPtr &rot_velocities);
        sensor_msgs::msgs::EscSensor sensor;
        transport::PublisherPtr esc_sensor_pub_;
        std::string command_sub_topic_2_{kGymFCDefaultCommandSubTopic};
        std::string motor_speed_pub_topic_2_{kGymFCDefaultMotorVelocityPubTopic};
        transport::SubscriberPtr command_sub_2_;
        event::ConnectionPtr resetEvent_;
        float current_force_{0};
        float current_torque_{0};
        //common::Time last_time_;
    };
}
#endif //GYMFC_PLUGINS_GYMFC_GAZEBO_MOTOR_MODEL_H
