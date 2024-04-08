//
// Created by arwillis on 4/7/24.
//

#ifndef GYMFC_PLUGINS_GYMFC_GAZEBO_WIND3D_MOTOR_MODEL_PLUGIN_H
#define GYMFC_PLUGINS_GYMFC_GAZEBO_WIND3D_MOTOR_MODEL_PLUGIN_H
#include "gazebo_wind3d_motor_model_plugin.h"
#include "CommandMotorSpeed.pb.h"
#include "EscSensor.pb.h"
#include "gazebo/transport/transport.hh"
//#include "gazebo/msgs/msgs.hh"
//#include "MotorSpeed.pb.h"

namespace gazebo {
    static const std::string kGymFCDefaultCommandSubTopic = "/aircraft/command/motor";
    static const std::string kGymFCDefaultESCPubTopic = "/aircraft/sensor/esc";
    static const std::string kGymFCDefaultMotorWindPubTopic = "/aircraft/sensor/motor_wind";
    //std::string wind_sub_topic_ = "/world_wind";

    class GymFCWind3DGazeboMotorModel : public GazeboWind3DMotorModel {
    public:
        GymFCWind3DGazeboMotorModel() {

        }

        ~GymFCWind3DGazeboMotorModel() {

        }
        virtual void InitializeParams();
        virtual void Publish();
    protected:
        virtual void UpdateForcesAndMoments() {
            GazeboWind3DMotorModel::UpdateForcesAndMoments();
        }
        /// \brief A function to check the motor_Failure_Number_ and stimulate motor fail
        /// \details Doing joint_->SetVelocity(0,0) for the flagged motor to fail
        virtual void UpdateMotorFail() {
            GazeboWind3DMotorModel::UpdateMotorFail();
        }
        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
        virtual void OnUpdate(const common::UpdateInfo &_info) {
            GazeboWind3DMotorModel::OnUpdate(_info);
        }
        virtual void OnTimeReset();
    private:

        transport::SubscriberPtr gymfc_wind_sub_;
        event::ConnectionPtr resetEvent_;
        virtual void VelocityCallback(CommandMotorSpeedPtr &rot_velocities);
        virtual void WindVelocityCallback(WindPtr & msg);
        sensor_msgs::msgs::EscSensor gymfc_esc_sensor_msg_;
        std::string gymfc_command_sub_topic_{kGymFCDefaultCommandSubTopic};
        transport::SubscriberPtr gymfc_command_sub_;
        std::string gymfc_esc_pub_topic_{kGymFCDefaultESCPubTopic};
        std::string gymfc_wind_pub_topic_{kGymFCDefaultMotorWindPubTopic};
        transport::PublisherPtr gymfc_esc_pub_;
        transport::PublisherPtr gymfc_wind_pub_;
        float current_force_{0};
        float current_torque_{0};
    };
}
#endif //GYMFC_PLUGINS_GYMFC_GAZEBO_WIND3D_MOTOR_MODEL_PLUGIN_H
