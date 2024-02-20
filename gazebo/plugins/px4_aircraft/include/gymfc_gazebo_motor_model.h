//
// Created by arwillis on 2/20/24.
//

#ifndef GYMFC_PLUGINS_GYMFC_GAZEBO_MOTOR_MODEL_H
#define GYMFC_PLUGINS_GYMFC_GAZEBO_MOTOR_MODEL_H

#include "gazebo_motor_model.h"

namespace gazebo {
    class GymFCGazeboMotorModel : public GazeboMotorModel {
    public:
        GymFCGazeboMotorModel() {

        }

        ~GymFCGazeboMotorModel() {

        }
    protected:
        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
        void OnTimeReset();
    private:
        transport::PublisherPtr esc_sensor_pub_;

        //physics::WorldPtr world_;
        //transport::PublisherPtr imu_pub_;
        event::ConnectionPtr resetEvent_;

        //common::Time last_time_;
    };
}
#endif //GYMFC_PLUGINS_GYMFC_GAZEBO_MOTOR_MODEL_H
