//
// Created by arwillis on 2/20/24.
//
#include "gymfc_gazebo_imu_plugin.h"

namespace gazebo {
    void GymFCGazeboImuPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
        gzdbg << "Loading IMU sensor\n";
        GazeboImuPlugin::Load(_model, _sdf);

        this->resetEvent_ = event::Events::ConnectTimeReset(
                boost::bind(&GymFCGazeboImuPlugin::OnTimeReset, this));

        imu_pub_->Fini();
        // Specify queue limit and rate, make this configurable
        imu_pub_ = node_handle_->Advertise<sensor_msgs::msgs::Imu>("/aircraft/sensor/imu");
        //gzdbg << "IMU sensor loaded.\n";
    }

    void GymFCGazeboImuPlugin::OnTimeReset() {
#if GAZEBO_MAJOR_VERSION >= 9
        last_time_ = world_->SimTime();
#else
        last_time_ = world_->GetSimTime();
#endif
    }

    GZ_REGISTER_MODEL_PLUGIN(GymFCGazeboImuPlugin);
}