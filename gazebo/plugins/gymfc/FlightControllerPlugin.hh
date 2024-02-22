/*
 * Copyright (C) 2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#ifndef GAZEBO_PLUGINS_QUADCOPTERWORLDPLUGIN_HH_
#define GAZEBO_PLUGINS_QUADCOPTERWORLDPLUGIN_HH_

#include <sdf/sdf.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
//#include <boost/thread.hpp>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/sensors.hh>
#include "gazebo/common/Events.hh"

#include <gazebo/physics/Base.hh>
#include "gazebo/transport/transport.hh"

//#include "MotorCommand.pb.h"
#include "CommandMotorSpeed.pb.h"
#include "EscSensor.pb.h"
#include "Imu.pb.h"
#include "State.pb.h"
#include "Action.pb.h"

#define ENV_SITL_PORT "GYMFC_SITL_PORT"
#define ENV_DIGITAL_TWIN_SDF "GYMFC_DIGITAL_TWIN_SDF"
#define ENV_NUM_MOTORS "GYMFC_NUM_MOTORS"
#define ENV_SUPPORTED_SENSORS "GYMFC_SUPPORTED_SENSORS"

namespace gazebo {
    static const std::string kDefaultCmdPubTopic = "/aircraft/command/motor";
    static const std::string kDefaultImuSubTopic = "/aircraft/sensor/imu";
    static const std::string kDefaultEscSubTopic = "/aircraft/sensor/esc";
    // TODO Change link name to CoM
    const std::string DIGITAL_TWIN_ATTACH_LINK = "base_link";
    const std::string kTrainingRigModelName = "attitude_control_training_rig";

    const std::string kAircraftConfigFileName = "libAircraftConfigPlugin.so";

    typedef const boost::shared_ptr<const sensor_msgs::msgs::Imu> ImuPtr;
    typedef const boost::shared_ptr<const sensor_msgs::msgs::EscSensor> EscSensorPtr;

    /// \brief List of all supported sensors. The client must
    // tell us which ones it will use. The client must be aware of the
    // digitial twin they are using and that it supports the corresponding
    // sensors.
    enum Sensors {
        IMU,
        ESC,
        BATTERY
    };

    class FlightControllerPlugin : public WorldPlugin {
        /// \brief Constructor.
    public:
        FlightControllerPlugin();

        /// \brief Destructor.
        ~FlightControllerPlugin();

        /// \brief The function called when the plugin in loaded
        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    private:
        void LoadVars();

        /// \brief Parse and process SDF
    public:
        void ProcessSDF(sdf::ElementPtr _sdf);

        /// \brief Dynamically load the digitial twin from the location
        // specified by the environment variable.
    private:
        void LoadDigitalTwin();

        void ParseDigitalTwinSDF();

        /// \brief Main loop thread waiting for incoming UDP packets
    public:
        void LoopThread();

        /// \brief Bind to the specified port to receive UDP packets
        bool Bind(const char *_address, const uint16_t _port);

        /// \brief Helper to make a socket
        void MakeSockAddr(const char *_address, const uint16_t _port, struct sockaddr_in &_sockaddr);

        /// \brief Receive action including motor commands
    private:
        bool ReceiveAction();

        /// \brief Initialize a single protobuf state that is
        // reused throughout the simulation.
        void InitState();

        /// \brief Send current state
        void SendState() const;

        /// \brief Reset the world time and model, differs from
        // world reset such that the random number generator is not
        // reset.
        void SoftReset();


        /// \brief Callback from the digital twin to recieve ESC sensor values
        // where each ESC/motor will be a separate message
        void EscSensorCallback(EscSensorPtr &_escSensor);

        /// \brief Callback from the digital twin to recieve IMU values
        void ImuCallback(ImuPtr &_imu);

        void CalculateCallbackCount();

        void ResetCallbackCount();

        // \brief Calling GetLink from a model will not traverse nested models
        // until found, this function will find a link name from the
        // entire model
        physics::LinkPtr FindLinkByName(physics::ModelPtr _model, std::string _linkName);

        /// \brief Block until the sensors are within a certain threshold. Useful for
        // flushing remote sensors at the beinning of a new task.
        void FlushSensors();

        /// \brief Block until all the callbacks for the supported sneors
        // are recieved.
        void WaitForSensorsThenSend();

        bool SensorEnabled(Sensors _sensor);

        std::string robotNamespace;

        /// \brief Main loop thread for the server
        boost::thread callbackLoopThread;

    public:
        /// \brief Pointer to the world
        physics::WorldPtr world;

        /// \brief Pointer to the update event connection.
        event::ConnectionPtr updateConnection;

        /// \brief keep track of controller update sim-time.
        gazebo::common::Time lastControllerUpdateTime;

        /// \brief Controller update mutex.
        std::mutex mutex;

        /// \brief Socket handle
        int handle;

        struct sockaddr_in remaddr;

    public:
        socklen_t remaddrlen;

        /// \brief number of times ArduCotper skips update
    public:
        int connectionTimeoutCount;

        /// \brief number of times ArduCotper skips update
        /// before marking Quadcopter offline
    public:
        int connectionTimeoutMaxCount;

        /// \brief File path to the digital twin SDF
    private:
        std::string digitalTwinSDF;

        std::string cmdPubTopic;
        std::string imuSubTopic;
        std::string escSubTopic;
        transport::NodePtr nodeHandle;
        // Now define the communication channels with the digital twin
        // The architecure treats this world plugin as the flight controller
        // while all other aircraft components are now external and communicated
        // over protobufs
        transport::PublisherPtr cmdPub;

        // Subscribe to all possible sensors
        transport::SubscriberPtr imuSub;
        std::vector <transport::SubscriberPtr> escSub;
//        cmd_msgs::msgs::MotorCommand cmdMsg;
//        mav_msgs::msgs::CommandMotorSpeed newCmdMsg;

        /// \brief Current callback count incremented as sensors are pbulished
        int sensorCallbackCount{0};
        int numSensorCallbacks{0};

        boost::condition_variable callbackCondition;

        gymfc::msgs::State state;
        gymfc::msgs::Action action;
        std::vector <Sensors> supportedSensors;

        int numActuators;
        sdf::SDFPtr sdfElement;
        std::string centerOfThrustReferenceLinkName;
        ignition::math::Vector3d cot;
        sdf::ElementPtr modelElement;


        gazebo::physics::JointPtr ballJoint;
        ignition::math::Vector3d ballJointForce;
    };
}
#endif
