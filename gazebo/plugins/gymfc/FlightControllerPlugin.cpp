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
#include <iomanip>


#include <functional>
#include <fcntl.h>
#include <cstdlib>


#ifdef _WIN32
#include <Winsock2.h>
#include <Ws2def.h>
#include <Ws2ipdef.h>
#include <Ws2tcpip.h>
using raw_type = char;
#else

#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>

using raw_type = void;
#endif

#if defined(_MSC_VER)
#include <BaseTsd.h>
typedef SSIZE_T ssize_t;
#endif

#include <mutex>
#include <string>
#include <vector>
#include <sdf/sdf.hh>
#include <boost/algorithm/string.hpp>
#include <ignition/math/Filter.hh>
#include "gazebo/util/IgnMsgSdf.hh"
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/physics/Base.hh>


#include "FlightControllerPlugin.hh"

#include "MotorCommand.pb.h"
#include "EscSensor.pb.h"
#include "Imu.pb.h"

#include "State.pb.h"
#include "Action.pb.h"

#include <gazebo/physics/dart/DARTModel.hh>
#include <gazebo/physics/dart/DARTJoint.hh>
#include <gazebo/physics/dart/DARTLink.hh>
#include <gazebo/physics/dart/DARTTypes.hh>
#include "gazebo/physics/dart/dart_inc.h"

/// \brief Obtains a parameter from sdf.
/// \param[in] _sdf Pointer to the sdf object.
/// \param[in] _name Name of the parameter.
/// \param[out] _param Param Variable to write the parameter to.
/// \param[in] _default_value Default value, if the parameter not available.
/// \param[in] _verbose If true, gzerror if the parameter is not available.
/// \return True if the parameter was found in _sdf, false otherwise.
template<class T>
bool getSdfParam(sdf::ElementPtr _sdf, const std::string &_name,
                 T &_param, const T &_defaultValue, const bool &_verbose = false) {
    if (_sdf->HasElement(_name)) {
        _param = _sdf->GetElement(_name)->Get<T>();
        return true;
    }

    _param = _defaultValue;
    if (_verbose) {
        gzerr << "[FlightControllerPlugin] Please specify a value for parameter ["
              << _name << "].\n";
    }
    return false;
}

// Helper function to find link with suffix 
bool hasEnding(std::string const &fullString, std::string const &ending) {
    if (fullString.length() >= ending.length()) {
        return (0 == fullString.compare(fullString.length() - ending.length(), ending.length(), ending));
    } else {
        return false;
    }
}

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(FlightControllerPlugin)

boost::mutex g_CallbackMutex;

FlightControllerPlugin::FlightControllerPlugin() {
    GOOGLE_PROTOBUF_VERIFY_VERSION;

    // socket
    this->handle = socket(AF_INET, SOCK_DGRAM /*SOCK_STREAM*/, 0);
#ifndef _WIN32
    // Windows does not support FD_CLOEXEC
    fcntl(this->handle, F_SETFD, FD_CLOEXEC);
#endif
    int one = 1;
    setsockopt(this->handle, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char *>(&one), sizeof(one));
    setsockopt(this->handle, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char *>(&one), sizeof(one));

#ifdef _WIN32
    u_long on = 1;
    ioctlsocket(this->handle, FIONBIO, reinterpret_cast<u_long FAR *>(&on));
#else
    fcntl(this->handle, F_SETFL, fcntl(this->handle, F_GETFL, 0) | O_NONBLOCK);
#endif

}

FlightControllerPlugin::~FlightControllerPlugin() {
    // Tear down the transporter
    gazebo::transport::fini();

    // Sleeps (pauses the destructor) until the thread has finished
    this->callbackLoopThread.join();
}


void FlightControllerPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {

    this->world = _world;
    this->ProcessSDF(_sdf);

    this->LoadVars();

    this->ParseDigitalTwinSDF();

    this->CalculateCallbackCount();
    this->nodeHandle = transport::NodePtr(new transport::Node());
    this->nodeHandle->Init(this->robotNamespace);

    blast3dSub = nodeHandle->Subscribe<blast3d_msgs::msgs::Blast3d>(
            blast3dSubTopic, &FlightControllerPlugin::Blast3dCallback, this);

    //Subscribe to all the sensors that are enabled
    for (auto sensor: this->supportedSensors) {
        switch (sensor) {
            case BAROMETER:
                this->baroSub = this->nodeHandle->Subscribe<sensor_msgs::msgs::Pressure>(
                        this->baroSubTopic, &FlightControllerPlugin::BarometerCallback, this);
                break;
            case ESC:
                //Each defined motor will have a unique index, since they are independent they must come in
                //as separate messages
                //XXX NOTE index starts at 1 to match that of beta flight mixer
                //for (unsigned int i = 1; i <= this->numActuators; i++)
                for (unsigned int i = 0; i < numActuators; i++) {
                    escSub.push_back(nodeHandle->Subscribe<sensor_msgs::msgs::EscSensor>(
                            this->escSubTopic + "/" + std::to_string(i), &FlightControllerPlugin::EscSensorCallback,
                            this));
                    motorWindSub.push_back(nodeHandle->Subscribe<physics_msgs::msgs::Wind>(
                            this->motorWindSubTopic + "/" + std::to_string(i),
                            &FlightControllerPlugin::MotorWindCallback,
                            this));
                }
                break;
            case GPS:
                this->gpsSub = this->nodeHandle->Subscribe<sensor_msgs::msgs::SITLGps>(
                        this->gpsSubTopic, &FlightControllerPlugin::GpsCallback, this);
                break;
            case GROUND_TRUTH:
                this->ground_truthSub = this->nodeHandle->Subscribe<sensor_msgs::msgs::Groundtruth>(
                        this->ground_truthSubTopic, &FlightControllerPlugin::GroundtruthCallback, this);
                break;
            case IMU:
                this->imuSub = this->nodeHandle->Subscribe<sensor_msgs::msgs::Imu>(
                        this->imuSubTopic, &FlightControllerPlugin::ImuCallback, this);
                break;
            case MAGNETOMETER:
                this->magnetoSub = this->nodeHandle->Subscribe<sensor_msgs::msgs::MagneticField>(
                        this->magnetoSubTopic, &FlightControllerPlugin::MagnetometerCallback, this);
                break;
        }
    }
    this->InitState();

//    this->cmdPub = this->nodeHandle->Advertise<cmd_msgs::msgs::MotorCommand>(this->cmdPubTopic);
    this->cmdPub = this->nodeHandle->Advertise<mav_msgs::msgs::CommandMotorSpeed>(this->cmdPubTopic);
    if (numActuators == 6) {
        this->fmPub = this->nodeHandle->Advertise<geometry_msgs::msgs::Wrench>(this->fmPubTopic);
    }

    // Force pause because we drive the simulation steps
    this->world->SetPaused(true);
    this->callbackLoopThread = boost::thread(boost::bind(&FlightControllerPlugin::LoopThread, this));
}

bool FlightControllerPlugin::SensorEnabled(Sensors _sensor) {
    for (auto sensor: this->supportedSensors) {
        if (sensor == _sensor) {
            return true;
        }
    }
    return false;
}

void FlightControllerPlugin::LoadVars() {

    //XXX This is getting messy, if there is anything the plugin needs
    //we need to switch to a config file

    // Default port can be read in from an environment variable
    // This allows multiple instances to be run
    int port = 9002;
    if (const char *env_p = std::getenv(ENV_SITL_PORT)) {
        port = std::stoi(env_p);
    }
    gzdbg << "Binding on port " << port << "\n";
    if (!this->Bind("127.0.0.1", port)) {
        gzerr << "failed to bind with 127.0.0.1:" << port << ", aborting plugin.\n";
        return;
    }
    if (const char *env_p = std::getenv(ENV_DIGITAL_TWIN_SDF)) {
        this->digitalTwinSDF = env_p;
    } else {
        gzerr << "Could not load digital twin model from environment variable " << ENV_DIGITAL_TWIN_SDF << "\n";
        return;
    }
}

void FlightControllerPlugin::InitState() {
    for (unsigned int i = 0; i < 3; i++) {
        this->state.add_force(0);
    }

    // Initialize the state of the sensors to a value that reflect the aircraft in an active state thus
    // forcing the sensors to be flushed.
    for (unsigned int i = 0; i < 3; i++) {
        this->state.add_imu_angular_velocity_rpy(1);
        // TODO
        this->state.add_imu_linear_acceleration_xyz(0);
    }
    for (unsigned int i = 0; i < 4; i++) {
        // TODO
        this->state.add_imu_orientation_quat(0);
    }
    // ESC sensor
    for (unsigned int i = 0; i < this->numActuators; i++) {
        this->state.add_esc_motor_angular_velocity(100);
        this->state.add_esc_temperature(10000);
        this->state.add_esc_current(-1);
        this->state.add_esc_voltage(-1);
        this->state.add_esc_torque(0);
        this->state.add_esc_force(0);
    }
    for (unsigned int i = 0; i < 3; i++) {
        this->state.add_gps_wgs84_pos(0);
        this->state.add_gps_enu_vel(0);
        this->state.add_magneto_magnetic_field(0);
        this->state.add_gt_wgs84_pos(0);
        this->state.add_gt_enu_vel(0);
    }
    for (unsigned int i = 0; i < 4; i++) {
        this->state.add_gt_attitude_quat(0);
    }
    for (unsigned int i = 0; i < 12; i++) {
        this->state.add_motor_wind3d_xyz(0);
    }
    for (unsigned int i = 0; i < 3; i++) {
        this->state.add_blast3d_pos(0);
    }
}

void FlightControllerPlugin::EscSensorCallback(EscSensorPtr &_escSensor) {
    //gzdbg << "Received ESC msg " << sensorCallbackCount << std::endl;
    uint32_t id = _escSensor->id();
    boost::mutex::scoped_lock lock(g_CallbackMutex);
    //gzdbg << "\nReceived ESC " << id << " speed=" <<  _escSensor->motor_speed() << std::endl;
    this->state.set_esc_motor_angular_velocity(id, _escSensor->motor_speed());
    this->state.set_esc_temperature(id, _escSensor->temperature());
    this->state.set_esc_current(id, _escSensor->current());
    this->state.set_esc_voltage(id, _escSensor->voltage());
    this->state.set_esc_force(id, _escSensor->force());
    this->state.set_esc_torque(id, _escSensor->torque());
    this->sensorCallbackCount++;
    this->callbackCondition.notify_all();
}

void FlightControllerPlugin::MotorWindCallback(WindPtr &_motorWind) {
    boost::mutex::scoped_lock lock(g_CallbackMutex);
//    gzdbg << __FUNCTION__ << "(): Received Wind3d message for motor " << _motorWind->frame_id() << std::endl;
    int motor_number = std::stoi(_motorWind->frame_id());
    this->state.set_motor_wind3d_xyz(3 * motor_number + 0, _motorWind->velocity().x());
    this->state.set_motor_wind3d_xyz(3 * motor_number + 1, _motorWind->velocity().y());
    this->state.set_motor_wind3d_xyz(3 * motor_number + 2, _motorWind->velocity().z());
}

void FlightControllerPlugin::Blast3dCallback(Blast3dPtr &_blast3d) {
//    gzdbg << __FUNCTION__ << "(): Received Blast3d message for time " << _blast3d->time() << std::endl;
    boost::mutex::scoped_lock lock(g_CallbackMutex);
    this->state.set_blast3d_time(_blast3d->time());
    this->state.set_blast3d_weight_tnt(_blast3d->weight_tnt_kg());
    this->state.set_blast3d_pos(0, _blast3d->x());
    this->state.set_blast3d_pos(1, _blast3d->y());
    this->state.set_blast3d_pos(2, _blast3d->z());
}

void FlightControllerPlugin::GroundtruthCallback(GroundtruthPtr &_gt) {
    boost::mutex::scoped_lock lock(g_CallbackMutex);
    //gzdbg << "Got ground truth msg." << std::endl;
    this->state.set_gt_wgs84_pos(0, _gt->latitude_rad() * 180.0 / M_PI);
    this->state.set_gt_wgs84_pos(1, _gt->longitude_rad() * 180.0 / M_PI);
    this->state.set_gt_wgs84_pos(2, _gt->altitude());
    this->state.set_gt_enu_vel(0, _gt->velocity_east());
    this->state.set_gt_enu_vel(1, _gt->velocity_north());
    this->state.set_gt_enu_vel(2, _gt->velocity_up());
    this->state.set_gt_attitude_quat(0, _gt->attitude_q_x());
    this->state.set_gt_attitude_quat(1, _gt->attitude_q_y());
    this->state.set_gt_attitude_quat(2, _gt->attitude_q_z());
    this->state.set_gt_attitude_quat(3, _gt->attitude_q_w());
    this->sensorCallbackCount++;
    this->callbackCondition.notify_all();
}

void FlightControllerPlugin::GpsCallback(GpsPtr &_gps) {
    boost::mutex::scoped_lock lock(g_CallbackMutex);
    //last_gps_msg = *_gps;
    //gzdbg << "Got GPS msg." << std::endl;
    this->state.set_gps_wgs84_pos(0, _gps->latitude_deg());
    this->state.set_gps_wgs84_pos(1, _gps->longitude_deg());
    this->state.set_gps_wgs84_pos(2, _gps->altitude());
    this->state.set_gps_eph(_gps->eph());
    this->state.set_gps_epv(_gps->epv());
    this->state.set_gps_ground_vel(_gps->velocity());
    this->state.set_gps_enu_vel(0, _gps->has_velocity_east());
    this->state.set_gps_enu_vel(1, _gps->has_velocity_north());
    this->state.set_gps_enu_vel(2, _gps->has_velocity_up());
    this->sensorCallbackCount++;
    this->callbackCondition.notify_all();
}

void FlightControllerPlugin::BarometerCallback(PressurePtr &_pressure) {
    boost::mutex::scoped_lock lock(g_CallbackMutex);
    //last_barometer_msg = *_pressure;
    //gzdbg << "Got pressure msg." << std::endl;
    this->state.set_baro_temperature(_pressure->temperature());
    this->state.set_baro_absolute_pressure(_pressure->absolute_pressure());
    this->state.set_baro_pressure_altitude(_pressure->pressure_altitude());
    //this->sensorCallbackCount++;
    this->callbackCondition.notify_all();
}

void FlightControllerPlugin::MagnetometerCallback(MagneticFieldPtr &_magneto) {
    boost::mutex::scoped_lock lock(g_CallbackMutex);
    //last_magnetometer_msg = *_magneto;
    //gzdbg << "Got magnetic field msg." << std::endl;
    this->state.set_magneto_magnetic_field(0, _magneto->magnetic_field().x());
    this->state.set_magneto_magnetic_field(1, _magneto->magnetic_field().y());
    this->state.set_magneto_magnetic_field(2, _magneto->magnetic_field().z());
    //this->sensorCallbackCount++;
    this->callbackCondition.notify_all();
}

void FlightControllerPlugin::ImuCallback(ImuPtr &_imu) {
    //gzdbg << "Received IMU msg " << sensorCallbackCount << std::endl;
    boost::mutex::scoped_lock lock(g_CallbackMutex);

    this->state.set_imu_angular_velocity_rpy(0, _imu->angular_velocity().x());
    this->state.set_imu_angular_velocity_rpy(1, _imu->angular_velocity().y());
    this->state.set_imu_angular_velocity_rpy(2, _imu->angular_velocity().z());

    this->state.set_imu_orientation_quat(0, _imu->orientation().w());
    this->state.set_imu_orientation_quat(1, _imu->orientation().x());
    this->state.set_imu_orientation_quat(2, _imu->orientation().y());
    this->state.set_imu_orientation_quat(3, _imu->orientation().z());

    this->state.set_imu_linear_acceleration_xyz(0, _imu->linear_acceleration().x());
    this->state.set_imu_linear_acceleration_xyz(1, _imu->linear_acceleration().y());
    this->state.set_imu_linear_acceleration_xyz(2, _imu->linear_acceleration().z());

    this->sensorCallbackCount++;
    this->callbackCondition.notify_all();
}

void FlightControllerPlugin::ProcessSDF(sdf::ElementPtr _sdf) {
    if (_sdf->HasElement("commandPubTopic")) {
        this->cmdPubTopic = _sdf->GetElement("commandPubTopic")->Get<std::string>();
    }
    if (_sdf->HasElement("imuSubTopic")) {
        this->imuSubTopic = _sdf->GetElement("imuSubTopic")->Get<std::string>();
    }
    if (_sdf->HasElement("escSubTopicPrefix")) {
        this->escSubTopic = _sdf->GetElement("escSubTopicPrefix")->Get<std::string>();
    }
    if (_sdf->HasElement("robotNamespace")) {
        this->robotNamespace = _sdf->GetElement("robotNamespace")->Get<std::string>();
    } else {
        gzerr << "[gazebo_motor_model] Please specify a robotNamespace.\n";
    }
}

void FlightControllerPlugin::SoftReset() {
    //this->world->Reset();
    this->world->ResetTime();
    this->world->ResetEntities(gazebo::physics::Base::BASE);
    this->world->ResetPhysicsStates();
}


physics::LinkPtr FlightControllerPlugin::FindLinkByName(physics::ModelPtr _model, std::string _linkName) {
    for (auto link: _model->GetLinks()) {
        //gzdbg << "Link name: " << link->GetName() << std::endl;
        if (hasEnding(link->GetName(), _linkName)) {
            return link;
        }
    }
    return NULL;
}

void FlightControllerPlugin::ParseDigitalTwinSDF() {
    // Load the root digital twin sdf file
    const std::string sdfPath(this->digitalTwinSDF);

    this->sdfElement.reset(new sdf::SDF());
    sdf::init(this->sdfElement);
    if (!sdf::readFile(sdfPath, this->sdfElement)) {
        gzerr << sdfPath << " is not a valid SDF file!" << std::endl;
        return;
    }
    const sdf::ElementPtr rootElement = this->sdfElement->Root();
    if (!rootElement->HasElement("model")) {
        gzerr << "Could not find model!" << std::endl;
        return;
    }
    this->modelElement = rootElement->GetElement("model");

    sdf::ElementPtr pluginPtr = this->modelElement->GetElement("plugin");

    //gzdbg << " Looking for plugin\n";
    bool foundAircraftConfigPlugin = false;
    while (pluginPtr) {
        if (pluginPtr->GetAttribute("filename")->GetAsString().compare(kAircraftConfigFileName) == 0) {
            foundAircraftConfigPlugin = true;
            break;
        }
        pluginPtr = pluginPtr->GetNextElement();
    }
    //gzdbg << " Done Looking for plugin\n";
    if (!foundAircraftConfigPlugin) {
        gzerr << "Could not find required " << kAircraftConfigFileName << ". Aborting!" << std::endl;
    }

    const sdf::ElementPtr centerOfThrustElement = pluginPtr->GetElement("centerOfThrust");
    this->centerOfThrustReferenceLinkName = centerOfThrustElement->Get<std::string>("link");
    gzdbg << "CoT link=" << this->centerOfThrustReferenceLinkName << std::endl;
    this->cot = centerOfThrustElement->Get<ignition::math::Vector3d>("offset");
    gzdbg << "Got COT from plugin " << this->cot.X() << " " << this->cot.Y() << " " << this->cot.Z() << std::endl;

    this->numActuators = pluginPtr->GetElement("motorCount")->Get<int>();
    gzdbg << "Num motors " << this->numActuators << std::endl;

    sdf::ElementPtr sensorsSDF = pluginPtr->GetElement("sensors");
    if (sensorsSDF == nullptr) {
        gzerr << "Could not find any sensors\n";
    } else {
        sdf::ElementPtr sensorSDF = sensorsSDF->GetElement("sensor");
        if (sensorSDF == nullptr) {
            gzerr << "sensorSDF is null!" << std::endl << std::flush;
        }
        while (sensorSDF != nullptr) {
            std::string type = sensorSDF->GetAttribute("type")->GetAsString();
            if (sensorSDF == nullptr) {
                break;
            }
            if (boost::iequals(type, "imu")) {
                this->supportedSensors.push_back(IMU);
            } else if (boost::iequals(type, "esc")) {
                this->supportedSensors.push_back(ESC);
            } else if (boost::iequals(type, "battery")) {
                this->supportedSensors.push_back(BATTERY);
            } else if (boost::iequals(type, "barometer")) {
                this->supportedSensors.push_back(BAROMETER);
            } else if (boost::iequals(type, "magnetometer")) {
                this->supportedSensors.push_back(MAGNETOMETER);
            } else if (boost::iequals(type, "gps")) {
                this->supportedSensors.push_back(GPS);
            } else if (boost::iequals(type, "groundtruth")) {
                this->supportedSensors.push_back(GROUND_TRUTH);
            }
            sensorSDF = sensorSDF->GetNextElement("sensor");
        }
    }
}

void FlightControllerPlugin::LoadDigitalTwin() {
    gzdbg << "Inserting vehicle model from SDF file: " << this->digitalTwinSDF << ".\n";
    this->ballJoint = nullptr;
    // XXX Better way to do this?
    // It appears the inserted model is not available in the world
    // right away, maybe due to the message passing that occurs?
    // For now poll until its there in the world
    unsigned int startModelCount = this->world->ModelCount();
    this->world->InsertModelSDF(*this->sdfElement);
    while (1) {
        unsigned int modelCount = this->world->ModelCount();
        if (modelCount >= startModelCount + 1) {
            break;
        } else {
            gazebo::common::Time::MSleep(100);
        }
    }

    // start parsing model
    const std::string modelName = this->modelElement->Get<std::string>("name");
    //gzdbg << "Found " << modelName << " model!" << std::endl;
    // Now get a pointer to the model
    physics::ModelPtr model = this->world->ModelByName(modelName);
    if (!model) {
        gzerr << "Could not access model " << modelName << " from world" << std::endl;
        return;
    }

    if (this->world->Name().compare("default") != 0) {
        gzdbg << "Using dyno, not linking aircraft to world" << std::endl;
        return;
    }

    physics::ModelPtr supportModel = this->world->ModelByName(kTrainingRigModelName);
    if (!supportModel) {
        gzerr << "Could not find training rig model." << std::endl;
        return;
    }

    gazebo::physics::LinkPtr centerOfThrustReferenceLink;
    centerOfThrustReferenceLink = FindLinkByName(model, this->centerOfThrustReferenceLinkName);
    if (!centerOfThrustReferenceLink) {
        gzerr << "Could not find the CoT link" << std::endl;
        return;
    }

    // Create the ball joint to attach the aircraft too
    gazebo::physics::JointPtr joint;
    joint = this->world->Physics()->CreateJoint("ball", supportModel);
    joint->SetName("ball_joint");
    joint->Attach(supportModel->GetLink("pivot"), centerOfThrustReferenceLink);
    this->ballJoint = joint;

    if (this->world->Physics()->GetType().compare("dart") == 0) {
        // Discussed here https://www.reddit.com/r/robotics/comments/5a7xrl/bullet_vs_ode_to_simulate_robotic_arm/

        // Do all the casting
        dart::dynamics::SkeletonPtr aircraftSkeleton = boost::dynamic_pointer_cast<gazebo::physics::DARTModel>(
                model)->DARTSkeleton();
        gazebo::physics::DARTModelPtr dartSupportModelPtr = boost::dynamic_pointer_cast<gazebo::physics::DARTModel>(
                supportModel);
        gazebo::physics::DARTJointPtr dartJoint = boost::dynamic_pointer_cast<gazebo::physics::DARTJoint>(joint);
        gazebo::physics::DARTLinkPtr dartLink = boost::dynamic_pointer_cast<gazebo::physics::DARTLink>(
                centerOfThrustReferenceLink);

        // Do all this just to get the dark joint
        std::shared_ptr<dart::dynamics::Joint::Properties> jointProperties = dartJoint->DARTProperties();
        std::pair<dart::dynamics::Joint *, dart::dynamics::BodyNode *> pair;
        dart::dynamics::BodyNode::AspectProperties properties("testbody");
        pair = aircraftSkeleton->createJointAndBodyNodePair<dart::dynamics::BallJoint, dart::dynamics::BodyNode>(
                dartSupportModelPtr->DARTSkeleton()->getBodyNode("pivot"),
                static_cast<const dart::dynamics::BallJoint::Properties &>(*jointProperties),
                properties
        );
        dart::dynamics::Joint *newJoint = pair.first;
        // XXX Cant initialize until SetDARTJoint called
        dartJoint->SetDARTJoint(newJoint);

        // This is required to constrain the distance of the child link
        // For some reason the Load implementation is missing this
        Eigen::Vector3d location = Eigen::Vector3d(this->cot.X(), this->cot.Y(), this->cot.Z());
        dart::constraint::BallJointConstraintPtr mBallConstraint;
        gzdbg << "Setting link to center " << centerOfThrustReferenceLink->GetName() << std::endl;
        mBallConstraint = std::make_shared<dart::constraint::BallJointConstraint>(aircraftSkeleton->getBodyNode(
                centerOfThrustReferenceLink->GetName()), location);
        dartLink->DARTWorld()->getConstraintSolver()->addConstraint(mBallConstraint);
    } else {
        joint->Load(supportModel->GetLink("pivot"), centerOfThrustReferenceLink,
                    ignition::math::Pose3d(this->cot.X(), this->cot.Y(), this->cot.Z(), 0, 0, 0));
    }

    joint->Init();

    // This is actually great because we've removed the ground plane so there is no possible collision
    gzdbg << "Aircraft model fixed to world\n";
}

void FlightControllerPlugin::FlushSensors() {
    // Make sure we do a reset on the time even if sensors are within range
    // XXX The first episode the sensor values are set to some active value
    // to force plugins to send action state.
    this->SoftReset();

    /*
    cmd_msgs::msgs::MotorCommand cmd;
    for (unsigned int i = 0; i < this->numActuators; i++)
    {
      cmd.add_motor(0);
    }
    this->cmdPub->Publish(cmd);
    this->world->Step(1);
    */


    // TODO: this must be based on the units or come up with something generic
    double error = 0.017;// About 1 deg/s
    while (true) {
        // Pitch and Yaw are negative
        //gzdbg << " Size =" << this->state.imu_angular_velocity_rpy_size() << std::endl;
        //gzdbg << "IMU [" << this->state.imu_angular_velocity_rpy(0) << "," << this->state.imu_angular_velocity_rpy(1) << "," << this->state.imu_angular_velocity_rpy(2) << "]" << std::endl;
        if (this->state.imu_angular_velocity_rpy_size() < 2 ||
            (
                    std::abs(this->state.imu_angular_velocity_rpy(0)) > error ||
                    std::abs(this->state.imu_angular_velocity_rpy(1)) > error ||
                    std::abs(this->state.imu_angular_velocity_rpy(2)) > error)) {
            gzdbg << "Gyro r=" << this->state.imu_angular_velocity_rpy(0) << " p="
                  << this->state.imu_angular_velocity_rpy(1) << " y=" << this->state.imu_angular_velocity_rpy(2)
                  << "\n";
            //gzdbg << "Error too great" << std::endl;
            //
            // Trigger all plugins to publish their values
            this->world->Step(1);
            // XXX
            //this->SoftReset();

            //boost::this_thread::sleep(boost::posix_time::milliseconds(100));
        } else {
            //gzdbg << "Target velocity reached! r=" << rates.X() << " p=" << rates.Y() << " y=" << rates.Z() << "\n";
            break;
        }
    }

    this->SoftReset();

}

void FlightControllerPlugin::LoopThread() {

    this->LoadDigitalTwin();
    while (1) {
        bool ac_received = this->ReceiveAction();
        //ignition::math::Vector3d f = this->world->ModelByName(kTrainingRigModelName)->GetLink("pivot")->RelativeForce();

        //gzdbg << "Received?" << ac_received << std::endl;
        if (!ac_received) {
            continue;
        }

        /* XXX This is a way to get force applied to possibly use for reward   
		 */
        if (this->ballJoint != nullptr) {
            this->ballJointForce = this->ballJoint->GetForceTorque(0).body1Force;
        }
        //gzdbg << "Force X=" << f.X() << " Y=" << f.Y() << " Z=" << f.Z() << std::endl;
        //ignition::math::Vector3d f2 = this->ballJoint->GetForceTorque(0).body2Force;
        //gzdbg << "Force Body 2 X=" << f2.X() << " Y=" << f2.Y() << " Z=" << f2.Z() << std::endl;

        // Handle reset command
        //  if (this->world->Name().compare("default") == 0)
        // {
        if (this->action.world_control() == gymfc::msgs::Action::RESET) {
            gzdbg << " Flushing sensors..." << std::endl;
            // Block until we get response from sensors
            this->FlushSensors();
            gzdbg << " Sensors flushed." << std::endl;
            this->state.set_sim_time(this->world->SimTime().Double());
            this->state.set_status_code(gymfc::msgs::State_StatusCode_OK);
            this->SendState();
            continue;
        }
        //}

        this->ResetCallbackCount();
        //gzdbg << "Callback count " << this->sensorCallbackCount << std::endl;
        //Forward the motor commands from the agent to each motor
        //cmd_msgs::msgs::MotorCommand cmd;
        if (numActuators > 0) { //} && numActuators != 6) {
            //gzdbg << "Sending " << numActuators << " motor commands to digital twin." << std::endl;
            mav_msgs::msgs::CommandMotorSpeed cmd;
            for (unsigned int i = 0; i < this->numActuators; i++) {
                // gzdbg << i << "=" << this->action.motor(i) << std::endl;
                //cmd.add_motor(this->action.motor(i));
                cmd.add_motor_speed(this->action.motor(i));
            }
            //gzdbg << "Publishing motor command\n";
            this->cmdPub->Publish(cmd);
            //gzdbg << "Done publishing motor command\n";
        }
        if (numActuators == 6) {
            geometry_msgs::msgs::Wrench fmCmd;
            gazebo::msgs::Vector3d *force = new gazebo::msgs::Vector3d();
            force->set_x(this->action.motor(0));
            force->set_y(this->action.motor(1));
            force->set_z(this->action.motor(2));
            fmCmd.set_allocated_force(force);
            gazebo::msgs::Vector3d *torque = new gazebo::msgs::Vector3d();
            torque->set_x(this->action.motor(3));
            torque->set_y(this->action.motor(4));
            torque->set_z(this->action.motor(5));
            fmCmd.set_allocated_torque(torque);
//            gzdbg << "Sending force " << force->z() << " and torque = (" << torque->x()
//                  << ", " << torque->y() << ", " << torque->z() << ")" << std::endl;
            this->fmPub->Publish(fmCmd);
        }
        // Triggers other plugins to publish
        this->world->Step(1);
        // gzdbg << "Waiting..." << std::endl;
        this->WaitForSensorsThenSend();
    }
}

void FlightControllerPlugin::ResetCallbackCount() {
    boost::mutex::scoped_lock lock(g_CallbackMutex);
    this->sensorCallbackCount = -1 * this->numSensorCallbacks;
}

void FlightControllerPlugin::CalculateCallbackCount() {
    // Reset the callback count, once we step the sim all the new
    // vales will be published

    for (auto sensor: this->supportedSensors) {
        switch (sensor) {
            // These sensors may not provide an output after an OnUpdate() call
            case BAROMETER:
            case MAGNETOMETER:
            case GPS:
                break;
            case BATTERY:
            case GROUND_TRUTH:
            case IMU:
                this->numSensorCallbacks += 1;
                break;
            case ESC:
                this->numSensorCallbacks += this->numActuators;
                break;
        }
    }
}

void FlightControllerPlugin::WaitForSensorsThenSend() {
    if (this->ballJoint != nullptr) {
        this->state.set_force(0, this->ballJointForce.X());
        this->state.set_force(1, this->ballJointForce.Y());
        this->state.set_force(2, this->ballJointForce.Z());
    }
    this->state.set_sim_time(this->world->SimTime().Double());
    this->state.set_status_code(gymfc::msgs::State_StatusCode_OK);

    boost::mutex::scoped_lock lock(g_CallbackMutex);
    while (this->sensorCallbackCount < 0) {
        //gzdbg << "Callback count = " << this->sensorCallbackCount << std::endl;
        this->callbackCondition.wait(lock);
    }
    /*
    while (this->sensorCallbackCount < 0)
    {
          boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

    }
    */
    //gzdbg << "Sending state"<<std::endl;
    this->SendState();
}

bool FlightControllerPlugin::Bind(const char *_address, const uint16_t _port) {
    struct sockaddr_in sockaddr;
    this->MakeSockAddr(_address, _port, sockaddr);

    if (bind(this->handle, (struct sockaddr *) &sockaddr, sizeof(sockaddr)) != 0) {
        shutdown(this->handle, 0);
#ifdef _WIN32
        closesocket(this->handle);
#else
        close(this->handle);
#endif
        return false;
    }
    return true;
}

void FlightControllerPlugin::MakeSockAddr(const char *_address, const uint16_t _port,
                                          struct sockaddr_in &_sockaddr) {
    memset(&_sockaddr, 0, sizeof(_sockaddr));

#ifdef HAVE_SOCK_SIN_LEN
    _sockaddr.sin_len = sizeof(_sockaddr);
#endif

    _sockaddr.sin_port = htons(_port);
    _sockaddr.sin_family = AF_INET;
    _sockaddr.sin_addr.s_addr = inet_addr(_address);
}

bool FlightControllerPlugin::ReceiveAction() {

    // TODO: What should the buf size be? How do we estimate the protobuf size?
    unsigned int buf_size = 1024;
    char buf[buf_size];

    int recvSize;
    recvSize = recvfrom(this->handle, buf, buf_size, 0, (struct sockaddr *) &this->remaddr, &this->remaddrlen);

    if (recvSize < 0) {
        return false;
    }
    /*
    for (int i = 0; i < recvSize; ++i)
    {
          std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)buf[i] << " ";
    }
    std::cout << std::endl;
    */

    std::string msg;
    // Do the reassignment because protobuf needs string
    msg.assign(buf, recvSize);
    this->action.ParseFromString(msg);
    //gzdbg << " Motor Size " << this->action.motor_size() << std::endl;
    //gzdbg << " World Control " << this->action.world_control() << std::endl;

    return true;
}

/////////////////////////////////////////////////
void FlightControllerPlugin::SendState() const {
    std::string buf;
    this->state.SerializeToString(&buf);

    //gzdbg << " Buf data= " << buf.data() << std::endl;
    //gzdbg << "State Buf size= " << buf.size() << std::endl;
    ::sendto(this->handle,
             buf.data(),
             buf.size(), 0,
             (struct sockaddr *) &this->remaddr, this->remaddrlen);
}



