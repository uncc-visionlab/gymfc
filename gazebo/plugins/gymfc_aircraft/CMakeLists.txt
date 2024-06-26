cmake_minimum_required(VERSION 3.5)
project(gymfc_aircraft_plugins)

find_package(Boost 1.58 REQUIRED COMPONENTS system thread timer)
find_package(gazebo REQUIRED)

#
# Find Protobuf
#
set(PROTOBUF_IMPORT_DIRS "")
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()

# PROTOBUF_IMPORT_DIRS has to be set before
# find_package is called
find_package(Protobuf REQUIRED)
pkg_check_modules(PROTOBUF protobuf)

if ("${PROTOBUF_VERSION}" VERSION_LESS "2.5.0")
  message(FATAL_ERROR "protobuf version: ${PROTOBUF_VERSION} not compatible, must be >= 2.5.0")
endif()

if("${GAZEBO_VERSION}" VERSION_LESS "6.0")
  message(FATAL_ERROR "You need at least Gazebo 6.0. Your version: ${GAZEBO_VERSION}")
else()
  message("Gazebo version: ${GAZEBO_VERSION}")
endif()
#
# End Find Protobuf
#

find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(Eigen QUIET)
  if(NOT EIGEN_FOUND)
    pkg_check_modules(EIGEN3 REQUIRED eigen3)
  else()
    set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
    set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
  endif()
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

include_directories(
    include 
    ${CMAKE_CURRENT_BINARY_DIR}
    ${Boost_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}/eigen3	# Workaround for Eigen3
    ${GAZEBO_INCLUDE_DIRS}
    ${GAZEBO_MSG_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

list(APPEND CMAKE_CXX_FLAGS ${GAZEBO_CXX_FLAGS})

#--------------------------#
# Begin Message Generation #
#--------------------------#

set(gymfc_aircraft_control_msgs
    msgs/MotorCommand.proto
  )
set(gymfc_aircraft_sensor_msgs
  msgs/Float.proto
  msgs/Imu.proto
  msgs/MotorSpeed.proto
  msgs/EscSensor.proto
  ${PROTOBUF_IMPORT_DIRS}/vector3d.proto
  ${PROTOBUF_IMPORT_DIRS}/quaternion.proto
  )

PROTOBUF_GENERATE_CPP(GYMFC_AIR_CTRL_PROTO_SRCS CTRL_PROTO_HDRS ${gymfc_aircraft_control_msgs})
PROTOBUF_GENERATE_CPP(GYMFC_AIR_SEN_PROTO_SRCS SEN_PROTO_HDRS ${gymfc_aircraft_sensor_msgs})

add_library(gymfc_aircraft_control_msgs SHARED ${GYMFC_AIR_CTRL_PROTO_SRCS})
add_library(gymfc_aircraft_sensor_msgs SHARED ${GYMFC_AIR_SEN_PROTO_SRCS})

#------------------------#
# End Message Generation #
#------------------------#

link_libraries(gymfc_aircraft_control_msgs gymfc_aircraft_sensor_msgs)

add_library(gymfc_motor_model SHARED src/gymfc_motor_model.h src/gymfc_motor_model.cpp)
add_library(gymfc_imu_plugin SHARED src/gymfc_motor_model.h src/gymfc_imu_plugin.cpp)

set(plugins
    gymfc_motor_model
    gymfc_imu_plugin
    )
foreach(plugin ${plugins})
  target_link_libraries(${plugin} ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES})
endforeach()


