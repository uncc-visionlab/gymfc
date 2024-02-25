cmake_minimum_required(VERSION 3.5)
project(gymfc_plugins)

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
find_package(Eigen3 REQUIRED)

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

include_directories(
  include 
  ${GAZEBO_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
#  /usr/include/eigen3
  ${EIGEN3_INCLUDE_DIR}
  )
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS ${GAZEBO_CXX_FLAGS})

#--------------------------#
# Begin Message Generation #
#--------------------------#

set(control_msgs
  msgs/MotorCommand.proto
  msgs/CommandMotorSpeed.proto
  )
set(sensor_msgs
  msgs/EscSensor.proto
  msgs/Float.proto
  msgs/Groundtruth.proto
  msgs/Imu.proto
  msgs/MagneticField.proto
  msgs/Pressure.proto
  msgs/SITLGps.proto
  msgs/State.proto
  msgs/Action.proto
  ${PROTOBUF_IMPORT_DIRS}/vector3d.proto
  ${PROTOBUF_IMPORT_DIRS}/quaternion.proto
  )

PROTOBUF_GENERATE_CPP(CTRL_PROTO_SRCS CTRL_PROTO_HDRS ${control_msgs})
PROTOBUF_GENERATE_CPP(SEN_PROTO_SRCS SEN_PROTO_HDRS ${sensor_msgs})

PROTOBUF_GENERATE_PYTHON(GYMFC_PROTO_PYTHON_SRC msgs/Action.proto msgs/State.proto)

set(CMAKE_PYTHON_SRC_INSTALL_PREFIX ${PROJECT_SOURCE_DIR}/../../../gymfc/msgs)
#add_custom_target(py_proto_targets ALL DEPENDS {GYMFC_PROTO_PYTHON_SRC})
#add_custom_command(TARGET proto_targets
#        COMMAND ${CMAKE_COMMAND} copy
#        ${GYMFC_PROTO_PYTHON_SRC}
#        ${CMAKE_PYTHON_SRC_INSTALL_PREFIX})
#foreach(TGT IN LISTS GYMFC_PROTO_PYTHON_SRC)
#  file(COPY ${TGT} DESTINATION ${CMAKE_PYTHON_SRC_INSTALL_PREFIX})
#  message("COPYING ${TGT} WILL COPY TO ${CMAKE_PYTHON_SRC_INSTALL_PREFIX}")
#endforeach ()

add_library(gymfc_control_msgs SHARED ${CTRL_PROTO_SRCS})
add_library(gymfc_sensor_msgs SHARED ${SEN_PROTO_SRCS} ${GYMFC_PROTO_PYTHON_SRC})

#------------------------#
# End Message Generation #
#------------------------#

link_libraries(gymfc_control_msgs gymfc_sensor_msgs)

add_library(FlightControllerPlugin SHARED FlightControllerPlugin.hh FlightControllerPlugin.cpp)
add_library(AircraftConfigPlugin SHARED AircraftConfigPlugin.cpp)
target_link_libraries(FlightControllerPlugin ${GAZEBO_LIBRARIES})
target_link_libraries(AircraftConfigPlugin ${GAZEBO_LIBRARIES})

#set(CMAKE_PYTHON_SRC_INSTALL_PREFIX ${PROJECT_SOURCE_DIR}/gymfc/msgs)
#foreach(TGT IN LISTS GYMFC_PROTO_PYTHON_SRC)
  #install(TARGETS ${TGT} DESTINATION ${CMAKE_PYTHON_SRC_INSTALL_PREFIX})
  #message("TARGET=${TGT} WILL INSTALL TO ${CMAKE_PYTHON_SRC_INSTALL_PREFIX}")
#endforeach()
