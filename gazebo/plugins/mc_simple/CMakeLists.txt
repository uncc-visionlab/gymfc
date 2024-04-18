cmake_minimum_required(VERSION 2.8.3)
project(fdcl_uav_plugins)

# Set c11
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(GNUInstallDirs)

#######################
## Find Dependencies ##
#######################

find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})

# XXX this approach is extremely error prone
# it would be preferable to either depend on the
# compiled headers from Gazebo directly
# or to have something entirely independent.
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

find_package(Eigen3 QUIET)
if (NOT EIGEN3_FOUND)
    # Fallback to cmake_modules
    find_package(Eigen QUIET)
    if (NOT EIGEN_FOUND)
        pkg_check_modules(EIGEN3 REQUIRED eigen3)
    else ()
        set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
        set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
    endif ()
else ()
    set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif ()

###########
## Build ##
###########

add_compile_options(-Wno-deprecated-declarations -Wno-address-of-packed-member)

set(GAZEBO_MSG_INCLUDE_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
    if(ITR MATCHES ".*gazebo-[0-9.]+$")
        set(GAZEBO_MSG_INCLUDE_DIRS "${ITR}/gazebo/msgs")
    endif()
endforeach()

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${CMAKE_CURRENT_BINARY_DIR} # for protobuf headers
        ${Boost_INCLUDE_DIR}
        ${EIGEN3_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}/eigen3	# Workaround for Eigen3
        ${GAZEBO_INCLUDE_DIRS}
        ${GAZEBO_MSG_INCLUDE_DIRS}
)

link_directories(
        ${GAZEBO_LIBRARY_DIRS}
        ${CMAKE_CURRENT_BINARY_DIR}
)
include_directories(${GAZEBO_INCLUDE_DIRS})

set(control_msgs
        msgs/Pose.proto
        msgs/String.proto
        msgs/Wrench.proto
        msgs/Imu.proto
)
PROTOBUF_GENERATE_CPP(CONTROL_PROTO_SRCS CONTROL_PROTO_HDRS ${control_msgs})

add_library(control_msgs SHARED ${CONTROL_PROTO_SRCS})

add_library(control_plugin
        SHARED
        src/control_plugin.cpp
        src/fdcl_matrix_utils.cpp
        src/fdcl_ros_utils.cpp
)

#add_library(gazebo_ros_p3d src/gazebo_ros_p3d.cpp)
#target_link_libraries(gazebo_ros_p3d ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#
#add_library(gazebo_ros_imu src/gazebo_ros_imu.cpp)
#target_link_libraries(gazebo_ros_imu ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#
add_library(gazebo_ros_imu_sensor src/gazebo_ros_imu_sensor.cpp)
target_link_libraries(gazebo_ros_imu_sensor ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} control_msgs)
add_dependencies(gazebo_ros_imu_sensor control_msgs)

target_link_libraries(control_plugin ${GAZEBO_LIBRARIES} control_msgs)
add_dependencies(control_plugin control_msgs)
