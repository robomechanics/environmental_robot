cmake_minimum_required(VERSION 3.0.2)
project(gps_navigation)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  genmsg
  std_msgs
  geometry_msgs
  #neospectra
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages, services and actions ##
################################################

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  Sample.msg
)

# Generate services in the 'srv' folder
add_service_files(
  FILES
  GetSamples.srv
  GetState.srv
  GetPoses.srv
  GetFloats.srv
  SetString.srv
  SetFloat.srv
  SaveSample.srv
  #GetNIRSample.srv
  #SaveNIRSample.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  #neospectra
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gps_navigation
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)
