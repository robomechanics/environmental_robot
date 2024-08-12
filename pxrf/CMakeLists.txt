cmake_minimum_required(VERSION 3.0.2)

project(pxrf)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

## Find necessary QT packages
find_package(Qt5 COMPONENTS Core WebSockets REQUIRED)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    rospy 
    std_msgs 
    actionlib
    actionlib_msgs
    message_generation
)

add_action_files(
    DIRECTORY action
    FILES TakeMeasurement.action
)

add_message_files(
    FILES
    PxrfMsg.msg
    CompletedScanData.msg
)

add_service_files(
  FILES
  GetPxrf.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
    actionlib_msgs
)

## Declare a catkin package
catkin_package(
    CATKIN_DEPENDS message_runtime
)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(
    main
    src/main.cpp
    src/msgfactory.cpp
    src/vantacommunicator.cpp
    src/websocketclient.cpp
)

target_link_libraries(
    main
    ${catkin_LIBRARIES}
    Qt5::Core
    Qt5::WebSockets
)

catkin_install_python(PROGRAMS
	scripts/pxrf_gui.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
