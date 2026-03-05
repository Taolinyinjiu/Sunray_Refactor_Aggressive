cmake_minimum_required(VERSION 3.0.2)
project(sunray_gimbal)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  rospy
  roscpp
  sensor_msgs
  std_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  GimbalParams.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  GimbalStatus.srv
  GimbalReboot.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs#   std_msgs
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  src/gimbal_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(image_publish src/image_publish.cpp)
target_link_libraries(image_publish ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})