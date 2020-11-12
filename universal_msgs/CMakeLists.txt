cmake_minimum_required(VERSION 2.8.3)
project(universal_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
  sensor_msgs
  message_generation
  geometry_msgs
  ur_msgs
)

# find_package(Boost REQUIRED COMPONENTS system)
# find_package(OpenCV REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Command.msg
  RobotMsg.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  DetectionTask.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  ur_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime roscpp rospy std_msgs cv_bridge image_transport sensor_msgs message_generation geometry_msgs ur_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
