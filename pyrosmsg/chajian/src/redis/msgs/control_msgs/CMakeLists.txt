cmake_minimum_required(VERSION 3.0.2)
project(control_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  rospy
  std_msgs
  common_msgs
)


## Generate messages in the 'msg' folder
add_message_files(
   DIRECTORY msg
   FILES
   VehicleCmd.msg
   VehicleDebug.msg
   VehicleFdb.msg
   ControlCmd.msg

 )


## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   common_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES diag_msgs
  CATKIN_DEPENDS 
  message_runtime
  std_msgs
  common_msgs
)




