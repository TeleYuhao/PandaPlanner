cmake_minimum_required(VERSION 3.0.2)
project(vrc_msgs)

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
)

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
  FILES
  Barrier.msg
  BranchPosition.msg
  Cloud2veh.msg
  DestLocation.msg
  DetectionData.msg
  Eventposition.msg
  InterSections.msg
  IntersectionPosition.msg
  Nodeposition.msg
  PathArry.msg
  Phases.msg
  PhasesInfo.msg
  PhaseStates.msg
  Planning.msg
  PointsPos.msg
  Polygon_point.msg
  Position.msg
  ReferencePaths.msg
  Rsis.msg
  Size.msg
  TrafficEvent.msg
  TrafficIndex.msg
  TrafficIndicInfos.msg
  TrafficLight.msg
  Velocity.msg
  VelocityCov.msg
  )


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES Vrc_msgs
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
   CATKIN_DEPENDS
   message_runtime
   std_msgs
)

