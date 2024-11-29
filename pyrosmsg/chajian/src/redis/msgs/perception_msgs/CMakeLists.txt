cmake_minimum_required(VERSION 2.8.3)
project(perception_msgs)

find_package(catkin REQUIRED COMPONENTS
        message_generation
        std_msgs
        geometry_msgs
        sensor_msgs
        common_msgs
        )


## Generate messages in the 'msg' folder
add_message_files(
        DIRECTORY msg
        FILES
             CameraObjectList.msg
             CameraObject.msg
             CameraTrafficLightList.msg
             CameraTrafficLight.msg
             CameraTrafficSignList.msg
             CameraTrafficSign.msg
             LidarObjectList.msg
             LidarObject.msg
             Object.msg
             ObstacleCell.msg
             PerceptionObjects.msg
             Point2D.msg
             RadarObjectList.msg
             RadarObject.msg
             PerceptionLocalization.msg
             SingleTrafficLight.msg
             TrafficLightDetection.msg
             TrajectoryPoint.msg
             TrajectoryInfo.msg
             Trajectory.msg
	     CtrlTest.msg
	     Polygonbus.msg
	     PolygonStamped.msg
	     Point64.msg
             ParkLot.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
        sensor_msgs
        common_msgs
)

catkin_package(
        CATKIN_DEPENDS
        message_runtime
        std_msgs
        geometry_msgs
        sensor_msgs
        common_msgs
)
