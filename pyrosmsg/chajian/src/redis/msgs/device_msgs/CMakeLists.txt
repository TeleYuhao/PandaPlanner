cmake_minimum_required(VERSION 2.8.3)
project(device_msgs)

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
             deviceStatus.msg
             imageListResult.msg
             imageReq.msg
             imageResult.msg
             mainCar.msg
             participantTrajectories.msg
             posArray.msg
             simulationPublish.msg
             taskProtocols.msg
             taskpublish.msg
             tessReq.msg
             tessResult.msg
             trajectory.msg
             startTask.msg
             taskstatus.msg
)            

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        std_msgs

)

catkin_package(
        CATKIN_DEPENDS
        message_runtime
        std_msgs
)
