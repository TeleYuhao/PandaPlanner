cmake_minimum_required(VERSION 2.8.3)
project(common_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
)


 add_message_files(
    DIRECTORY msg
    FILES

    actuator.msg
    app.msg
    cloudpant.msg
    cloudpants.msg
    collectmap.msg
    collectpoint.msg
    controllat.msg
    controllon.msg
    decisionbehavior.msg
    elapsedtime.msg
    error.msg
    fusiontrackingobject.msg
    fusiontrackingobjects.msg
    intersectionroad.msg
    intersectionroads.msg
    lane.msg
    lanes.msg
    lonlat.msg
    lonlatmappoints.msg
    monitor.msg
    objecthistory.msg
    objectprediction.msg
    obupant.msg
    obupants.msg
    oburoadlist.msg
    oburoadpoint.msg
    planningmotion.msg
    point3d.msg
    rdcontrol.msg
    remotedrivestatus.msg
    requestmap.msg
    roadpoint.msg
    sensorcameralight.msg
    sensorgps.msg
    sensorobject.msg
    sensorobjects.msg
    sensorstatus.msg
    sl.msg
    trajectorypoints.msg
    v2xapp.msg
    obulight.msg
    obutrafficlights.msg
    pullover.msg
    mapformat.msg
    pointformat.msg
    padtohd.msg
    hdintersectiontoglobal.msg
    hdintersectionstoglobal.msg
    hdroute.msg
    hdmap.msg
    hdroutestopad.msg
    hdroutetoglobal.msg
    hdstoppointstoglobal.msg
    FaultInfo.msg
    FaultVec.msg
    Header.msg
    TimeStatistics.msg
    TimeStatus.msg
    DRPoseWithTime.msg
    EulerWithCovariance.msg
    LLH.msg
    NavStatus.msg
    Pose.msg
    PoseEuler.msg
    PoseQuaternion.msg
    Quaternion.msg
    TimeStatistics.msg
    TimeStatus.msg
    Twist.msg
    UnsureVar.msg
    Vector3.msg
    Vector3WithCovariance.msg
    CalibrationResult.msg
    LidarCalibrationResult.msg
    UdpMsg.msg
 )

add_service_files(FILES
  TranslateCoordinate.srv
  TimeSyncmsg.srv
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

