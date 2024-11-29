Header header

uint32 object_id
uint8  type  # 0--unknown 1--pedestrian 2--cyclist 3--car 4--truck

float32 detect_confidence
float32 type_confidence

geometry_msgs/Vector3 dimensions  # the scale of obj  x--length, y--width, z--height
geometry_msgs/Pose cluster_pose   # the clustered pose of obj; car-body(F-L-U)
geometry_msgs/Pose tracked_pose   # the tracked pose of obj; car-body(F-L-U)

float32 cluster_yaw  # yaw by clustering, (0,360), F--0, L--90,  car-body(F-L-U)
float32 tracked_yaw  # yaw by tracking, (0,360), F--0, L--90, car-body(F-L-U)

float32 azimuth  # (0,360), F--0, L--90, car-body(F-L-U)

geometry_msgs/Vector3 velocity       # car-body(F-L-U)
geometry_msgs/Vector3 acceleration   # car-body(F-L-U)

uint8 tracking_state  # tracker level 1--init, 4--stable, 5--cover, 10--lose
ObstacleCell[] cells  # 


