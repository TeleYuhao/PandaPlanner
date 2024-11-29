Header header

uint8  sensor_id  # sensor id, 0-front_long_focus, 1-front_short_focus, 2-right, 3-rear, 4-left
uint32 object_id
float32 detect_confidence
float32 type_confidence
float32 azimuth
float32 yaw  # car-body(R-F-U)  R--0, F--90,  (0,360)

uint8 type  # 0--unknown 1--pedestrian 2--cyclist 3--car 4--truck
float32 tracking_time   #
int8 tracking_level     #
int8 lane_assignment    #

geometry_msgs/Point32 position  # relative position, car-body(R-F-U)
geometry_msgs/Vector3 velocity  # relative velocity, car-body(R-F-U)
geometry_msgs/Vector3 acceleration  # relative acceleration, car-body(R-F-U)
geometry_msgs/Vector3 dimensions  # the scale of obj  x--length, y--width, z--height

Point2D pixel_central_point 
Point2D pixel_box_size


 


