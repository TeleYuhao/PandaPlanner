Header header

bool is_invalid
bool is_matched
bool is_still
uint8 cluster_mask
uint8 radar_id    # 0--f_radar, 1--fr_radar, 3--rr_radar, 5--rl_radar, 7--fl_radar
uint16 object_id  #
uint8 type        # 0--unknown 1--pedestrian 2--cyclist 3--car 4--truck

uint16 life_count  #
float32 rcs 
float32 confidence
float32 azimuth    # car-body(R-F-U)  R--0, F--90,  (0,360)

geometry_msgs/Point32 position  # relative position, car-body(R-F-U)
geometry_msgs/Vector3 velocity  # relative velocity, car-body(R-F-U)
geometry_msgs/Vector3 acceleration  # relative acceleration, car-body(R-F-U)
geometry_msgs/Vector3 dimensions  # the scale of obj  x--length, y--width, z--height

