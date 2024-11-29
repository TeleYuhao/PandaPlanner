Header header

int8 sensor_id  # sensor id, 0-front_long_focus, 1-front_short_focus, 2-right, 3-rear, 4-left
int32 object_id
float32 detect_confidence
float32 type_confidence

geometry_msgs/Point32 position  # relative position, car-body(R-F-U)

Point2D pixel_central_point 
Point2D pixel_box_size

uint8 light_color  # 0--rad, 1--green, 2--yellow, 3--off or unknow
uint8 light_shape  # 0--round, 1--left_arrow, 2--other

