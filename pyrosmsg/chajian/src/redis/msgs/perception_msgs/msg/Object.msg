uint32 id 		# obstacle ID.
float32 x  		# relative distance of x, FLU车体坐标系, unit: m
float32 y 		# relative distance of y, FLU车体坐标系, unit: m
float32 z 		# relative distance of z, FLU车体坐标系, unit: m
float32 vxrel     # relative speed of x, FLU车体坐标系, unit: m/s
float32 vyrel     # relative speed of y, VCSFLU车体坐标系, unit: m/s
float64 xabs      # Absolute distance of x, ENU世界坐标系, unit: m
float64 yabs      # Absolute distance of y, ENU世界坐标系 unit: m
float32 vxabs     # Absolute speed of x, ENU世界坐标系 unit: m/s
float32 vyabs     # Absolute speed of y, ENU世界坐标系, unit: m/s
float32 width     # unit: m
float32 length    # unit: m
float32 height     				# unit: m
float32 speed      				# unit: m/s
float32 heading      			        # unit: du
uint8 type 					# CAR_TYPE=0, TRUCK_TYPE=1, PEDESTRIAN_TYPE=2, CYCLIST_TYPE=3, UNKNOWN_TYPE=4, UNKNOWN_MOVABLE_TYPE=5, UNKNOWN_UNMOVABLE_TYPE=6
uint8 source 					# VISION=0x0000 MOBILEYE=0x0001 LIDAR=0x0100 RADAR=0x0200 ARS_0=0x0201   ARS_1=0x0202  ARS_2=0x0203  ARS_3=0x0204
float32 confidence				# 0.0~1.0
uint32 age				# 
uint8 velocitystatus 			# 
ObstacleCell[] cells 

# uint8 track_level
# uint8 invi_count
# uint8 vis_count

# float32 acc_x
# float32 acc_y

