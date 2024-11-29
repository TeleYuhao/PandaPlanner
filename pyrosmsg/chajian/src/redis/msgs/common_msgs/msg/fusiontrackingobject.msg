uint8 objectsource             # 目标来源 1:ldiar,2:radar,3:camera,4:gps,5:OBU,6:cloudplatform,7:fusion,8:tracking
int64 lidartimestamp
int64 lidargpstime
sensorobject lidarObject
int64 radartimestamp
int64 radargpstime
sensorobject radarObject       # 匹配的radar目标
int64 obutimestamp
int64 obugpstime
sensorobject obuObject         # 匹配的obu目标
obupant obupantObject          # 匹配的obu目标
