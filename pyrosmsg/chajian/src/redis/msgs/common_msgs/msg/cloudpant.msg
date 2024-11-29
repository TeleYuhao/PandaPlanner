string  Id                  # 参与者ID，唯一值；
int32   vehicletype         # 分车辆类型:0-未知；1-小客车；2-大货车；3-大巴车；4-行人；5-自行车；6-摩托车；7-中巴车；8-危化车；9-遗撒物；10-小货车;11-中货车
float32 length              # 长度 单位m
float32 width               # 宽度 单位m
float32 height              # 高度 单位m
float64 longitude           # 分辨率1e-7°，东经为正，西经为负
float64 latitude            # 分辨率1e-7°，北纬为正，南纬为负
int32   locationConfidence  # 位置置信度，单位：%
float32 speed               # 速度，单位：m/s
float32 courseAngle         # 航向角，单位：rad，保留1位小数，车头与正北夹角
int32   sportConfidence     # 运动参数置信度，单位：%