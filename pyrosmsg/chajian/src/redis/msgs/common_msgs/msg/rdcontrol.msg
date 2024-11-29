Header  header
int64   timestamp  # 时间戳，单位ms
uint8   drivemode  # 驾驶模式，0-非远程驾驶，1-远程驾驶
uint8   drivestate # 座舱状态，0-正常，1-异常
float32 angle      # 方向盘转角，正负540，单位度
float32 gas        # 油门开度，0-100%，单位1%
float32 brake      # 刹车踏板开度，0-100%，单位1%
uint8   turnlignt  # 转向灯，0-无效（不亮），1-左转，2-右转，3-双闪
uint8   gear       # 档位，0-无效，1-P，2-R,3-N,4-D
uint8   epb        # 手刹，0-松开，1-拉紧