int64 timestamp
uint8 drivemode         # 驾驶模式 0-自动驾驶 1-远程驾驶 2-人工驾驶
uint8 remotedrivestatus # 远程驾驶状态 0-正常 1-warning 2-丢失数据帧数较多强制刹车 3-延迟较大强制刹车 4-上游数据给出异常强制刹车
uint8 systemstatus      # 车辆异常标志位，预留，待定义
