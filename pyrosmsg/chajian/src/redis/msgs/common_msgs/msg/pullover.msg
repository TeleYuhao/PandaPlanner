int64 timestamp      # 当前时间戳 ms 级
uint8 POdecision        # 0:暂未入站    1:入站成功   2：接收成功（弹出按钮）   
uint8 PObutton          # 0:暂不启动    1:可以出站（按钮消失，车辆起步）
uint8 doorstatus      #车门状态   0：关门    1：开门
int32 reserve         #预留
