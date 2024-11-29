Header header
#245
float64 ICPV_Cmd_StrAngle                   #方向盘转向角度控制
float64 ICPV_Cmd_StrAngleLimit              #方向盘限幅
float64 ICPV_Cmd_EPSXBW_En                  #转向控制使能
float64 ICPV_Cmd_EPSDir                     #方向盘转动方向
float64 ICPV_Cmd_StrAngleSpd                #方向盘目标角速度
float64 ICPV_Cmd_Checksum                  
float64 ICPV_Cmd_RCnt                
#226
float64 ICPV_Cmd_TgtSft                     #档位控制指令
float64 ICPV_Cmd_AccPelEnable               #加速踏板使能
float64 ICPV_Cmd_BrkPelEnable               #制动踏板使能
float64 ICPV_Cmd_AccPelPosAct               #加速踏板开度
float64 ICPV_Cmd_AccPelPosActInv            #100 - 加速踏板开度
float64 ICPV_Cmd_BrkPelPosAct               #制动踏板开度
float64 ICPV_Cmd_BrkPelPosActInv            #100 - 制动踏板开度
float64 ICPV_Cmd_BrkLightReq                #制动灯控制指令
float64 ICPV_Cmd_TgtSftEnable               #挡位控制使能
float64 ICPV_Cmd_ATOModeEnable              #自动驾驶模式使能
float64 CAN_ICPV_VCU_Control_EPB_REQ        #线控灯光
float64 CAN_ICPV_VCU_BUZZING                #线控声音
#target point and velocity
float64 target_x
float64 target_y
float64 velocity
float64 calc_accel
int16 auto_mode

