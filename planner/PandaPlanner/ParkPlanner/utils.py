'''
@Project :park_planning
@Author : YuhaoDu
@Date : 2024/10/13 
'''
import numpy as np
class Config_step_1:
    PI = np.pi
    T = 0.4
    RF = 4.5
    RB = 1.0
    W = 3.0
    WD = 0.7 * W
    WB = 2.9
    LF = 1.45
    LR = WB - LF
    TR = 0.5
    TW = 1
    MAX_STEER = 0.6
    MAX_SPEED = 4
    MIN_SPEED = 0
    MAX_ACC = 5
    MAX_STEERING_RATE = 20