from collections import deque


print_flag = True
class Longitudinal_PID_controller(object):
    """
    PID 控制
    包括比例项， 积分项，微分项
    只有比例项会产生稳态误差，（稳态误差就是控制最终稳定在一个值但是和目标值有一定的差距）
    引入积分项可以消除稳态误差，但是会引起超调、震荡问题和积分饱和问题
    采用积分分离来克服系统超调和震荡
    """
    def __init__(self,  K_P=0.4, K_I=0.01, K_D=0, dt=0.1):
        """
        采用PID进行纵向控制
        :param ego_vehicle: 控制的车辆， 类型是carla.Vehicle
        :param K_P: 比例项系数
        :param K_I: 积分项系数
        :param K_D: 微分项系数
        :param dt: 控制间隔
        """
        self.K_P = K_P
        self.K_I = K_I
        self.K_D = K_D
        self.dt = dt
        self.target_speed = None
        self.error_buffer = deque(maxlen=60)  # 设置一个误差缓存区，用于积分项和差分项的计算
        self.error_threshold = 1  # 设定一个阈值，进行积分分离，标量单位是km/h,
        # 由于carla的最大throttle是1，因此误差如果大于1就让采取积分分离

    def PID_fun(self):
        """

        :return:
        """

        cur_speed = self.VehState.v

        error = self.target_speed - cur_speed  # 当前误差
        self.error_buffer.append(error)  # 将新的误差放入缓存区，如果缓存区满了，最左边的溢出，整体数据左移一位，新的数据加在最右边

        if len(self.error_buffer) >= 2:
            # 积分误差，为了解决稳态误差引入的积分项
            integral_error = sum(self.error_buffer) * self.dt
            # 微分误差，为了缓解超调
            differential_error = (self.error_buffer[-1] - self.error_buffer[-2]) / self.dt
        else:
            integral_error = 0.0
            differential_error = 0.0

        # 积分分离，当误差较大时，采取积分分离防止超调
        # if print_flag:
        #     print("absolute speed error:", abs(error))

        if abs(error) > self.error_threshold:
            # 一旦出现误差大于阈值的情况，积分分离让积分项为0，清除误差缓存区，此时只有比例项发挥作用
            integral_error = 0.0
            self.error_buffer.clear()

        return self.K_P * error + self.K_I * integral_error + self.K_D * differential_error

    def PID_control(self, VehState,target_speed):

        self.target_speed = target_speed
        self.VehState = VehState
        return self.PID_fun()