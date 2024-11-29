'''
@Project :Piecewise_jerk_minimize
@Author : YuhaoDu
@Date : 2024/9/23 
'''
import time

import casadi as ca
import numpy as np
import matplotlib.pyplot as plt

class PJSO:
    def __init__(self,num_point,t):
        self.num_point = num_point
        self.t = t

        self.make_param()
        self.make_weight()

        self.make_constraint()
    def make_param(self):
        '''
        function:生成优化变量
        :return:
        '''
        self.s   = ca.SX.sym('s',self.num_point)
        self.sd  = ca.SX.sym('sd',self.num_point)
        self.sdd = ca.SX.sym('sdd',self.num_point)

        self.v_max = 35 / 3.6
        self.v_min = 0

        self.a_max = 0.8
        self.a_min = -5

        self.jerk_max = 1
        self.jerk_min = -8

    def make_weight(self):
        '''
        function: 配置权重系数
        :return:
        '''
        self.w_x_ref = 0
        self.w_dx = 1
        self.w_dx_ref = 5
        self.w_ddx = 5
        self.w_dddx = 100

        self.w_x_end = 1
        self.w_dx_end = 1
        self.w_ddx_end = 1

    def Update(self,state,ref_s,ref_v,lower_bound,upper_bound):
        self.ref_s = ref_s
        self.ref_v = ref_v
        self.make_variable(state,lower_bound,upper_bound)
        self.CostFunction()
    def CostFunction(self):
        self.Cost = 0
        for i in range(self.num_point - 1):
            # 参考路径偏移
            self.Cost += self.w_x_ref * (self.s[i] - self.ref_s[i])**2
            # 参考速度偏移
            self.Cost += self.w_dx_ref * (self.sd[i] - self.ref_v)**2
            # 速度控制损失
            self.Cost += self.w_dx * self.sd[i]**2
            # 加速度控制损失
            self.Cost += self.w_ddx * self.sdd[i]**2
            # jerk控制损失
            self.Cost += self.w_dddx * ((self.sdd[i+1] - self.sdd[i])/(self.t))**2
        self.Cost += self.w_x_end * (self.s[-1] - self.ref_s[-1])**2
        self.Cost += self.w_dx_ref * (self.sd[-1] - self.ref_v)**2
        self.Cost += self.w_dx * self.sd[-1] **2
        self.Cost += self.w_ddx *self.sdd[-1] **2

    def make_variable(self,state,lower_bound,upper_bound):
        self.lbx = [state[0],state[1],state[2]]
        self.ubx = [state[0],state[1],state[2]]
        self.variable = [self.s[0],self.sd[0],self.sdd[0]]
        for i in range(1,self.num_point):
            self.variable += self.s[i],self.sd[i],self.sdd[i]
            self.lbx += [lower_bound[i],self.v_min,self.a_min]
            self.ubx += [upper_bound[i],self.v_max,self.a_max]

    def make_constraint(self):
        self.lbg = []
        self.ubg = []
        self.constraint = []
        for i in range(self.num_point-1):
            self.constraint += [self.s[i+1] - self.s[i] - self.sd[i]*self.t - 0.5 * self.sdd[i]*self.t**2 - 1/6 * self.t**2]
            self.constraint += [self.sd[i+1] - self.sd[i] - 1/2*self.t*self.sdd[i] - 1/2*self.t*self.sdd[i+1]]
            self.constraint += [(self.sdd[i+1] - self.sdd[i])/self.t]
            self.lbg += [0,0,self.jerk_min]
            self.ubg += [0,0,self.jerk_max]
    def Solve(self):
        nlp_prob = {'f': self.Cost, 'x': ca.vertcat(*self.variable),
                    'g': ca.vertcat(*self.constraint)}
        solver = ca.qpsol('solver', 'proxqp', nlp_prob)
        init_x = np.zeros(3*self.num_point)
        sol = solver(x0=ca.vertcat(*init_x), lbx=ca.vertcat(*self.lbx), ubx=ca.vertcat(*self.ubx), ubg=self.ubg,
                     lbg=self.lbg)
        res = sol['x']
        self.solution_s = res[::3]
        self.solution_v = res[1::3]
        self.solution_a = res[2::3]

        return self.solution_v


    def VizResult(self):
        self.ref_path_t = np.arange(0.25,5.01,0.25)
        plt.subplot(3, 1, 1)
        plt.plot(self.ref_path_t, [100]*20, 'r', marker="x")
        plt.plot(self.ref_path_t, [0]*20, 'r', marker="x")
        plt.plot(self.ref_path_t, self.ref_s, 'g', marker="x")
        plt.plot(self.ref_path_t, self.solution_s, 'b')
        plt.grid()
        plt.legend(["upper_bound", "lower_bound", "ref_path_s", "solution_s"])
        plt.title("PieceWise Jerk Path optimization")

        plt.subplot(3, 1, 2)
        plt.plot(self.ref_path_t, self.solution_v, 'b')
        plt.plot(self.ref_path_t, [self.v_min]*20, 'r')
        plt.plot(self.ref_path_t, [self.v_max]*20, 'r')
        plt.grid()
        plt.legend(["solution_ds", "ds_bound"])

        plt.subplot(3, 1, 3)
        plt.plot(self.ref_path_t, self.solution_a, 'b')
        plt.plot(self.ref_path_t, [self.a_min]*20, 'r')
        plt.plot(self.ref_path_t, [self.a_max]*20, 'r')
        plt.legend(["solution_dds", "dds_bound"])
        plt.grid()

        # plt.subplot(4, 1, 4)
        # plt.plot(self.ref_path_t, self.solution_dddx, 'b')
        # plt.plot(self.ref_path_t, self.dddx_upper_bound, 'r')
        # plt.plot(self.ref_path_t, self.dddx_lower_bound, 'r')
        # plt.legend(["solution_ddds", "ddds_bound"])
        plt.grid()
        plt.show()


if __name__ == '__main__':
    speed_planner = PJSO(20,0.25)
    ref_v = 10
    state = [0, 0/3.6, 0]
    ref_s = [state[0]+ref_v*t for t in np.arange(0.25,5.01,0.25)]
    lower_bound = [0]*20
    upper_bound = [14]*20
    start_time = time.time()
    speed_planner.Update(state,ref_s,ref_v,lower_bound,upper_bound)
    speed_planner.Solve()
    print(time.time() - start_time)
    speed_planner.VizResult()
