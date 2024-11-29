'''
@Project :Piecewise_jerk_minimize
@Author : YuhaoDu
@Date : 2024/9/25 
'''
import numpy as np
import casadi as ca
import matplotlib.pyplot as plt
import math

class PJPO:
    def __init__(self,num_point,ds):
        self.num_point = num_point # 路径点数量
        self.ds = ds # 路段间隔

        self.make_param()
        self.make_weight()
        self.make_constraint()
    def make_weight(self):
        '''
        function: 配置权重系数
        :return:
        '''
        self.wdl = 10
        self.wddl = 100
        self.wdddl = 100

        self.w_ref = 2

    def make_param(self):
        '''
        function:生成优化变量
        :return:
        '''
        self.s   = ca.SX.sym('s',self.num_point)
        self.sd  = ca.SX.sym('sd',self.num_point)
        self.sdd = ca.SX.sym('sdd',self.num_point)

        # self.dl_bound = math.tan(np.deg2rad(30))
        # self.ddl_bound = (math.tan(self.delta_max)/self.wheel_base - 0.0)
        # self.dddl_bound =  self.max_delta_rate / self.wheel_base / 2.0
        self.dl_bound   = 1.2
        self.ddl_bound  = 2
        self.dddl_bound =  2

    def make_variable(self,state,lower_bound,upper_bound):
        self.lbx = [state[0],state[1],state[2]]
        self.ubx = [state[0],state[1],state[2]]
        self.lb = lower_bound
        self.ub = upper_bound
        self.variable = [self.s[0],self.sd[0],self.sdd[0]]
        for i in range(1,self.num_point):
            self.variable += self.s[i],self.sd[i],self.sdd[i]
            self.lbx += [lower_bound[i], -self.dl_bound, -self.ddl_bound]
            self.ubx += [upper_bound[i],  self.dl_bound,  self.ddl_bound]
    def make_constraint(self):
        self.lbg = []
        self.ubg = []
        self.constraint = []
        for i in range(self.num_point-1):
            self.constraint += [self.s[i+1] - self.s[i] - self.sd[i]*self.ds - 1/2 * self.sdd[i]*self.ds**2 - 1/6 * self.sdd[i] * self.ds**2]
            self.constraint += [self.sd[i+1] - self.sd[i] - 1/2*self.ds*self.sdd[i] - 1/2*self.ds*self.sdd[i+1]]
            self.constraint += [self.sdd[i+1] - self.sdd[i]]
            self.lbg += [0,0,-self.dddl_bound]
            self.ubg += [0,0,self.dddl_bound]

    def Update(self,state,ref_s,lower_bound,upper_bound):
        self.ref_s = ref_s
        self.make_variable(state,lower_bound,upper_bound)
        self.CostFunction()

    def CostFunction(self):
        self.Cost = 0
        for i in range(self.num_point - 1):
            # from 0 to n-1
            # 参考路径偏移

            self.Cost += self.w_ref * (self.s[i] - self.ref_s[i]) ** 2
            # 速度控制损失
            self.Cost += self.wdl * self.sd[i]**2
            # 加速度控制损失
            self.Cost += self.wddl * self.sdd[i]**2
            # jerk控制损失
            self.Cost += self.wddl * ((self.sdd[i+1] - self.sdd[i])/(self.ds**2))**2
        # calculate n step cost
        self.Cost += self.w_ref * 100 * (self.s[-1] - self.ref_s[-1]) ** 2
        self.Cost += self.wdl * self.sd[-1] ** 2
        self.Cost += self.wddl * self.sdd[-1] ** 2

    def Solve(self):
        nlp_prob = {'f': self.Cost, 'x': ca.vertcat(*self.variable),
                    'g': ca.vertcat(*self.constraint)}
        # nlp_prob.addOption('print_level', 0)
        # opts_setting = {
        #     'ipopt.max_iter': 20,  # 最大迭代次数
        #     'ipopt.print_level': 5,  # 用于优化计算的日志输出详细级别，数字越高，越详细，0即为不输出相关信息
        #     'print_time': 1,  # 不输出最后的求解时间
        #     'ipopt.acceptable_tol': 1e-5,
        #     'ipopt.acceptable_obj_change_tol': 1e-5
        # }
        # solver = ca.nlpsol('solver', 'ipopt', nlp_prob,opts_setting)
        # solver = ca.nlpsol('solver', 'ipopt', nlp_prob)
        solver = ca.qpsol('solver', 'proxqp', nlp_prob)
        # solver = ca.qpsol('solver', 'qrqp', nlp_prob)
        # solver = ca.qpsol('solver', 'osqp', nlp_prob)
        init_x = np.zeros(3*self.num_point)
        # opts = {'ipopt.print_level': 0}
        sol = solver(x0=ca.vertcat(*init_x), lbx=ca.vertcat(*self.lbx), ubx=ca.vertcat(*self.ubx), ubg=self.ubg,
                     lbg=self.lbg)
        res = sol['x']
        self.solution_s = res[::3]
        self.solution_v = res[1::3]
        self.solution_a = res[2::3]

        return self.solution_s.full().flatten()


    def VizResult(self):
        self.ref_path_t = np.arange(0,50,1)
        plt.subplot(3, 1, 1)
        plt.plot(self.ref_path_t, self.lb, 'r', marker="x")
        plt.plot(self.ref_path_t, self.ub, 'r', marker="x")
        plt.plot(self.ref_path_t, self.ref_s, 'g', marker="x")
        plt.plot(self.ref_path_t, self.solution_s, 'b')
        plt.grid()
        plt.legend(["upper_bound", "lower_bound", "ref_path_s", "solution_s"])
        plt.title("PieceWise Jerk Path optimization")

        plt.subplot(3, 1, 2)
        plt.plot(self.ref_path_t, self.solution_v, 'b')
        plt.plot(self.ref_path_t, [-self.dl_bound]*self.num_point, 'r')
        plt.plot(self.ref_path_t, [self.dl_bound]*self.num_point, 'r')
        plt.grid()
        plt.legend(["solution_ds", "ds_bound"])

        plt.subplot(3, 1, 3)
        plt.plot(self.ref_path_t, self.solution_a, 'b')
        plt.plot(self.ref_path_t, [-self.ddl_bound]*self.num_point, 'r')
        plt.plot(self.ref_path_t, [self.ddl_bound]*self.num_point, 'r')
        plt.legend(["solution_dds", "dds_bound"])
        plt.grid()

        # plt.subplot(4, 1, 4)
        # plt.plot(self.ref_path_t, self.solution_dddx, 'b')
        # plt.plot(self.ref_path_t, self.dddx_upper_bound, 'r')
        # plt.plot(self.ref_path_t, self.dddx_lower_bound, 'r')
        # plt.legend(["solution_ddds", "ddds_bound"])
        plt.grid()
        plt.show()


def GetRefS(state, offset):
    # gap = 0.3
    ref_s = np.ones(50) * offset
    num = max(int(state[-1] * 3.6),1)
    n = max(int(state[-1]) + 2, 7)
    gap = abs(offset - state[0]) / num
    for i in range(n): ref_s[i] = state[0]
    for i in range(n, num + n):
        ref_s[i] = state[0] + (i - n) * gap if offset > 0 else state[0] - (i - n) * gap
    return ref_s

if __name__ == '__main__':
    import time
    path_planner = PJPO(50,1)
    path_planner.w_ref = 2
    state = [0, 0, 0,24/3.6]
    # ref_s = [4 for t in np.arange(0,50,1)]
    ref_s = GetRefS(state,4)
    lower_bound = [-5]*50
    upper_bound = [5]*50

    # for i in range(20,50): lower_bound[i] = 2
    # for i in range(35,45): upper_bound[i] = 0

    path_planner.Update(state,ref_s,lower_bound,upper_bound)
    start_time = time.time()
    path_planner.Solve()
    print("calculate time",time.time() - start_time)
    path_planner.VizResult()