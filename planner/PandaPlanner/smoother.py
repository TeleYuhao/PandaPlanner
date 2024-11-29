
import casadi as ca
import matplotlib.pyplot as plt
import numpy as np


class path_smoother:
    def __init__(self):
        '''
        func: smooth the raw path
        :param ref_p:the reference path point
        '''
        # self.ref_p = ref_p
    def make_variable(self):
        self.N = len(self.ref_p)
        self.N_slack = self.N - 2
        self.N_variable = self.N + self.N_slack
        self.lbx = []
        self.ubx = []
        self.variable = []
        self.constrains = []
        self.lbg = []
        self.ubg = []

        self.lb = 1
        self.ub = 1
        self.x0 = []

        self.ave_length = 0
        self.cur_limit = 0.15
    def update_traj(self,ref_p):
        self.ref_p = ref_p
    # def initialize(self):
    def build_model(self):
        '''
        func:build the nolinear model and construct the variable
        :return:
        '''
        self.x = ca.SX.sym('x',2,self.N)
        self.s = ca.SX.sym('slack',self.N_slack)
        self.obj = 0

        diff_p = np.diff(self.ref_p,axis=0)
        self.ave_length = np.sum(np.sqrt(diff_p[:,0]**2 + diff_p[:,1] **2))/(self.N-1)


    def generate_obj(self):
        '''
        func: generate the objective func of path smoother
        :return:
        '''
        for i in range(1,self.N-1):
            x_0 = self.x[:,i-1]
            x_1 = self.x[:,i]
            x_2 = self.x[:,i+1]
            # latex term
            # cost += \sum_{i = 1} ^ {n - 1}((x_{i-1} + x_{i+1} - 2 * x_i) ^ 2 + (y_{i-1} + y_{i+1} - 2 * y_i) ^ 2)
            smooth_term = x_0 + x_2 - 2*x_1
            self.obj += smooth_term.T @ smooth_term * 1000
            # self.obj += (
            #             (x_0[0] + x_1[0] - 2*x_2[0])**2 +
            #              (x_0[1] + x_1[1] - 2*x_2[1])**2
            # )

            # cost+= \sum_{i=1}^{n-1} ((x_i - x_{i+1})^2 + (y_i-y_{i+1})^2)
            length_term = x_1-x_2
            self.obj += length_term.T @ length_term*0
            # cost+= \sum_{i=1}^{n-1} ((x_i - x_{ref})^2 + (y_i-y_{ref})^2)
            ref_term = x_1 - self.ref_p[i]
            self.obj += ref_term.T@ref_term

            self.obj += self.s[i-1]* 1000
    def generate_variable(self):
        '''
        func: generate the raw variable to be optimized
        :return:
        '''
        self.variable += [self.x[:, 0]]
        self.lbx += [self.ref_p[0].tolist()]
        self.ubx += [self.ref_p[0].tolist()]
        self.x0  += [self.ref_p[0].tolist()]
        self.variable += [self.x[:, 1]]
        self.lbx += [self.ref_p[1].tolist()]
        self.ubx += [self.ref_p[1].tolist()]
        self.x0  += [self.ref_p[1].tolist()]
        for i in range(2,self.N-2):
            self.variable += [self.x[:,i]]
            self.lbx += [(self.ref_p[i] - self.lb).tolist()]
            self.ubx += [(self.ref_p[i] + self.ub).tolist()]
            self.x0 += [self.ref_p[i].tolist()]

        self.variable += [self.x[:, self.N - 2]]
        self.lbx += [self.ref_p[self.N - 2].tolist()]
        self.ubx += [self.ref_p[self.N - 2].tolist()]
        self.x0 += [self.ref_p[self.N - 2]]

        self.variable += [self.x[:, self.N-1]]
        self.lbx += [self.ref_p[self.N-1].tolist()]
        self.ubx += [self.ref_p[self.N-1].tolist()]
        self.x0 += [self.ref_p[self.N-1]]
        for i in range(1, self.N - 1):
            self.variable += [self.s[i - 1]]
            self.lbx += [0]
            self.ubx += [1e5]
            self.x0 += [0.0]

    def generate_constraint(self):
        '''
        func: generative the constraint of smooth term
        :return:
        todo: replace the nonlinear curvature constraint with linear constraint
        '''
        for i in range(1, self.N - 1):
            x_0 = self.x[:, i - 1]
            x_1 = self.x[:, i]
            x_2 = self.x[:, i + 1]
            # latex function
            # cost += \sum_{i = 1} ^ {n - 1}((x_{i-1} + x_{i+1} - 2 * x_i) ^ 2 + (y_{i-1} + y_{i+1} - 2 * y_i) ^ 2)
            smooth_term = x_0 + x_2 - 2 * x_1
            self.constrains +=[smooth_term[0]**2 + smooth_term[1]**2 - self.s[i-1]]
            self.lbg += [-1e10]
            self.ubg += [(self.ave_length**2 * self.cur_limit)**2]
    def solve(self,ref_p):
        '''
        solve the nonlinear problem
        :return:
        '''
        self.ref_p = ref_p
        self.make_variable()
        self.build_model()
        self.generate_variable()
        self.generate_constraint()
        self.generate_obj()
        nlp_prob = {'f': self.obj, 'x': ca.vertcat(*self.variable),
                    'g': ca.vertcat(*self.constrains)}
        # 构造求解器 选择求解其为ipopt
        solver = ca.nlpsol('solver', 'ipopt', nlp_prob)

        sol = solver(x0=ca.vertcat(*self.x0), lbx=ca.vertcat(*self.lbx), ubx=ca.vertcat(*self.ubx),ubg=self.ubg, lbg=self.lbg)

        res = sol['x']
        optimized = res[:2*self.N].toarray().reshape(self.N,2)

        return optimized


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    points_x = [1.0, 1.5, 2.1, 2.1, 2.1, 2.1, 2.1, 2.1, 2.5, 2.9, 3.3, 3.8]
    points_y = [0.0, 0.3, 0.5, 0.9, 1.2, 1.5, 1.9, 2.2, 2.2, 2.2, 2.2, 2.2]
    points = np.vstack([points_x,points_y]).T

    points = np.load("test.npy").T

    smoother = path_smoother()
    optimized_points = np.array(smoother.solve(points[:,:2]))
    plt.plot(points[:,0],points[:,1],label='raw')
    plt.plot(optimized_points[:,0],optimized_points[:,1],label='optimized')
    plt.legend()
    plt.show()
    # print(smoother.ave_length)

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
