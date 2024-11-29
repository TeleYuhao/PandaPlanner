import numpy as np
from casadi import *
# from pypoman import compute_polytope_halfspaces
from planner.PandaPlanner.ParkPlanner.utils import Config_step_1 as config
from planner.PandaPlanner.ParkPlanner.reeds_shepp import calc_reeds_shepp
def rotate_point(rotate_angle, points):
    rotation_matrix = np.array([[np.cos(rotate_angle), -np.sin(rotate_angle)],
                                [np.sin(rotate_angle), np.cos(rotate_angle)]])
    rotated = np.dot(points, rotation_matrix)
    return rotated
class OBCAOptimizer:
    def __init__(self):
        # self.L = cfg.length
        self.L = 2.49
        # self.offset = self.L/2 - cfg.baselink_to_rear
        self.offset = self.L/2 - config.RB
        self.lf = 1.245
        self.lr = 1.245
        self.config = config
        self.n_controls = 2
        # self.n_controls = 3
        self.n_states = 5
        '''约束了多边形的个数为4'''
        self.n_dual_variable = 3
        self.n_mu = 4
        self.poly_num = None
        self.constrains = []
        self.lbg = []
        self.ubg = []
        self.lbx = []
        self.ubx = []
        self.variable = []
        self.N = 0
        self.x0 = []
        self.obstacles = []
        '''G和g描述车辆的形状'''
        self.G = DM([[1, 0],
                     [-1, 0],
                     [0, 1],
                     [0, -1], ])
        self.g = vertcat(SX([self.L/2, self.L/2,
                             0.5*2.0, 0.5*2]))
        # self.T = config.T
        self.T = 0.2
        self.park_area = np.array([[1e8, 1e8],
                     [-1e8, -1e8]])
        self.max_x = max(self.park_area[:,0])
        self.min_x = min(self.park_area[:,0])
        self.max_y = max(self.park_area[:,1])
        self.min_y = min(self.park_area[:,1])

        self.r = [[1e1, 0], [0, 1e1]]
        self.q = [[1e1, 0, 0, 0, 0],
                  [0, 1e1, 0, 0, 0],
                  [0, 0, 0, 0, 0],
                  [0, 0, 0, 0., 0],
                  [0, 0, 0, 0, 0],
                  ]

    def initialize(self, start_state,end_state, obs):
    # def initialize(self, init_guess, obs):
        '''
        func : 初始化状态
        :param init_guess:初始猜测解
        :param obs: 障碍物列表
        :param max_x:最大X
        :param max_y: 最大X
        :return:
        '''
        reeds_shepp_path = calc_reeds_shepp(start_state,end_state,maxc=0.1)

        for i in range(len(reeds_shepp_path.yaw)):
            if (reeds_shepp_path.yaw[i] < 0):
                reeds_shepp_path.yaw[i] += 2 * np.pi
        # import matplotlib.pyplot as plt
        # plt.plot(reeds_shepp_path.x,reeds_shepp_path.y)
        # plt.show()
        zero_path = np.zeros(len(reeds_shepp_path.x))
        init_guess = np.vstack((reeds_shepp_path.x, reeds_shepp_path.y, zero_path, reeds_shepp_path.yaw, zero_path)).T
        # self.init_state = SX([init_guess[0].x, init_guess[0].y,
        #                      init_guess[0].v, init_guess[0].heading, init_guess[0].steer])
        self.init_state = SX(init_guess[0] + 1e-8)
        # self.end_state = SX([init_guess[-1].x, init_guess[-1].y,
        #                     init_guess[-1].v, init_guess[-1].heading, init_guess[-1].steer])
        self.end_state = SX(init_guess[-1] + 1e-8)
        # print(init_guess[-1].__dict__)
        self.N = len(init_guess)

        self.obstacles = obs
        # self.poly_num = int(np.array(self.obstacles).flatten().shape[0]/2)
        self.poly_num = sum(len(sub_ob) for sub_ob in self.obstacles)
        for state in init_guess:
            # self.x0 += [[state.x, state.y, state.v,state.heading, state.steer]]
            self.x0 += [state]

        self.x0 += [[0]*(self.n_controls*(self.N-1))]
        # self.x0 += [[0.1]*(self.n_dual_variable*(self.N)*2*len(obs))]
        '''定义描述车辆形状的对偶变量 /mu 的个数'''
        self.x0 += [[0.1]*(self.n_mu*(self.N)*len(obs))]
        # self.x0 += [[0.1]*(self.n_dual_variable*(self.N)*len(obs))]
        '''定义描述障碍物形状的对偶变量 /lambda 的个数，根据总障碍物及边数之和*时间步长确定'''
        self.x0 += [[0.1]*(self.poly_num* self.N)]
        # self.dual_num = sum(len(sublist) for sublist in obs)
        # self.x0 += [[0.1]*((self.N)*2*self.dual_num)]
        self.ref_state = init_guess

        self.max_y = max(init_guess[:,1]) + 0.25
        self.min_y = min(init_guess[:,1]) - 0.25


    def build_model(self) -> bool:
        if self.N < 1:
            print('empty init guess')
            return False
        x = SX.sym('x')
        y = SX.sym('y')
        v = SX.sym('v')
        theta = SX.sym('theta')
        steering = SX.sym('steering')
        a = SX.sym('a')
        t = SX.sym('t')
        steering_rate = SX.sym('steering_rate')
        # state:[x,y,v,\theta, steering]
        self.state = vertcat(x, y, v, theta, steering)
        # a = vertcat(x, y, v, theta, steering)
        # control:[acc, delta_f]
        self.control = vertcat(a, steering_rate)
        # control:[acc, delta_f ,t]
        # self.control = vertcat(a, steering_rate,t)
        # arctan(lr/(l) * tan(delta_f))
        beta = atan(self.lr*tan(steering)/(self.lr+self.lf))
        # beta = atan(self.lr*tan(steering)/(self.lr+self.lf))*t
        # v*cos(theta + beta) v*sin(theta+beta) a  v/lr*sin(beta) steering rate
        # self.rhs = vertcat(vertcat(v*cos(theta+beta)*t, v*sin(theta+beta)*t,
        #                            a*t, v/self.lr*sin(beta))*t, steering_rate*t)
        self.rhs = vertcat(vertcat(v*cos(theta+beta), v*sin(theta+beta),
                                   a, v/self.lr*sin(beta)), steering_rate)
        self.f = Function('f', [self.state, self.control], [self.rhs])
        self.X = SX.sym('X', self.n_states, self.N)
        self.U = SX.sym('U', self.n_controls, self.N-1)
        # self.MU = SX.sym('MU', self.n_dual_variable,
        #                  self.N*len(self.obstacles))
        '''车辆的对偶变量'''
        self.MU = SX.sym('MU', self.n_mu,
                         self.N*len(self.obstacles))
        '''obstacle 对偶变量'''
        # self.LAMBDA = SX.sym('LAMBDA', self.n_dual_variable,
        #                      self.N*len(self.obstacles))
        self.LAMBDA = SX.sym('LAMBDA',self.poly_num,self.N)
        self.obj = 0

        return True

    def solve(self):
        self.build_model()
        self.generate_variable()
        self.generate_constrain()
        self.generate_object()
        # 构造非线性问题
        nlp_prob = {'f': self.obj, 'x': vertcat(*self.variable),
                    'g': vertcat(*self.constrains)}
        # 构造求解器 选择求解其为ipopt
        solver = nlpsol('solver', 'ipopt', nlp_prob)
        # 传入变量和约束，进行求解
        sol = solver(x0=vertcat(*self.x0), lbx=self.lbx, ubx=self.ubx,
                     ubg=self.ubg, lbg=self.lbg)
        u_opt = sol['x']
        self.x_opt = u_opt[0:self.n_states*(self.N):self.n_states]
        self.y_opt = u_opt[1:self.n_states*(self.N):self.n_states]
        self.v_opt = u_opt[2:self.n_states*(self.N):self.n_states]
        self.theta_opt = u_opt[3:self.n_states*(self.N):self.n_states]
        self.steer_opt = u_opt[4:self.n_states*(self.N):self.n_states]
        self.a_opt = u_opt[self.n_states*(self.N):self.n_states*(
            self.N)+self.n_controls*(self.N-1):self.n_controls]
        self.steerate_opt = u_opt[self.n_states*(self.N)+1:self.n_states*(
            self.N)+self.n_controls*(self.N-1):self.n_controls]

        self.x_opt_array = np.array(self.x_opt).ravel()
        self.y_opt_array = np.array(self.y_opt).ravel()
        self.theta_opt_array = np.array(self.theta_opt).ravel()
        # self.t_opt = u_opt[self.n_states*(self.N)+2:self.n_states*(
        #     self.N)+self.n_controls*(self.N-1):self.n_controls]
        # print(self.t_opt)
        totalPath = np.vstack([self.x_opt_array, self.y_opt_array, self.theta_opt_array]).T
        length = sum(np.linalg.norm(np.diff(totalPath,axis=0),axis = 1))
        return  totalPath,length
    def generate_object(self):
        '''
        func: 计算cost func
        :param r: matrix r
        :param q: matrix q
        :return:
        '''
        R = SX(self.r)
        Q = SX(self.q)
        for i in range(self.N-1):
            st = self.X[:, i]
            ref_st = self.x0[i]
            error = st - ref_st
            con = self.U[:, i]
            #cost func  control^T * R * control + (state - ref_state)*Q*(state - ref_state)
            self.obj += (con.T@R@con)
            self.obj += (error.T@Q@error)

            # st_next = self.X[:, i + 1]
            # self.obj += ((st[0] - st_next[0]) ** 2 + (st[1] - st_next[1]) ** 2)
            # smooth_term = self.X[:, i - 1] + self.X[:, i + 1] - 2 * st
            # self.obj += smooth_term.T @ smooth_term * 1

            '''TDR-OBCA'''
            # obs_index = 0
            # for obstacle in self.obstacles:
            #     # 将障碍物转换为凸包表示
            #     A, b = compute_polytope_halfspaces(obstacle)
            #
            #     lamb = vertcat(self.LAMBDA[:, i])[obs_index:obs_index + len(obstacle)]
            #     self.obj += dot(A.T @ lamb, A.T @ lamb)
            #     obs_index += len(obstacle)

    def generate_variable(self):
        '''
        func:生成变量并添加约束条件
        :return:
        '''
        for i in range(self.N):
            # 一阶控制约束
            self.variable += [self.X[:, i]]
            # todo change the upper bound and lower bound
            self.lbx += [self.min_x, self.min_y, -self.config.MAX_SPEED, -2 * pi, -self.config.MAX_STEER]
            self.ubx += [self.max_x, self.max_y,  self.config.MAX_SPEED,  2 * pi,  self.config.MAX_STEER]

        for i in range(self.N-1):
            # 二阶控制约束
            self.variable += [self.U[:, i]]
            # self.lbx += [-self.v_cfg.max_acc, -self.v_cfg.max_steer_rate , 0.1]
            self.lbx += [-self.config.MAX_ACC, -self.config.MAX_STEERING_RATE ]
            # self.ubx += [self.v_cfg.max_acc, self.v_cfg.max_steer_rate, 0.2]
            self.ubx += [self.config.MAX_ACC, self.config.MAX_STEERING_RATE]
        j = 0
        for i in range(self.N):
            ob_index = 0
            for ob in self.obstacles:
                self.variable += [self.MU[:, j]]
                self.lbx += [0.0] * (self.MU[:, j].shape[0])
                self.ubx += [100000] * (self.MU[:, j].shape[0])
                self.variable += [self.LAMBDA[:, i][ob_index:ob_index + len(ob)]]
                self.lbx += [0.0] * (self.LAMBDA[:, i][ob_index:ob_index + len(ob)].shape[0])
                self.ubx += [100000.0] * (self.LAMBDA[:, i][ob_index:ob_index + len(ob)].shape[0])
                ob_index += len(ob)
                j += 1
        print('success')

    def generate_constrain(self):
        '''
        func:增加碰撞约束
        :return:
        '''
        # 增加初始位置约束
        self.constrains += [self.X[:, 0]-self.init_state]
        self.lbg += [0, 0, 0, 0, 0]
        self.ubg += [0, 0, 0, 0, 0]
        for i in range(self.N-1):
            st = self.X[:, i]
            con = self.U[:, i]
            f_value = self.f(st, con)
            st_next_euler = st+self.T*f_value
            # st_next_euler = st+f_value*con[2]
            st_next = self.X[:, i+1]
            # 增加运动可行约束
            self.constrains += [st_next-st_next_euler]
            self.lbg += [0, 0, 0, 0, 0]
            self.ubg += [0, 0, 0, 0, 0]
        self.constrains += [self.X[:, -1]-self.end_state]
        self.lbg += [0, 0, 0, 0, 0]
        self.ubg += [0, 0, 0, 0, 0]

        # for i in range(self.N):
        #     index = 0
        #     obs_index = 0
        #     for obstacle in self.obstacles:
        #         # 将障碍物转换为凸包表示
        #         A, b = compute_polytope_halfspaces(obstacle)
        #         st = self.X[:, i]
        #         heading = st[3]
        #         x = st[0]
        #         y = st[1]
        #         t = vertcat(x+self.offset*cos(heading),
        #                     y+self.offset*sin(heading))
        #         r = np.array([[cos(heading), -sin(heading)],
        #                       [sin(heading), cos(heading)]])
        #         # lamb = vertcat(self.LAMBDA[:, len(self.obstacles)*i+index])
        #         lamb = vertcat(self.LAMBDA[:,i])[obs_index:obs_index + len(obstacle)]
        #         obs_index += len(obstacle)
        #         mu = vertcat(self.MU[:, len(self.obstacles)*i+index])
        #         index += 1
        #         # compute 0< \left \| A^T\lambda  \right \| <1
        #         self.constrains += [dot(A.T@lamb, A.T@lamb)]
        #         self.lbg += [0]
        #         self.ubg += [1]
        #         # compute 0= G^T \mu  + R^T A^T \lambda =0
        #         self.constrains += [self.G.T@mu+(r.T@A.T)@lamb]
        #         self.lbg += [0, 0]
        #         self.ubg += [0, 0]
        #         # compute -g^T\mu + (At-b)\lambda >0
        #         self.constrains += [(-dot(self.g, mu)+dot(A@t-b, lamb))]
        #         self.lbg += [0.1]
        #         # self.lbg += [0.02]
        #         self.ubg += [100000]
