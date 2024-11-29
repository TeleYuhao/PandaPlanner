import casadi as ca
import matplotlib.pyplot as plt
import numpy as np
from planner.PandaPlanner.ParkPlanner.utils import Config_step_1 as C

from planner.PandaPlanner.ParkPlanner.reeds_shepp import calc_reeds_shepp

def get_front_position(x,y,yaw):
    x_fr = x +3.5* np.cos(yaw)
    y_fr = y +3.5* np.sin(yaw)
    return np.array([x_fr,y_fr]).T
class ParkPlanner:
    def __init__(self, start_state,end_state):
        '''
        func: smooth the raw path
        :param ref_p:the reference path point
        '''
        # 参考轨迹

        # self.fr         = get_front_position(ref_p[:,0],ref_p[:,1],ref_p[:,3])
        init_path = calc_reeds_shepp(start_state,end_state,maxc=0.2)
        zero_path = np.zeros(len(init_path.x))
        self.ref_p      = np.vstack((init_path.x,init_path.y,zero_path,init_path.yaw,zero_path)).T
        self.N          = len(self.ref_p)
        # 共有5个状态量
        self.n_states   = 5
        # 变量约束
        self.lbx        = []
        self.ubx        = []
        # 变量
        self.variable   = []
        # 约束
        self.constrains = []
        # 约束条件上下限
        self.lbg        = []
        self.ubg        = []
        # 初始解
        self.x0         = []
        self.start_pose = self.ref_p[0]
        self.end_pose   = self.ref_p[-1]
        # 固定时间窗 控制量  [acc, delta_f]
        self.n_controls = 2
        # 可变时间窗 控制量修改为[acc, delta_f, t]
        # self.n_controls = 3
        self.res        = None
        self.corridor_rear  = None
        self.corridor_front = None

        self.r = [[1e0, 0], [0, 1e1]]
        self.q = [[0, 0, 0, 0, 0],
                 [0, 0, 0, 0, 0],
                 [0, 0, 0, 0, 0],
                 [0, 0, 0, 0., 0],
                 [0, 0, 0, 0, 0],
                 ]

        self.x_hist = []
        self.y_hist = []

    def initialize(self):

        self.x0 = []
        self.lbx = []
        self.ubx = []
        self.lbg = []
        self.ubg = []
        self.variable = []
        self.constrains = []
        self.corridor_min_front = []
        self.corridor_min_rear = []
        self.corridor_max_front = []
        self.corridor_max_rear = []
        # self.fr = get_front_position(self.ref_p[:, 0], self.ref_p[:, 1], self.ref_p[:, 3])

        self.init_state = ca.SX([   self.ref_p[0][0],
                                    self.ref_p[0][1],
                                    self.ref_p[0][2],
                                    self.ref_p[0][3],
                                    self.ref_p[0][4]])
        # 终止状态约束
        self.end_state = ca.SX([self.ref_p[-1][0],
                                self.ref_p[-1][1],
                                self.ref_p[-1][2],
                                self.ref_p[-1][3],
                                self.ref_p[-1][4]])
        for i in range(0,len(self.ref_p)):
            # x , y  , v , yaw, a ,steer
            self.x0 += [self.ref_p[i][0] ,self.ref_p[i][1], self.ref_p[i][2], self.ref_p[i][3],self.ref_p[i][4]]
        # 增加控制量
        self.x0 += [[0]*(self.n_controls*(self.N-1))]

    def build_model(self):
        '''
        func:build the nolinear model and construct the variable
        :return:
        '''
        # 构造符号变量
        x       = ca.SX.sym('x')
        y       = ca.SX.sym('y')
        theta   = ca.SX.sym('theta')
        v       = ca.SX.sym('v')
        steering= ca.SX.sym('steering')
        a       = ca.SX.sym('a')
        # 步长t
        steering_rate = ca.SX.sym('steering_rate')
        self.obj = 0

        # state:[x,y,v,\theta, steering]
        self.state      = ca.vertcat(x, y, v, theta, steering)
        # control:[acc, delta_f]
        self.control    = ca.vertcat(a, steering_rate)
        # spcified :control:[acc , delta_f, t]
        beta            = ca.atan( C.LR* ca.tan(steering) / (C.LR + C.LF))
        # 运动学模型
        # v*cos(theta + beta) v*sin(theta+beta) a  v/lr*sin(beta) steering rate
        self.rhs        = ca.vertcat(ca.vertcat(v * ca.cos(theta + beta), v * ca.sin(theta + beta),
                                   a, v / C.LR * ca.sin(beta)), steering_rate)
        # 构建f = AX+BU
        self.f = ca.Function('f', [self.state, self.control], [self.rhs])
        # 构建状态量集合 N个时刻 每个时刻5个状态量
        self.X = ca.SX.sym('X', self.n_states, self.N)
        # 构建控制量集合 N-1个间隔， 每个时刻2个控制量
        self.U = ca.SX.sym('U', self.n_controls, self.N - 1)


    def generate_obj(self):
        '''
        func: generate the objective func of path smoother
        :return:
        '''
        R = ca.SX(self.r)
        Q = ca.SX(self.q)
        for i in range(1,self.N-1):
            # state x,y,yaw,v,steering
            st = self.X[:, i]

            # ref_state
            ref_st = self.x0[i]
            error = st - ref_st
            # control a , delta_f , t
            con = self.U[:, i]
            #cost func  control^T * R * control + (state - ref_state)*Q*(state - ref_state)
            self.obj += (con.T@R@con)
            # self.obj += st[2]*st[4]*1e0
            # self.obj += con[2] * 1e8
            self.obj += (error.T@Q@error)
            # self.obj += t
    #         todo add smooth cost term
    #         st_next = self.X[:, i + 1]
            # self.obj+= ((st[0] - st_next[0])**2 + (st[1] - st_next[1])**2)*1e0
            # smooth_term = self.X[:, i-1] + self.X[:, i+1] - 2*st
            # self.obj += smooth_term.T @ smooth_term * 1e0 *2
            self.vio = 0
            self.vio_coe = 1e4
            for i in range(0, self.N - 1):
                st = self.X[:, i]
                st_ = self.X[:, i + 1]
                con = self.U[:, i]
                f_value = self.f(st, con[:2])*C.T
                # constrain_12 = con[2] * f_value
                # kinematic constrain
                # self.vio += ((st_ - st) - constrain_12).T @ ((st_ - st) - constrain_12) * self.vio_coe
                self.vio += ((st_ - st) - f_value).T @ ((st_ - st) - f_value) * self.vio_coe
            self.obj += self.vio



    def generate_variable(self):
        '''
        func: generate the raw variable to be optimized
        :return:
        '''
        for i in range(self.N):
            # 一阶控制约束
            self.variable += [self.X[:, i]]
            self.lbx += [self.corridor_min_rear[i][0], self.corridor_min_rear[i][1], - C.MAX_SPEED,  -40 * np.pi , -C.MAX_STEER]
            self.ubx += [self.corridor_max_rear[i][0], self.corridor_max_rear[i][1],   C.MAX_SPEED,   40 * np.pi ,  C.MAX_STEER]

        for i in range(self.N - 1):
            # 二阶控制约束
            self.variable += [self.U[:, i]]
            self.lbx += [-C.MAX_ACC, -C.MAX_STEERING_RATE ]
            self.ubx += [ C.MAX_ACC,  C.MAX_STEERING_RATE ]

    def generate_constraint(self):
        '''
        func: generative the constraint of smooth term
        :return:
        todo: replace the nonlinear curvature constraint with linear constraint
        '''
        # 增加起始位置约束
        self.constrains += [self.X[:, 0] - self.ref_p[0][:5]]
        self.lbg += [0, 0, 0, 0, 0]
        self.ubg += [0, 0, 0, 0, 0]

        # 增加结束状态约束
        self.constrains += [self.X[:, -1] - self.ref_p[-1][:5]]
        self.lbg += [0, 0, 0, 0, 0]
        self.ubg += [0, 0, 0, 0, 0]
    def get_corridor_constrain(self):
        '''
        func:生成通行走廊约束
        '''
        self.corridor_min_rear = []
        self.corridor_max_rear = []
        for corridor in self.corridor_rear:
            self.corridor_min_rear.append([min(corridor[:,0]) , min(corridor[:,1])])
            self.corridor_max_rear.append([max(corridor[:,0]) , max(corridor[:,1])])

        # self.corridor_min_front = []
        # self.corridor_max_front = []
        # for corridor in self.corridor_front:
        #     self.corridor_min_front.append([min(corridor[:,0]) , min(corridor[:,1])])
        #     self.corridor_max_front.append([max(corridor[:,0]) , max(corridor[:,1])])
    def solve(self):
        '''
        solve the nonlinear problem
        :return:
        '''
        self.initialize()
        self.corridor_rear  = self.make_corridor(self.ref_p[:,:2])
        self.get_corridor_constrain()
        self.build_model()
        self.generate_variable()
        self.generate_constraint()
        self.generate_obj()
        # plt.plot(self.ref_p[:, 0], self.ref_p[:, 1])
        # plt.show()
        nlp_prob = {'f': self.obj, 'x': ca.vertcat(*self.variable),
                    'g': ca.vertcat(*self.constrains)}
        # 构造求解器 选择求解其为ipopt
        solver = ca.nlpsol('solver', 'ipopt', nlp_prob)
        # solver = ca.qpsol('solver', 'proxqp', nlp_prob)

        sol = solver(x0=ca.vertcat(*self.x0), lbx=ca.vertcat(*self.lbx), ubx=ca.vertcat(*self.ubx),ubg=self.ubg, lbg=self.lbg)

        res = sol['x']
        self.x_opt = res[0:self.n_states * (self.N):self.n_states]
        self.y_opt = res[1:self.n_states * (self.N):self.n_states]
        self.v_opt = res[2:self.n_states * (self.N):self.n_states]
        self.theta_opt = res[3:self.n_states * (self.N):self.n_states]
        self.steer_opt = res[4:self.n_states * (self.N):self.n_states]
        self.a_opt =        res[self.n_states * (self.N):self.n_states * (self.N) + self.n_controls * (self.N - 1):self.n_controls]
        self.steerate_opt = res[self.n_states * (self.N) + 1:self.n_states * (self.N) + self.n_controls * (self.N - 1):self.n_controls]
        self.t = res[self.n_states * (self.N) + 2:self.n_states * (self.N) + self.n_controls * (self.N - 1):self.n_controls]
        # print(self.t)

        self.x_hist.append(self.x_opt.toarray())
        self.y_hist.append(self.y_opt.toarray())

        return np.hstack((self.x_opt.toarray(),self.y_opt.toarray()))

    def make_corridor(self,path):
        '''
        func:构建整条轨迹的行车走廊
        param：path： type：ndarray
        '''
        point_corridor = []
        for i in range(len(path)):
            point_corridor.append(self.expend_corridor(path[i]))
        return point_corridor

    def expend_corridor(self,point):
        '''
        func: 构建单个路径点的行车走廊
        param:point：  行车坐标点
        '''
        expand_length = np.ones(4) * 2
        obs_rec = get_rec_vertice(point,expand_length)
        return obs_rec





def get_rec_vertice(rec_center:np.array,expand_length):
    # 右下 右上 左上 左下
    return np.array([[rec_center[0]+expand_length[0],rec_center[1]-expand_length[3]],
                     [rec_center[0]+expand_length[0],rec_center[1]+expand_length[1]],
                     [rec_center[0]-expand_length[2],rec_center[1]+expand_length[1]],
                     [rec_center[0]-expand_length[2],rec_center[1]-expand_length[3]]])


def pi_2_pi(theta):
    while theta > np.pi:
        theta -= 2.0 * np.pi

    while theta < -np.pi:
        theta += 2.0 * np.pi

    return theta

def calc_theta(points):
    '''
    func: calc the path yaw
    para:  points:  the path point of trajectory
    '''
    theta = []
    for i in range(len(points)-1):
        dx = (points[i+1][0] - points[i][0])
        dy = (points[i+1][1] - points[i][1])
        angle = np.arctan2(dy,dx)
        theta.append(angle)
    return theta

class Car:
    maxSteerAngle = 0.6
    steerPresion = 10
    wheelBase = 3.5 * 1e-1
    axleToFront = 4.5 * 1e-1
    axleToBack = 1 * 1e-1
    width = 3 * 1e-1


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    points_x = [1.0, 1.5, 2.1, 2.1, 2.1, 2.1, 2.1, 2.1, 2.5, 2.9, 3.3, 3.8]
    points_y = [0.0, 0.3, 0.5, 0.9, 1.2, 1.5, 1.9, 2.2, 2.2, 2.2, 2.2, 2.2]
    goal = [points_x[-1],points_y[-1],0]
    points = np.vstack([points_x,points_y]).T
    obs = []
    smoother = MPC_path_smoother(points,obs,goal)
    # optimized_points = smoother.DL_IAPS()
    optimized_points = np.array(smoother.solve())
    plt.plot(points_x,points_y,label='raw')
    plt.plot(optimized_points[:,0],optimized_points[:,1],label='optimized')
    plt.legend()
    plt.show()
    # print(smoother.ave_length)

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
