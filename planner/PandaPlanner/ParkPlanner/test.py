import numpy as np
import matplotlib.pyplot as plt
from CubicSpline import Spline2D
from planner.PandaPlanner.Spline2d import Spline2d
from reeds_shepp import calc_reeds_shepp
from ref_smoother import path_smoother
from OBCA_optimizer import OBCAOptimizer
from utils import Config_step_1 as C
import pyproj
wgs84 = pyproj.Proj(proj='latlong', datum='WGS84')
utm = pyproj.Proj(proj='utm', zone=49, ellps='WGS84')
tmerc = pyproj.Proj(proj="tmerc", lon_0=108.90577060170472, lat_0=34.37650478465651, ellps='WGS84')

utm2tmerc   = pyproj.Transformer.from_proj( utm, tmerc)
def Utm2TmercPath(path):
    '''
        function: transform the umt path to tmerc point
    '''
    transformed_path = utm2tmerc.transform(path[:, 0], path[:, 1])
    return np.vstack((transformed_path[0], transformed_path[1])).T
# 从 .npy 文件加载轨迹数据
traj = np.load('transform.npy')
traj += np.random.rand(*traj.shape) / 1e8  # 避免奇异矩阵问题
traj = Utm2TmercPath(traj)
# spline = Spline2D(traj[:, 0], traj[:, 1])
print(1)
spline = Spline2d(traj[::5, 0], traj[::5, 1])

print(2)
# start_yaw = np.arctan((traj[-1][1] - traj[-2][1])/(traj[-1][0] - traj[-2][0])) + np.pi
# start_state = [traj[-1][0],traj[-1][1],start_yaw]
# end_state = [306846.23006, 3805738.0227, np.deg2rad(98.3415756) + np.pi/2]
# end_state = [306846.23006, 3805738.0227, np.deg2rad(171.3)]
# end_state = [306847.0388634986, 3805735.428941139, np.deg2rad(171.31813800000003)]
# end_state = [306846.23006, 3805738.0227, (98.3415756)]
# end_state = [306846.63988, 3805740.4858, -np.deg2rad(98.3415756)]

start_state = [-566.6413452273209, -158.06628904102007, 3.5062346948980827]
# start_state = [-566.2550186303242, -157.66696743875514, -2.7850037708897153]
end_state =[-594.674016850958, -165.13897001695577, 9.279189868193582]
# end_state = [-595.3045,-162.522971,  -0.14558809257579775]

reeds_shepp_path = calc_reeds_shepp(start_state,end_state,maxc=0.1)
for i in range(len(reeds_shepp_path.yaw)):
    if(reeds_shepp_path.yaw[i] < 0):
        reeds_shepp_path.yaw[i] += 2*np.pi
zero_path = np.zeros(len(reeds_shepp_path.x))
path_MPC = np.vstack((reeds_shepp_path.x,reeds_shepp_path.y,zero_path,reeds_shepp_path.yaw,zero_path)).T
path = np.vstack((reeds_shepp_path.x,reeds_shepp_path.y)).T

# smoother_MPC = MPC_path_smoother(start_state,end_state)
# smoother = path_smoother(path)
# smoother_MPC.solve()
# optimized_points = np.array(smoother.solve())


OBCA = OBCAOptimizer()
OBCA.initialize(start_state,end_state,[])
# OBCA.initialize(path_MPC,[])
OBCA.solve()

parkSpline = Spline2d(OBCA.x_opt.full().flatten(),OBCA.y_opt.full().flatten())
parkpath = parkSpline.GetPath()
plt.plot(reeds_shepp_path.x,reeds_shepp_path.y,label="reedshepp_path")
plt.plot(traj[:,0],traj[:,1], label="global_traj")
plt.plot(OBCA.x_opt,OBCA.y_opt,label="OBCA_optimized")
plt.plot(parkpath[:,0],parkpath[:,1],label="cubic_spline")
# plt.plot(smoother_MPC.x_opt,smoother_MPC.y_opt,label="MPC_optimized")
# plt.plot(optimized_points[:,0],optimized_points[:,1],label="optimized")
plt.legend()
plt.show()
plt.subplot(311)
plt.plot(OBCA.v_opt)
plt.subplot(312)
plt.plot(OBCA.a_opt)
plt.subplot(313)
plt.plot(OBCA.steer_opt)
plt.show()