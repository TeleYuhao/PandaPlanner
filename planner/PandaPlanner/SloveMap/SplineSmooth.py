from planner.PandaPlanner.SloveMap.RoutePlanner import RoutePlanner
from planner.PandaPlanner.Spline2d import Spline2d
from planner.PandaPlanner.TrajectoryPlanner.CubicSpline import Spline2D
import numpy as np
import matplotlib.pyplot as plt

if __name__ == '__main__':
    route_id = [249, 255, 430, 247, 112, 188, 143, 79, 86, 1, 400, 216, 427, 266, 105, 40, 39, 402, 11, 154, 393, 341,
                380, 311,
                241, 47, 48, 129, 221, 237, 7, 54, 53, 205, 422, 243, 115, 84, 89, 212, 211, 56, 145, 146, 147, 418,
                202, 203, 14,
                45, 164, 190, 134, 260, 386, 379, 369, 363, 23]
    RP = RoutePlanner(route_id)
    Ref,LaneChange,distance = RP.GetRefLaneSuccessor(255)
    Ref += np.random.rand(*Ref.shape)/1e8
    # Ref[:,0] -= 300000
    # Ref[:,1] -= 3000000
    Spline = Spline2d(Ref[::10,0],Ref[::10,1])
    path = []
    yaw_list = []
    for i in np.arange(0,150,0.1):
        xy = Spline.frenet_to_cartesian1D([i,0])
        path.append(xy)
        yaw_list.append(Spline.calc_yaw(i))
    path = np.array(path)

    plt.plot(path[:,0],path[:,1],label="Frenet")
    # plt.plot(Ref[:,0],Ref[:,1],label = "Vertical")
    plt.show()

    plt.plot(yaw_list)
    plt.show()