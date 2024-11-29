'''
Author: ruiqingli ruiqingli@foxmail.com
Date: 2024-09-27 10:25:38
LastEditors: ruiqingli ruiqingli@foxmail.com
LastEditTime: 2024-09-27 11:32:54
FilePath: /undefined/home/lrq/source_disk/mypybind/Spline2d/build/testSpline.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''
from Spline2d import Spline2d
import  matplotlib.pyplot as plt
import numpy as np
np.set_printoptions(suppress=True)


import time
start = time.time()
wx = np.array([0.0, 10.0, 20.5, 35.0, 70.5])
wy = np.array([0.0, -6.0, 5.0, 6.5, 0.0])

# wx = np.array([0.0, 100.0 ])
# wy = np.array([0.0, 0])
s = Spline2d(wx,wy)
s_fre = s.cartesian_to_frenet2D([10, 0, np.pi/4, 2])
print(s_fre)
path = s.GetPath()[:-1]
print(time.time() - start)
plt.plot(path[:,0],path[:,1])
plt.show()
# print(path)
