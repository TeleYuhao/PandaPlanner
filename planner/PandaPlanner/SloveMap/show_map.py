'''
@Project :高精度地图
@Author : YuhaoDu
@Date : 2024/10/12 
'''
from utils.visualizer import Visualizer

if __name__ == '__main__':
    vis = Visualizer()
    path = "chanan-v0.4-20241008.xodr"
    vis.show_map(path)