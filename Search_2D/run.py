import matplotlib.pyplot as plt
import time
import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search_based_Planning/")
from Search_2D import plotting_test
from variable_set import Environment
# from env_test import Env  # 替换为您的路径规划类模块
from plotting_test import Plotting  # 替换为您的绘图类模块
# from test_getneighbor import DStar  # 替换为您的路径规划类模块
def data_set(scale):
    env = Environment(scale=scale)
def run_for_scale(scale):
    env = Environment(scale=scale)
    
    planner = plotting_test.DStar(env)  # 假设PathPlanner类接受一个环境实例

    plt.figure()  # 为当前尺度创建一个新的图形窗口
    planner.plotter.plot_grid(f"D* Lite - Scale {scale}")
    print(f"Scale {scale}:")
    print("障礙物總數:", len(env.obs))
    print("障礙物覆蓋率:", env.obstacle_coverage)
    print("地圖大小:", env.get_map_size())

    start_time = time.time()
    print("ComputePath")
    planner.ComputePath()
    print("plot_path")
    planner.plotter.plot_path(planner.extract_path())
    print("animate_path")
    planner.plotter.animate_path(planner.extract_path())
    print(len(planner.extract_path())-1, "total_time")
    end_time = time.time()
    print("Map generation time:", end_time - start_time, "seconds")

    plt.show()  # 显示当前尺度的图形
    plt.close()  # 关闭当前图形窗口

def main():
    run_for_scale('A')  # 为A尺度执行路径规划和绘图
    run_for_scale('B')  # 为B尺度执行路径规划和绘图

if __name__ == '__main__':
    main()
