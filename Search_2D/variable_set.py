import random
import os
import sys
import sched
import time
import matplotlib.pyplot as plt
sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search_based_Planning/")
# from Search_2D import plotting_test, env_test 
# from test_getneighbor import d_star_main
# from env_test import Env
class Environment:
    def __init__(self):
        scale = 'A'
        self.scale = scale
        start_point = (2, 190)
        end_point = (980, 300)
        scale_A_size = (1000, 800)
        scale_B_size = (100,100)
        robot_size = (1,1)
        min_obstacle_count = 10
        self.min_obstacle_count = min_obstacle_count
        self.robot_size = robot_size
        self.start_point = start_point
        self.end_point = end_point
        self.scale_A_size = scale_A_size
        self.scale_B_size = scale_B_size
        print(self.scale, "environment scale")
def timed_execution(sc,scale): 
    next_scale = 'A'if scale == 'B' else 'B'
    main(next_scale)  # 执行包含 DStar 算法的 main 函数
    sc.enter(1, 1, timed_execution, (sc,next_scale))  # 每60秒执行一次

def main(scale):
    envi = Environment(scale)
    plt.figure(figsize=(12, 6))
    # Env().scale = scale
    # d_star_main(envi)

scheduler = sched.scheduler(time.time, time.sleep)    

if __name__ == '__main__':
    initial_scale = 'B'
    main(initial_scale)
    scheduler.enter(1, 1, timed_execution, (scheduler,initial_scale))
    scheduler.run()
    
