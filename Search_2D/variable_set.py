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
    start_point = None
    end_point = None
    def __init__(self,scale):
        self.scale = scale
        Environment.start_point = (800, 73)
        Environment.end_point = (50,460 )
        # if Environment.start_point is None and Environment.end_point is None:
        # Environment.start_point = (random.randint(1, 999), random.randint(1, 999))
        # Environment.end_point = (random.randint(1, 999), random.randint(1, 999))
        # Environment.start_point = (10, 10)
        # Environment.end_point = (88, 80)
        # scale_A_size = (100, 100)
        # scale_B_size = (10,10)
        scale_A_size = (1000, 1000)
        scale_B_size = (100,100)
        robot_size = (1,1)
        num_initial_obstacles = 16000
        # num_initial_obstacles = random.randint(2000, 20000)
        min_obstacle_count = 2.25
        seed = 15
        self.seed = seed
        self.num_initial_obstacles = num_initial_obstacles
        self.min_obstacle_count = min_obstacle_count
        self.start_point = Environment.start_point
        self.robot_size = robot_size
        self.end_point = Environment.end_point
        self.scale_A_size = scale_A_size
        self.scale_B_size = scale_B_size
        self.s_start_first = self.start_point
        self.s_goal_first = self.end_point
        # print(self.scale, "environment scale")
    # def scale(self):
    #     from test_getneighbor import d_star_main
    #     scale =d_star_main()







    
