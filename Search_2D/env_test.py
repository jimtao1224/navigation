import numpy as np
import random
import math
import csv
import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search_based_Planning/")

from Search_2D import plotting_test, env_test,autorun,test_getneighbor

class Env:
    scale_warning_shown = False
    def __init__(self, scale='B'):
        # autorun_test
        self.autorun = autorun.Environment() 
        scale = self.autorun.scale
        self.robot_size = self.autorun.robot_size
        self.scale = scale       
        self.min_obstacle_count = self.autorun.min_obstacle_count
        
        self.x_range_A = self.autorun.scale_A_size[0]
        self.y_range_A = self.autorun.scale_A_size[1]
        self.x_range_B = self.autorun.scale_B_size[0]
        self.y_range_B = self.autorun.scale_B_size[1]
        
        # ~~    
        # self.x_range_A = 1000  # 尺度A的宽度
        # self.y_range_A = 800  # 尺度A的高度
        # self.x_range_B = 200   # 尺度B的宽度
        # self.y_range_B = 100  # 尺度B的高度 
        
        self.motions = [(-1, 0), (-1, 1), (0, 1), (1, 1),
                        (1, 0), (1, -1), (0, -1), (-1, -1)]
        self.obs = self.obs_map()
        self.obstacle_coverage = self.get_obstacle_coverage()
        self.map_size = self.get_map_size()
        self.obs_total = len(self.obs)
        
    def update_obs(self, obs):
        self.obs = obs
    def obs_map(self):
        """
        Initialize obstacles' positions
        :return: map of obstacles
        """
        obs = set()
        x_A = self.x_range_A
        y_A = self.y_range_A
        obs_A = set()
        obs_A = {(i, 0) for i in range(x_A)} | \
                {(i, y_A - 1) for i in range(x_A)} | \
                {(0, i) for i in range(y_A)} | \
                {(x_A - 1, i) for i in range(y_A)}
                # Add complex obstacles (inner boundaries)
        for i in range(150, 200):  # Increase size and complexity
            for j in range(0, 700):  # Increase size and complexity
                obs_A.add((i, j))

        for i in range(300, 400):  # Increase size and complexity
            for j in range(300, 800):  # Increase size and complexity
                obs_A.add((i, j))

        for i in range(600, 700):  # Increase size and complexity
            for j in range(200, 600):  # Increase size and complexity
                obs_A.add((i, j))

        for i in range(834, 967):  # Increase size and complexity
            for j in range(123, 680):  # Increase size and complexity
                obs_A.add((i, j))

        # Add more complex obs_Atacles
        for i in range(200, 400, 20):  # Increase step size
            for j in range(200, 400, 20):  # Increase step size
                obs_A.add((i, j))

        for i in range(600, 800, 20):  # Increase step size
            for j in range(400, 600, 20):  # Increase step size
                obs_A.add((i, j))

        for i in range(400, 600, 20):  # Increase step size
            for j in range(100, 300, 20):  # Increase step size
                obs_A.add((i, j))

        for i in range(700, 900, 20):  # Increase step size
            for j in range(500, 700, 20):  # Increase step size
                obs_A.add((i, j))
        # for i in range(20, 40):
        #     obs_A.add((i, 30))
        # for i in range(30):
        #     obs_A.add((40, i))
        # for i in range(30, 60):
        #     obs_A.add((60, i))
        # for i in range(32):
        #     obs_A.add((80, i))
         
        x_B = self.x_range_B
        y_B = self.y_range_B
        obs_B = set()
        x_ratio = self.x_range_A / self.x_range_B
        y_ratio = self.y_range_A / self.y_range_B
        if int(x_ratio)*int(y_ratio) < int(self.robot_size[0]*self.robot_size[1]) and not Env.scale_warning_shown:
            print("尺度錯誤,以機器人尺寸為基準,重新設定尺度")
            Env.scale_warning_shown = True
            self.x_range_B = int(self.x_range_A/int(self.robot_size[0]))
            self.y_range_B = int(self.y_range_A/int(self.robot_size[1]))
            
            x_B = self.x_range_B 
            y_B = self.y_range_B
            x_ratio = self.x_range_A / self.x_range_B
            y_ratio = self.y_range_A / self.y_range_B
        for y_B in range(self.y_range_B):
            for x_B in range(self.x_range_B):
                if (x_B == 0 or x_B == self.x_range_B -1 or y_B == 0 or y_B == self.y_range_B -1):
                    # print(f"B节点({x_B}, {y_B}) 包含的A节点障碍物数量: 0")
                    obs_B.add((x_B, y_B))
                else:                
                    x_A_start = x_B * x_ratio
                    y_A_start = y_B * y_ratio
                    x_A_end = (x_B + 1) * x_ratio
                    y_A_end = (y_B + 1) * y_ratio
                    obstacle_count = sum((x_A, y_A) in obs_A
                                    for y_A in range(int(y_A_start), int(y_A_end))
                                    for x_A in range(int(x_A_start), int(x_A_end)))
                    if obstacle_count >= self.min_obstacle_count:
                        # with open('output4.txt', 'a') as file:
                        #     # 使用str.format()来格式化字符串，将x_B, y_B, obstacle_count组合成一行，以逗号分隔
                        #     line = "{},{},{}\n".format(x_B, y_B, obstacle_count)
                        #     file.write(line)

                        # print(f"B节点({x_B}, {y_B}) 包含的A节点障碍物数量: {obstacle_count}")
                        obs_B.add((x_B, y_B))
    
        if self.scale == 'A':
            obs = obs_A
        elif self.scale == 'B':
            obs = obs_B
        return obs

    def get_obstacle_coverage(self):
        """
        Calculate the complexity of the map by the ratio of obstacles
        :return: complexity as a float
        """
        if self.scale == 'A':
            total_cells = self.x_range_A * self.y_range_A
        elif self.scale == 'B':
            total_cells = self.x_range_B * self.y_range_B
        obstacle_coverage = len(self.obs) / total_cells
        return obstacle_coverage
        # return len(self.obs) / total_cells

    def get_map_size(self):
        """
        Return the size of the map based on the scale
        :return: tuple of (width, height)
        """
        if self.scale == 'A':
            return self.x_range_A, self.y_range_A
        elif self.scale == 'B':
            return self.x_range_B, self.y_range_B

# env = Env()  # 使用尺度A初始化環境


# if  not Env.map_info:
#     print("障礙物總數:", len(env.obs))
#     print("障礙物覆蓋率:",env.obstacle_coverage)
#     print("地圖大小:", env.get_map_size())
#     Env.map_info = True   
# with open("obstacle_coverage.csv", "w", newline="") as csvfile:
#   writer = csv.writer(csvfile)
#   writer.writerow(data_set)
 
# print("障礙物到最近鄰居的平均距離:", env.avg_obstacle_distance())
