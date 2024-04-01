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
    def __init__(self, scale='B'):
        # autorun_test
        # self.autorun = autorun.Environment() 
        # scale = self.autorun.scale
        
        self.scale = scale       

        # self.x_range_A = self.autorun.scale_A_size[0]
        # self.y_range_A = self.autorun.scale_A_size[1]
        # self.x_range_B = self.autorun.scale_B_size[0]
        # self.y_range_B = self.autorun.scale_B_size[1]
        # ~~    
        self.x_range_A = 1000  # 尺度A的宽度
        self.y_range_A = 800  # 尺度A的高度
        self.x_range_B = 200   # 尺度B的宽度
        self.y_range_B = 100  # 尺度B的高度
        obstacle_threshold = 0.01
        self.obstacle_threshold = obstacle_threshold
        self.motions = [(-1, 0), (-1, 1), (0, 1), (1, 1),
                        (1, 0), (1, -1), (0, -1), (-1, -1)]
        self.obs = self.obs_map()
        self.obstacle_coverage = self.get_obstacle_coverage()
        self.map_size = self.get_map_size()
        self.obs_total = len(self.obs)
    def update_obs(self, obs):
        # if self.scale == 'A' :
        self.obs = obs
        # elif self.scale == 'B':
        #     self.obs 
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
        # x_ratio = self.x_range_B / self.x_range_A
        # y_ratio = self.y_range_B / self.y_range_A
        x_ratio = self.x_range_A / self.x_range_B
        y_ratio = self.y_range_A / self.y_range_B
        min_obstacle_count = 10  # 例如，我们只关注至少包含1个障碍物的B节点

# 遍历尺度B的每个节点
        for y_B in range(self.y_range_B):
            for x_B in range(self.x_range_B):
                # 计算尺度A中对应的区域

                # is_on_boundary = (x_B == 0 or x_B == self.x_range_B  or y_B == 0 or y_B == self.y_range_B )
                # print(f"B节点({x_B}, {y_B}) 对应的A节点区域: ({x_A_start}, {y_A_start}) - ({x_A_end}, {y_A_end})")
                # 检查并统计区域内的障碍物数量
                # print(f"障碍物坐标: {x_B}, {y_B}")
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
                    if obstacle_count >= min_obstacle_count:
                        # with open('output4.txt', 'a') as file:
                        #     # 使用str.format()来格式化字符串，将x_B, y_B, obstacle_count组合成一行，以逗号分隔
                        #     line = "{},{},{}\n".format(x_B, y_B, obstacle_count)
                        #     file.write(line)

                        # print(f"B节点({x_B}, {y_B}) 包含的A节点障碍物数量: {obstacle_count}")
                        obs_B.add((x_B, y_B))

                # 只处理或输出满足阈值条件的B节点信息
                
                    # print(f"B节点({x_B}, {y_B}) 包含的A节点障碍物数量: {obstacle_count}")
        # for y in range(self.y_range_B):
        #     for x in range(self.x_range_B):
                # 对于B尺度的每个节点，检查对应的A尺度10x10区块中是否存在障碍物 
                # if any((x * 10 + dx, y * 10 + dy) in obs_A for dy in range(10) for dx in range(10)):
                #     obs_B.add((x, y))
                # if any((x * x_ratio + dx, y * y_ratio + dy) in obs_A for dy in range(int(self.y_range_B)) 
                #        for dx in range(int(self.x_range_B))):
                #     obs_B.add((x, y))
                # obstacle_count = sum((x * 10 + dx, y * 10 + dy) in obs_A for dy 
                #                      in range(10) for dx in range(10))
                # if obstacle_count / x_ratio*y_ratio >= self.obstacle_threshold:
                #     obs_B.add((x, y))


                # if any((x * x_ratio + dx * x_ratio / 10, y * y_ratio + dy * y_ratio / 10) in obs_A
                #     for dy in range(10) for dx in range(10)):
                #     obs_B.add((x, y))
                # if any((x + dx / 10 * x_ratio, y + dy / 10 * y_ratio) in obs_A 
                #     for dy in range(10) for dx in range(10)):
                #     obs_B.add((x, y))

                # obstacle_count = sum((x * x_ratio + dx, y * y_ratio + dy) in obs_A
                #      for dy in range(int(y_ratio)) for dx in range(int(x_ratio)))
                # if (obstacle_count / (x_ratio * y_ratio)) >= self.obstacle_threshold:
                #     obs_B.add((x, y))

    

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
    # def show(self):
    #     print("障礙物總數:", self.obs_total)
    #     print("障礙物覆蓋率:",self.obstacle_coverage)
    #     print("地圖大小:", self.map_size)
    

    # def avg_obstacle_distance(self):
    #     """
    #     Calculate the average Euclidean distance between each pair of obstacles.
    #     This provides an indication of how spread out or clustered the obstacles are.
    #     :return: average distance as a float
    #     """
    #     if not self.obs:
    #         return 0

    #     total_distance = 0
    #     count = 0
    #     obstacles = list(self.obs)
    #     for i in range(len(obstacles)):
    #         for j in range(i + 1, len(obstacles)):
    #             dist = math.sqrt((obstacles[i][0] - obstacles[j][0])**2 + (obstacles[i][1] - obstacles[j][1])**2)
    #             total_distance += dist
    #             count += 1

    #     if count > 0:
    #         return total_distance / count
    #     else:
        #         return 0
env = Env()  # 使用尺度A初始化環境
# data_set = [env.obstacle_coverage,env.get_map_size()]
print("障礙物總數:", len(env.obs))
print("障礙物覆蓋率:",env.obstacle_coverage)
# with open("obstacle_coverage.csv", "w", newline="") as csvfile:
#   writer = csv.writer(csvfile)
#   writer.writerow(data_set)
print("地圖大小:", env.get_map_size())    
# print("障礙物到最近鄰居的平均距離:", env.avg_obstacle_distance())
