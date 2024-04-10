import numpy as np
import random
import math
import csv
import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search_based_Planning/")

# from Search_2D import plotting_test
from variable_set import Environment
# from Search_2D import run
# from run import data_set
class Env:
    scale_warning_shown = False
    def __init__(self,scale):
        print("Env scale:", scale)
        environment = Environment(scale=scale)
        start_point = environment.start_point
        end_point = environment.end_point
        scale = environment.scale
        self.start_point= start_point
        self.end_point = end_point
        self.s_start_first= start_point
        self.s_goal_first = end_point
        self.robot_size = environment.robot_size
        self.scale = scale       
        self.min_obstacle_count = environment.min_obstacle_count
        self.x_range_A = environment.scale_A_size[0]
        self.y_range_A = environment.scale_A_size[1]
        self.x_range_B = environment.scale_B_size[0]
        self.y_range_B = environment.scale_B_size[1]

        
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

        if scale == 'A':
            self.start_point = self.s_start_first
            self.end_point = self.s_goal_first
            self.start_point = self.adjust_for_obstacles(self.start_point, self.obs, "出發點")
            self.end_point = self.adjust_for_obstacles(self.end_point, self.obs, "目標點")
        elif scale == 'B':
            self.start_point = self.convert_to_B_scale(self.s_start_first)
            self.end_point = self.convert_to_B_scale(self.s_goal_first)
            print("B尺度各點座標,尚未檢驗",self.start_point, self.end_point)
            self.start_point = self.adjust_coordinates(self.start_point, self.x_range_B, self.y_range_B, "出發點")
            self.end_point = self.adjust_coordinates(self.end_point, self.x_range_B, self.y_range_B, "目標點")
            self.start_point = self.adjust_for_obstacles(self.start_point, self.obs, "出發點")
            self.end_point = self.adjust_for_obstacles(self.end_point, self.obs, "目標點")
        self.converted_target_point = self.calculate_target_displacement(self.s_goal_first, self.end_point)
    
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

    def get_map_size(self):
        """
        Return the size of the map based on the scale
        :return: tuple of (width, height)
        """
        if self.scale == 'A':
            return self.x_range_A, self.y_range_A
        elif self.scale == 'B':
            return self.x_range_B, self.y_range_B



    def convert_to_B_scale(self, a_scale_point):
        # 計算A尺度到B尺度的轉換比例
        x_ratio = self.x_range_B / self.x_range_A
        y_ratio = self.y_range_B / self.y_range_A
        # 按比例轉換A尺度點到B尺度
        b_scale_approx = (a_scale_point[0] * x_ratio, a_scale_point[1] * y_ratio)
        offset_x = b_scale_approx[0] - int(b_scale_approx[0])
        offset_y = b_scale_approx[1] - int(b_scale_approx[1])
        if offset_x > 0.5:
            b_scale_x = min(self.x_range_B - 1, int(b_scale_approx[0]) + 1)
        else:
            b_scale_x = max(1, int(b_scale_approx[0]))

        if offset_y > 0.5:
            b_scale_y = min(self.y_range_B - 1, int(b_scale_approx[1]) + 1)
        else:
            b_scale_y = max(1, int(b_scale_approx[1]))
        b_scale_point = (b_scale_x, b_scale_y)
        return b_scale_point
    def calculate_target_displacement(self,original_target_point, converted_target_point):
    # 计算原始目标点和转换后目标点之间的欧几里得距离
        x_ratio = self.x_range_B / self.x_range_A
        y_ratio = self.y_range_B / self.y_range_A
        original_target_point = (original_target_point[0] * x_ratio, original_target_point[1] * y_ratio)
        displacement = ((converted_target_point[0] - original_target_point[0])**2 + (converted_target_point[1] - original_target_point[1])**2)**0.5
        return displacement
    def adjust_coordinates(self,coord, x_range, y_range,point_name):
            x, y = coord
            if x >= x_range - 1:
                print(f"座標轉換後{point_name}x軸位置錯誤,調整座標")
                x = x_range - 2  # 减少一个单位而不是减到-1
            if y >= y_range - 1:
                print(f"座標轉換後{point_name}y軸位置錯誤,調整座標")
                y = y_range - 2
            return x, y
    def adjust_for_obstacles(self,coord, obs, point_name):
            if coord in obs:
                print(f"{point_name}與障礙物重疊，正在尋找新的位置...")
                new_coord = (coord[0] + 1, coord[1] + 1)
                # 检查新位置是否仍然重叠，如果是，则再次调整（在实际应用中，可能需要更复杂的处理方式）
                while new_coord in obs:
                    new_coord = (new_coord[0] + 1, new_coord[1] + 1)
                print(f"{point_name}已移至新位置：{new_coord}")
                return new_coord
            return coord  # 如果没有重叠，就返回原坐标
    