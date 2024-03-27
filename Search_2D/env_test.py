import numpy as np
import random
import math
class Env:
    def __init__(self, scale = 'A'):
        self.scale = scale
        self.x_range_A = 100  # 尺度A的宽度
        self.y_range_A = 100  # 尺度A的高度
        self.x_range_B = 10   # 尺度B的宽度
        self.y_range_B = 10   # 尺度B的高度
        self.motions = [(-1, 0), (-1, 1), (0, 1), (1, 1),
                        (1, 0), (1, -1), (0, -1), (-1, -1)]
        self.obs = self.obs_map()
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
        if self.scale == 'A' :
            x = self.x_range_A
            y = self.y_range_A
            
        elif self.scale == 'B':
            x = self.x_range_B
            y = self.y_range_B
            
        else:
            raise ValueError("Invalid scale: {}".format(self.scale))    
        
        obs = set()
        obs = {(i, 0) for i in range(x)} | \
              {(i, y - 1) for i in range(x)} | \
              {(0, i) for i in range(y)} | \
              {(x - 1, i) for i in range(y)}
        for i in range(20, 40):
            obs.add((i, 30))
        for i in range(30):
            obs.add((40, i))
        for i in range(30, 60):
            obs.add((60, i))
        for i in range(32):
            obs.add((80, i))


        return obs

    def get_obstacle_density(self):
        """
        Calculate the complexity of the map by the ratio of obstacles
        :return: complexity as a float
        """
        if self.scale == 'A':
            total_cells = self.x_range_A * self.y_range_A
        elif self.scale == 'B':
            total_cells = self.x_range_B * self.y_range_B

        return len(self.obs) / total_cells

    def get_map_size(self):
        """
        Return the size of the map based on the scale
        :return: tuple of (width, height)
        """
        if self.scale == 'A':
            return self.x_range_A, self.y_range_A
        elif self.scale == 'B':
            return self.x_range_B, self.y_range_B
    def avg_obstacle_distance(self):
        """
        Calculate the average Euclidean distance between each pair of obstacles.
        This provides an indication of how spread out or clustered the obstacles are.
        :return: average distance as a float
        """
        if not self.obs:
            return 0

        total_distance = 0
        count = 0
        obstacles = list(self.obs)
        for i in range(len(obstacles)):
            for j in range(i + 1, len(obstacles)):
                dist = math.sqrt((obstacles[i][0] - obstacles[j][0])**2 + (obstacles[i][1] - obstacles[j][1])**2)
                total_distance += dist
                count += 1

        if count > 0:
            return total_distance / count
        else:
            return 0
env = Env()  # 使用尺度A初始化環境
print("障礙物總數:", len(env.obs))
print("障礙物密度:", env.get_obstacle_density())
print("地圖大小:", env.get_map_size())    
print("障礙物到最近鄰居的平均距離:", env.avg_obstacle_distance())
