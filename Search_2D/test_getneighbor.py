"""
D_star_Lite 2D
@author: huiming zhou
"""

import os
import sys
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation    
import time
sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search_based_Planning/")

from Search_2D import plotting_test, env_test,autorun
# from multiprocessing import Pool, Process
# from concurrent.futures import ProcessPoolExecutor
# from concurrent.futures import ThreadPoolExecutor

class DStar:
    def __init__(self, s_start, s_goal, heuristic_type):
        self.Env = env_test.Env()
        scale = self.Env.scale
        self.scale = scale
        print(self.scale, "dstar scale")
        

        # autorun_test
        self.s_start_auto = autorun.Environment().start_point
        self.s_goal_auto = autorun.Environment().end_point
        print(self.s_start_auto, self.s_goal_auto)
        if scale == 'A':
            self.s_start = self.s_start_auto
            self.s_goal = self.s_goal_auto
        elif scale == 'B':
            self.s_start = self.convert_to_B_scale(self.s_start_auto)
            self.s_goal = self.convert_to_B_scale(self.s_goal_auto)
        # ~~~~


        # if scale == 'A':
        #     self.s_start = s_start
        #     self.s_goal = s_goal
        # elif scale == 'B':
        #     self.s_start = self.convert_to_B_scale(s_start)
        #     self.s_goal = self.convert_to_B_scale(s_goal)
        
        print(self.s_start, self.s_goal)
        self.plotter = plotting_test.Plotting(self.s_start, self.s_goal,self.Env) 
          # class Env
        self.u_set = self.Env.motions  # feasible input set
        self.Env.obs = self.Env.obs
        self.heuristic_type = heuristic_type
        self.x, self.y = self.set_scale(scale)
        self.g, self.rhs, self.U = {}, {}, {}
        self.km = 0
        for i in range(1, self.x - 1):
            for j in range(1, self.y - 1):
                self.rhs[(i, j)] = float("inf")
                self.g[(i, j)] = float("inf")
        self.rhs[self.s_goal] = 0.0
        self.U[self.s_goal] = self.CalculateKey(self.s_goal)
        # print(self.U)
        self.visited = set()
        self.count=0
        # self.fig= plt.figure()
    
    def set_scale(self,scale):
        self.scale = scale
        if scale == 'A':
            self.x = self.Env.x_range_A
            self.y = self.Env.y_range_A
            return self.x,self.y
        elif self.scale == 'B':
            self.x = self.Env.x_range_B
            self.y = self.Env.y_range_B
            return self.x,self.y

    def run(self):
        # self.Plot.plot_grid("D* Lite")
        # if self.s_goal == self.Env.obs:
        #     print("The goal is in the obstacle")
        #     return
        self.plotter.plot_grid("D* Lite")
        print("障礙物總數:", len(self.Env.obs))
        print("障礙物覆蓋率:",self.Env.obstacle_coverage)
        print("地圖大小:", self.Env.get_map_size())
        start_time = time.time()
        print("ComputePath")
        self.ComputePath()
        if self.scale == 'A':
            print("plot_path")
            self.plotter.plot_path(self.extract_path())    
            print("animate_path")
            self.plotter.animate_path(self.extract_path())

        elif self.scale == 'B':
            print("plot_path")
            self.plotter.plot_path(self.extract_path())    
            print("animate_path")
            self.plotter.animate_path(self.extract_path())
        print(len(self.extract_path())-1,"total_time")
        end_time = time.time()  
        print("Map generation time:", end_time - start_time, "seconds") 
        plt.show()
        plt.close()

        

    def ComputePath(self):
        while True:
            s, v = self.TopKey()  
            if v >=self.CalculateKey(self.s_start) and \
                    self.rhs[self.s_start] == self.g[self.s_start]:
                break
            # print(self.U)    
            k_old = v
            self.U.pop(s)
            self.visited.add(s)
            if k_old < self.CalculateKey(s):
                self.U[s] = self.CalculateKey(s)
            elif self.g[s] > self.rhs[s]:
                self.g[s] = self.rhs[s]
                for x in self.get_neighbor(s):
                    self.UpdateVertex(x)
            else:
                self.g[s] = float("inf")
                self.UpdateVertex(s)
                for x in self.get_neighbor(s):
                    self.UpdateVertex(x)     

    def UpdateVertex(self, s):
        if s != self.s_goal:
            self.rhs[s] = float("inf")
            for x in self.get_neighbor(s):
                if x not in self.g:
                    continue  # 跳过当前的邻居节点，继续下一个
                self.rhs[s] = min(self.rhs[s], self.g[x] + self.cost(s, x))
        if s in self.U:
            self.U.pop(s)
        if s not in self.g:
            self.g[s] = float('inf')  # 为不存在的键赋予默认值
        if self.g[s] != self.rhs[s]:
            self.U[s] = self.CalculateKey(s)
   
   
    def CalculateKey(self, s):
        return [min(self.g[s], self.rhs[s]) + self.h(self.s_start, s) + self.km,
                min(self.g[s], self.rhs[s])]

    def TopKey(self):
        """
        :return: return the min key and its value.
        """
        s = min(self.U, key=self.U.get)
        return s, self.U[s]

    def h(self, s_start, s_goal):
        heuristic_type = self.heuristic_type  # heuristic type

        if heuristic_type == "manhattan":
            return abs(s_goal[0] - s_start[0]) + abs(s_goal[1] - s_start[1])
        else:
            return math.hypot(s_goal[0] - s_start[0], s_goal[1] - s_start[1])

    def cost(self, s_start, s_goal):
        """
        Calculate Cost for this motion
        :param s_start: starting node
        :param s_goal: end node
        :return:  Cost for this motion
        :note: Cost function could be more complicate!
        """

        if self.is_collision(s_start, s_goal):
            return float("inf")

        return math.hypot(s_goal[0] - s_start[0], s_goal[1] - s_start[1])

    def is_collision(self, s_start, s_end):
        if s_start in self.Env.obs or s_end in self.Env.obs:
            return True

        if s_start[0] != s_end[0] and s_start[1] != s_end[1]:
            if s_end[0] - s_start[0] == s_start[1] - s_end[1]:
                s1 = (min(s_start[0], s_end[0]), min(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
            else:
                s1 = (min(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), min(s_start[1], s_end[1]))

            if s1 in self.Env.obs or s2 in self.Env.obs:
                return True

        return False

    def get_neighbor(self, s):
        nei_list = set()
        distance = self.calculate_distance(s, self.s_start)
        # if distance <= 2: 
        #     self.step = 1  
        #     (distance)
        # else  :
        #     self.step = 2 
        # with open("output3.txt", "a") as file: 
        #     file.write(f"{s},{self.U}\n")
        for u in self.u_set:
            s_next = tuple([s[i] + u[i]   for i in range(2)])    
            # if 0 <= s_next[0] < 100 and 0 <= s_next[1] < 60 and s_next not in self.Env.obs:    
            if s_next not in self.Env.obs:
                nei_list.add(s_next) 
        return nei_list

    def calculate_distance(self, s1, s2):
        return (s1[0] - s2[0]) + abs(s1[1] - s2[1])
    
    def extract_path(self):
        """
        Extract the path based on the PARENT set.
        :return: The planning path
        """

        path = [self.s_start]
        s = self.s_start

        for k in range(10000):
            g_list = {}
            for x in self.get_neighbor(s):
                if not self.is_collision(s, x):
                    g_list[x] = self.g[x]
            s = min(g_list, key=g_list.get)
            path.append(s)
            if s == self.s_goal:
                break
        return list(path)

    def plot_visited(self, visited):
        color = ['gainsboro', 'lightgray', 'silver', 'darkgray',
                 'bisque', 'navajowhite', 'moccasin', 'wheat',
                 'powderblue', 'skyblue', 'lightskyblue', 'cornflowerblue']

        if self.count >= len(color) - 1:
            self.count = 0

        for x in visited:
            plt.plot(x[0], x[1], marker='s', color=color[self.count])
    def convert_to_B_scale(self, a_scale_point):
        # 計算A尺度到B尺度的轉換比例
        x_ratio = self.Env.x_range_B / self.Env.x_range_A
        y_ratio = self.Env.y_range_B / self.Env.y_range_A
        # 按比例轉換A尺度點到B尺度
        b_scale_approx = (a_scale_point[0] * x_ratio, a_scale_point[1] * y_ratio)
        offset_x = b_scale_approx[0] - int(b_scale_approx[0])
        offset_y = b_scale_approx[1] - int(b_scale_approx[1])
        if offset_x > 0.5:
            b_scale_x = min(self.Env.x_range_B - 1, int(b_scale_approx[0]) + 1)
        else:
            b_scale_x = max(1, int(b_scale_approx[0]))

        if offset_y > 0.5:
            b_scale_y = min(self.Env.y_range_B - 1, int(b_scale_approx[1]) + 1)
        else:
            b_scale_y = max(1, int(b_scale_approx[1]))
        # b_scale_x = max(1, min(self.Env.x_range_B - 1, int(b_scale_approx[0])))
        # b_scale_y = max(1, min(self.Env.y_range_B - 1, int(b_scale_approx[1])))
        b_scale_point = (b_scale_x, b_scale_y)
        return b_scale_point
    

def d_star_main():
    s_start = (10, 10)
    s_goal = (980, 330)
    dstar = DStar(s_start, s_goal, "euclidean") 
    dstar.run()


if __name__ == '__main__':
    d_star_main()
