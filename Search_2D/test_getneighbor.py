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

from Search_2D import plotting_test, env_test
# from multiprocessing import Pool, Process
# from concurrent.futures import ProcessPoolExecutor
# from concurrent.futures import ThreadPoolExecutor

class DStar:
    def __init__(self, s_start, s_goal, heuristic_type):
        scale = env_test.Env().scale
        self.scale = scale
        self.s_start, self.s_goal = self.coordinate_translate(s_start, s_goal)
        print(self.s_start, self.s_goal)
        self.heuristic_type = heuristic_type
        self.Env = env_test.Env()  # class Env
        self.plotter = plotting_test.Plotting(self.s_start, self.s_goal,self.Env) 
        self.u_set = self.Env.motions  # feasible input set
        self.Env.obs = self.Env.obs
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
        self.plotter.plot_grid("D* Lite")
        start_time = time.time()
        # self.node_list(self.x,self.y)
        self.ComputePath()
        # print(self.extract_path())
        self.plotter.plot_path(self.extract_path())
        self.plotter.animate_path(self.extract_path())
        # self.plot_path(self.extract_path())
        end_time = time.time()  
        print("Map generation time:", end_time - start_time, "seconds") 
        plt.show()
        

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
                self.rhs[s] = min(self.rhs[s], self.g[x] + self.cost(s, x))
        if s in self.U:
            self.U.pop(s)

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
        #     print(distance)
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

    def coordinate_translate(self,s_start,s_goal):
        if self.scale == 'B':
            return (s_start[0] // 10, s_start[1] // 10), (s_goal[0] // 10, s_goal[1] // 10)
        else :
            return s_start,s_goal
    def node_list(scale,x_range,y_range):
        node_list = []
        for i in range(1,x_range-1):
            for j in range(1,y_range-1):
                node_list.append((i, j))
        return node_list

def main():
    s_start = (10, 10)
    s_goal = (400, 750)
    dstar = DStar(s_start, s_goal, "euclidean") 
    dstar.run()


if __name__ == '__main__':
    main()
