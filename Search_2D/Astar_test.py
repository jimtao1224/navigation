"""
A_star 2D
@author: huiming zhou
"""

import os
import sys
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation   
import heapq
import time
import pandas as pd
import psutil
sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search_based_Planning/")


from Search_2D import plotting_test, env_test
from variable_set import Environment
from env_test import Env

class AStar:
    """AStar set the cost + heuristics as the priority
    """
    def __init__(self, heuristic_type,scale):

        self.heuristic_type = heuristic_type

        environment = Env(scale=scale)
        self.Env = environment
        scale = self.Env.scale
        self.scale = scale
        self.u_set = self.Env.motions  # feasible input set
        self.obs = self.Env.obs
        self.s_start_first = self.Env.s_start_first
        self.s_goal_first = self.Env.s_goal_first
        self.s_start = self.Env.start_point
        self.s_goal = self.Env.end_point
        self.min_obstacle_count = self.Env.min_obstacle_count
        # self.s_start = s_start
        # self.s_goal = s_goal
        # self.Env = env.Env()  # class Env
        # self.u_set = self.Env.motions  # feasible input set
        # self.obs = self.Env.obs  # position of obstacles
        self.converted_target_point = self.Env.converted_target_point
        self.OPEN = []  # priority queue / OPEN set
        self.CLOSED = []  # CLOSED set / VISITED order
        self.PARENT = dict()  # recorded parent
        self.g = dict()  # cost to come
        self.plotter = plotting_test.Plotting(self.s_start, self.s_goal,environment) 
        self.memory_usage = []
        self.memory_usage_peak = 0  # 初始化記憶體高峰值

    def record_memory_usage(self):
        process = psutil.Process(os.getpid())
        current_memory_usage = process.memory_info().rss / 1024 / 1024  # 以MB為單位
        self.memory_usage_peak = max(self.memory_usage_peak, current_memory_usage) 
    def searching(self):
        """
        A_star Searching.
        :return: path, visited order
        """

        self.PARENT[self.s_start] = self.s_start
        self.g[self.s_start] = 0
        self.g[self.s_goal] = math.inf
        heapq.heappush(self.OPEN,
                       (self.f_value(self.s_start), self.s_start))

        while self.OPEN:
            _, s = heapq.heappop(self.OPEN)
            self.CLOSED.append(s)

            if s == self.s_goal:  # stop condition
                break

            for s_n in self.get_neighbor(s):
                new_cost = self.g[s] + self.cost(s, s_n)

                if s_n not in self.g:
                    self.g[s_n] = math.inf

                if new_cost < self.g[s_n]:  # conditions for updating Cost
                    self.g[s_n] = new_cost
                    self.PARENT[s_n] = s
                    heapq.heappush(self.OPEN, (self.f_value(s_n), s_n))

        return self.extract_path(self.PARENT), self.CLOSED

    def searching_repeated_astar(self, e):
        """
        repeated A*.
        :param e: weight of A*
        :return: path and visited order
        """

        path, visited = [], []

        while e >= 1:
            p_k, v_k = self.repeated_searching(self.s_start, self.s_goal, e)
            path.append(p_k)
            visited.append(v_k)
            e -= 0.5

        return path, visited

    def repeated_searching(self, s_start, s_goal, e):
        """
        run A* with weight e.
        :param s_start: starting state
        :param s_goal: goal state
        :param e: weight of a*
        :return: path and visited order.
        """

        g = {s_start: 0, s_goal: float("inf")}
        PARENT = {s_start: s_start}
        OPEN = []
        CLOSED = []
        heapq.heappush(OPEN,
                       (g[s_start] + e * self.heuristic(s_start), s_start))

        while OPEN:
            _, s = heapq.heappop(OPEN)
            CLOSED.append(s)

            if s == s_goal:
                break

            for s_n in self.get_neighbor(s):
                new_cost = g[s] + self.cost(s, s_n)

                if s_n not in g:
                    g[s_n] = math.inf

                if new_cost < g[s_n]:  # conditions for updating Cost
                    g[s_n] = new_cost
                    PARENT[s_n] = s
                    heapq.heappush(OPEN, (g[s_n] + e * self.heuristic(s_n), s_n))

        return self.extract_path(PARENT), CLOSED

    def get_neighbor(self, s):
        """
        find neighbors of state s that not in obstacles.
        :param s: state
        :return: neighbors
        """

        return [(s[0] + u[0], s[1] + u[1]) for u in self.u_set]

    def cost(self, s_start, s_goal):
        """
        Calculate Cost for this motion
        :param s_start: starting node
        :param s_goal: end node
        :return:  Cost for this motion
        :note: Cost function could be more complicate!
        """

        if self.is_collision(s_start, s_goal):
            return math.inf

        return math.hypot(s_goal[0] - s_start[0], s_goal[1] - s_start[1])

    def is_collision(self, s_start, s_end):
        """
        check if the line segment (s_start, s_end) is collision.
        :param s_start: start node
        :param s_end: end node
        :return: True: is collision / False: not collision
        """

        if s_start in self.obs or s_end in self.obs:
            return True

        if s_start[0] != s_end[0] and s_start[1] != s_end[1]:
            if s_end[0] - s_start[0] == s_start[1] - s_end[1]:
                s1 = (min(s_start[0], s_end[0]), min(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
            else:
                s1 = (min(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), min(s_start[1], s_end[1]))

            if s1 in self.obs or s2 in self.obs:
                return True

        return False

    def f_value(self, s):
        """
        f = g + h. (g: Cost to come, h: heuristic value)
        :param s: current state
        :return: f
        """

        return self.g[s] + self.heuristic(s)

    def extract_path(self, PARENT):
        """
        Extract the path based on the PARENT set.
        :return: The planning path
        """

        path = [self.s_goal]
        s = self.s_goal

        while True:
            s = PARENT[s]
            path.append(s)

            if s == self.s_start:
                break

        return list(path)

    def heuristic(self, s):
        """
        Calculate heuristic.
        :param s: current node (state)
        :return: heuristic function value
        """

        heuristic_type = self.heuristic_type  # heuristic type
        goal = self.s_goal  # goal node

        if heuristic_type == "manhattan":
            return abs(goal[0] - s[0]) + abs(goal[1] - s[1])
        else:
            return math.hypot(goal[0] - s[0], goal[1] - s[1])
    def info_output(self,start_time,end_time,path):
        new_data = {
        "地圖大小": [self.Env.get_map_size()],  
        # "障礙物覆蓋率": [self.Env.obstacle_coverage],
        "地圖熵值": [round(self.Env.entropy, 3)],
        "選用路徑規劃算法":["A*"],
        "尺度": [self.scale],
        "路徑長度": [f"{len(path)-1} "],  
        "系統運行時間": [f"{round(end_time - start_time, 3)} 秒"],  
        "記憶體使用量":[f"{round(self.memory_usage_peak, 3)} MB"]
        # "障礙物閥值": [self.min_obstacle_count]
    }

        # 調整列的顯示順序
        columns_order = ["地圖大小","地圖熵值", "選用路徑規劃算法", "尺度","路徑長度", "系統運行時間","記憶體使用量"]
        
        # 將字典轉換成DataFrame
        new_df = pd.DataFrame(new_data)
        file_path = 'algorithm_samemap.xlsx'
        try:

            existing_df = pd.read_excel(file_path)
            # 將新資料附加到現有資料的末尾
            updated_df = pd.concat([existing_df, new_df], ignore_index=True)
        except FileNotFoundError:

            updated_df = new_df
        # 重新排序列
        updated_df.to_excel(file_path, index=False, columns=columns_order)
    def append_memory_usage_peak_to_excel(self):
        memory_df = pd.DataFrame({"記憶體高峰使用量(MB)": [self.memory_usage_peak]})
        file_path = 'memory_usage_astar.xlsx'
        
        if os.path.exists(file_path):
            existing_df = pd.read_excel(file_path)
            updated_df = pd.concat([existing_df, memory_df], ignore_index=True)
        else:
            updated_df = memory_df
        
        updated_df.to_excel(file_path, index=False)
def main(scale):
    start_time = time.time()
    astar = AStar("euclidean",scale=scale)
    astar.record_memory_usage()
    print("障礙物總數:", len(astar.Env.obs))
    # print("障礙物覆蓋率:",astar.Env.obstacle_coverage)
    print("地圖大小:", astar.Env.get_map_size())
    print("選用路徑規劃算法:A*")
    print( "原A尺度出發點與目標點座標",astar.s_start_first, astar.s_goal_first)
    if astar.scale == 'B':
        print("檢驗後B尺度目標點與出發點座標",astar.s_start, astar.s_goal)
    s_start = astar.s_start
    s_goal = astar.s_goal
    environment=astar.Env
    plotter = astar.plotter
    astar.record_memory_usage()
    # if scale == 'A':
    #     plotter.plot_grid("A*,scale A")
    # elif scale == 'B':
    #     plotter.plot_grid("A*,scale B")
    # plot = plotting_test.Plotting(s_start,s_goal,environment)
    path, visited = astar.searching()
    astar.record_memory_usage()
    print("路徑長度",len(path)-1)
    if scale == 'A':
        # plotter.animate_path(path) 
        plotter.animation(path, visited, "A*,scale A")
    elif scale == 'B':
        # plotter.animate_path(path) 
        plotter.animation(path, visited, "A*,scale B")
    # plotter.animate_path(path) 
    # plotter.animation(path, visited, "A*")  # animation
    astar.record_memory_usage()
    end_time = time.time()
    if scale == 'B':
        print("目標點誤差值",astar.converted_target_point)
    print(f"系統運行時間: {end_time - start_time:.3f} seconds") 
    info_output = astar.info_output(start_time,end_time,path)
    # astar.append_memory_usage_peak_to_excel()
    # plt.show()
    # path, visited = astar.searching_repeated_astar(2.5)               # initial weight e = 2.5
    # plot.animation_ara_star(path, visited, "Repeated A*")
def a_star_main():
    main(scale='A')
    plt.show(block=False)
    # # plt.savefig('Astar_A.png',dpi=300, bbox_inches='tight')
    main(scale='B')
    # # plt.savefig('Astar_B.png',dpi=300, bbox_inches='tight')
    plt.show()
    # for _ in range(2):
    #     try:
    #         main(scale='A')
    #         # plt.show(block=False) 
    #         main(scale='B')
    #         # plt.show()
    #         plt.close()
    #     except Exception as e:
    #         print(e)
    #         continue

if __name__ == '__main__':
  a_star_main()