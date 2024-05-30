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
import pandas as pd
import psutil
sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search_based_Planning/")

from Search_2D import plotting_test, env_test
from variable_set import Environment
from env_test import Env
class DStar:
    def __init__(self, heuristic_type,scale):
        environment = Env(scale=scale)
        self.Env = environment
        scale = self.Env.scale
        self.scale = scale
        self.u_set = self.Env.motions  # feasible input set
        self.Env.obs = self.Env.obs
        self.s_start = self.Env.start_point
        self.s_goal = self.Env.end_point
        self.s_start_first = self.Env.s_start_first
        self.s_goal_first = self.Env.s_goal_first
        self.converted_target_point =self.Env.converted_target_point
        self.entropy = self.Env.entropy
        # if scale == 'A':
        #     self.s_start = self.s_start_first
        #     self.s_goal = self.s_goal_first
        #     self.s_start = self.adjust_for_obstacles(self.s_start, self.Env.obs, "出發點")
        #     self.s_goal = self.adjust_for_obstacles(self.s_goal, self.Env.obs, "目標點")
        # elif scale == 'B':
        #     self.s_start = self.convert_to_B_scale(self.s_start_first)
        #     self.s_goal = self.convert_to_B_scale(self.s_goal_first)
        #     print(self.s_start, self.s_goal, "B尺度各點座標,尚未檢驗")
        #     self.s_start = self.adjust_coordinates(self.s_start, self.Env.x_range_B, self.Env.y_range_B, "出發點")
        #     self.s_goal = self.adjust_coordinates(self.s_goal, self.Env.x_range_B, self.Env.y_range_B, "目標點")
        #     self.s_start = self.adjust_for_obstacles(self.s_start, self.Env.obs, "出發點")
        #     self.s_goal = self.adjust_for_obstacles(self.s_goal, self.Env.obs, "目標點")
        # self.converted_target_point = self.calculate_target_displacement(self.s_goal_first, self.s_goal)
        self.plotter = plotting_test.Plotting(self.s_start, self.s_goal,environment) 
        # self.fig = plt.figure()
        self.heuristic_type = heuristic_type
        self.x, self.y = self.set_scale(scale)
        self.g, self.rhs, self.U = {}, {}, {}
        self.km = 0
        self.count = 0
        for i in range(1, self.x - 1):
            for j in range(1, self.y - 1):
                self.rhs[(i, j)] = float("inf")
                self.g[(i, j)] = float("inf")
        self.rhs[self.s_goal] = 0.0
        self.U[self.s_goal] = self.CalculateKey(self.s_goal)
        self.visited = set()
        self.memory_usage = []
        self.memory_usage_peak = 0  # 初始化記憶體高峰值
    
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
    def record_memory_usage(self):
        process = psutil.Process(os.getpid())
        current_memory_usage = process.memory_info().rss / 1024 / 1024  # 以MB為單位
        self.memory_usage_peak = max(self.memory_usage_peak, current_memory_usage) 
    def run(self):
        start_time = time.time()
        self.record_memory_usage()
        if self.scale == 'A':
            self.plotter.plot_grid("D* Lite_A")
        elif self.scale == 'B':
            self.plotter.plot_grid("D* Lite_B")
        self.record_memory_usage()
        print("障礙物總數:", len(self.Env.obs))
        # print("障礙物覆蓋率:",self.Env.obstacle_coverage)
        print("地圖大小:", self.Env.get_map_size())
        print("選用路徑規劃算法:D* Lite")
        print( "原A尺度出發點與目標點座標",self.s_start_first, self.s_goal_first)
        if self.scale == 'B':
            print("檢驗後B尺度目標點與出發點座標",self.s_start, self.s_goal)
        # print("ComputePath")
        # print("障礙物",self.Env.obs)
        self.ComputePath()
        self.record_memory_usage()
        # print("plot_path")
        self.plotter.plot_path(self.extract_path())    
        # print("animate_path")
        # self.plotter.animate_path(self.extract_path())
        print("路徑長度",len(self.extract_path())-1)
        end_time = time.time()
        # info_output = self.info_output(start_time,end_time)
        if self.scale == 'B':
            print("目標點誤差值",self.converted_target_point)
        print(f"系統運行時間: {end_time - start_time:.3f} seconds")
        self.plotter.fig.canvas.mpl_connect('button_press_event', self.on_press)
        self.record_memory_usage()  # 記錄高峰值

        # 將記憶體使用高峰值附加到excel文件中
        self.append_memory_usage_peak_to_excel()
        # self.fig.canvas.mpl_connect('button_press_event', self.on_press)
        # plt.show()
        # plt.close()

    def append_memory_usage_peak_to_excel(self):
        memory_df = pd.DataFrame({"記憶體高峰使用量(MB)": [self.memory_usage_peak]})
        file_path = 'memory_usage.xlsx'
        
        if os.path.exists(file_path):
            existing_df = pd.read_excel(file_path)
            updated_df = pd.concat([existing_df, memory_df], ignore_index=True)
        else:
            updated_df = memory_df
        
        updated_df.to_excel(file_path, index=False)

    def on_press(self, event):
        x, y = event.xdata, event.ydata
        if x < 0 or x > self.x - 1 or y < 0 or y > self.y - 1:
            print("Please choose right area!")
        else:
            x, y = int(x), int(y)
            print("Change position: s =", x, ",", "y =", y)

            s_curr = self.s_start
            s_last = self.s_start
            i = 0
            path = [self.s_start]

            while s_curr != self.s_goal:
                s_list = {}

                for s in self.get_neighbor(s_curr):
                    s_list[s] = self.g[s] + self.cost(s_curr, s)
                s_curr = min(s_list, key=s_list.get)
                path.append(s_curr)

                if i < 1:
                    self.km += self.h(s_last, s_curr)
                    s_last = s_curr
                    if (x, y) not in self.Env.obs:
                        self.Env.obs.add((x, y))
                        plt.plot(x, y, 'sk')
                        self.g[(x, y)] = float("inf")
                        self.rhs[(x, y)] = float("inf")
                    else:
                        self.Env.obs.remove((x, y))
                        plt.plot(x, y, marker='s', color='white')
                        self.UpdateVertex((x, y))
                    for s in self.get_neighbor((x, y)):
                        self.UpdateVertex(s)
                    i += 1

                    self.count += 1
                    self.visited = set()
                    self.ComputePath()

            self.plotter.plot_path(path)
            self.plotter.fig.canvas.draw_idle()       

    def ComputePath(self):
        while True:
            s, v = self.TopKey()  
            if v >=self.CalculateKey(self.s_start) and \
                    self.rhs[self.s_start] == self.g[self.s_start]:
                break  
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
            #     if x not in self.g:
            #         continue  # 跳过当前的邻居节点，继续下一个
                self.rhs[s] = min(self.rhs[s], self.g[x] + self.cost(s, x))
        if s in self.U:
            self.U.pop(s)
        # if s not in self.g:
        #     self.g[s] = float('inf')  # 为不存在的键赋予默认值
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
        # distance = self.calculate_distance(s, self.s_start)
        for u in self.u_set:
            s_next = tuple([s[i] + u[i]   for i in range(2)])       
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
    # def convert_to_B_scale(self, a_scale_point):
    #     # 計算A尺度到B尺度的轉換比例
    #     x_ratio = self.Env.x_range_B / self.Env.x_range_A
    #     y_ratio = self.Env.y_range_B / self.Env.y_range_A
    #     # 按比例轉換A尺度點到B尺度
    #     b_scale_approx = (a_scale_point[0] * x_ratio, a_scale_point[1] * y_ratio)
    #     offset_x = b_scale_approx[0] - int(b_scale_approx[0])
    #     offset_y = b_scale_approx[1] - int(b_scale_approx[1])
    #     if offset_x > 0.5:
    #         b_scale_x = min(self.Env.x_range_B - 1, int(b_scale_approx[0]) + 1)
    #     else:
    #         b_scale_x = max(1, int(b_scale_approx[0]))

    #     if offset_y > 0.5:
    #         b_scale_y = min(self.Env.y_range_B - 1, int(b_scale_approx[1]) + 1)
    #     else:
    #         b_scale_y = max(1, int(b_scale_approx[1]))
    #     b_scale_point = (b_scale_x, b_scale_y)
    #     return b_scale_point
    # def calculate_target_displacement(self,original_target_point, converted_target_point):
    # # 计算原始目标点和转换后目标点之间的欧几里得距离
    #     x_ratio = self.Env.x_range_B / self.Env.x_range_A
    #     y_ratio = self.Env.y_range_B / self.Env.y_range_A
    #     original_target_point = (original_target_point[0] * x_ratio, original_target_point[1] * y_ratio)
    #     displacement = ((converted_target_point[0] - original_target_point[0])**2 + (converted_target_point[1] - original_target_point[1])**2)**0.5
    #     return displacement
    # def adjust_coordinates(self,coord, x_range, y_range,point_name):
    #         x, y = coord
    #         if x >= x_range - 1:
    #             print(f"座標轉換後{point_name}x軸位置錯誤,調整座標")
    #             x = x_range - 2  # 减少一个单位而不是减到-1
    #         if y >= y_range - 1:
    #             print(f"座標轉換後{point_name}y軸位置錯誤,調整座標")
    #             y = y_range - 2
    #         return x, y
    # def adjust_for_obstacles(self,coord, obs, point_name):
    #         if coord in obs:
    #             print(f"{point_name}與障礙物重疊，正在尋找新的位置...")
    #             new_coord = (coord[0] + 1, coord[1] + 1)
    #             # 检查新位置是否仍然重叠，如果是，则再次调整（在实际应用中，可能需要更复杂的处理方式）
    #             while new_coord in obs:
    #                 new_coord = (new_coord[0] + 1, new_coord[1] + 1)
    #             print(f"{point_name}已移至新位置：{new_coord}")
    #             return new_coord
    #         return coord  # 如果没有重叠，就返回原坐标
    def info_output(self,start_time,end_time):
        new_data = {
        "地圖大小": [self.Env.get_map_size()],  
        "障礙物總數": [len(self.Env.obs)],
        # "障礙物覆蓋率": [self.Env.obstacle_coverage],
        "地圖熵值": [round(self.Env.entropy, 3)],
        "選用路徑規劃算法":["D* Lite"],
        "尺度": [self.scale],
        "路徑長度": [f"{len(self.extract_path())-1} "],  
        "系統運行時間": [f"{round(end_time - start_time, 3)} 秒"],  
        "目標點誤差值": [round(self.converted_target_point, 3)]
    }

        # 調整列的顯示順序
        columns_order = ["地圖大小","障礙物總數", "地圖熵值", "選用路徑規劃算法", "尺度","路徑長度", "系統運行時間","目標點誤差值"]
      

        
        # 將字典轉換成DataFrame
        new_df = pd.DataFrame(new_data)
        file_path = 'dstar_lite_隨機.xlsx'
        try:

            existing_df = pd.read_excel(file_path)
            # 將新資料附加到現有資料的末尾
            updated_df = pd.concat([existing_df, new_df], ignore_index=True)
        except FileNotFoundError:

            updated_df = new_df
        # 重新排序列
        updated_df.to_excel(file_path, index=False, columns=columns_order)
        # df = df[columns_order]


def d_star_main():
    # dstar = DStar("euclidean",scale='B') 
    # dstar.run()
    # plt.show(block=False)  # 显示当前图形
    # plt.pause(5)
    # plt.close()
    # dstar = DStar("euclidean",scale='A') 
    # dstar.run()
    # plt.show()
    # 分開顯示
    for _ in range(20): 
        try:   
            dstar = DStar("euclidean",scale='A') 
            dstar.run()
            plt.show(block=False)  
            dstar = DStar("euclidean",scale='B') 
            dstar.run()
            plt.close()
        except Exception as e:
            print(e)
            continue
            # plt.show()

            # dstar = DStar("euclidean",scale='A') 
            # dstar.run()
            # # plt.savefig('Dstar_A2.png',dpi=300,bbox_inches='tight')
            # plt.show(block=False) 
            # dstar = DStar("euclidean",scale='B') 
            # dstar.run()
            # # plt.savefig('Dstar_B.png',dpi=300,bbox_inches='tight')
            # plt.show()
            # 最後同步顯示
if __name__ == '__main__':
    d_star_main()
