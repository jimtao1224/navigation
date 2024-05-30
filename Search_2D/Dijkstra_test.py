"""
Dijkstra 2D
@author: huiming zhou
"""

import os
import sys
import math
import heapq
import time
import matplotlib.pyplot as plt
import pandas as pd
import psutil
sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search_based_Planning/")

from Search_2D import plotting_test, env_test

from Search_2D.Astar_test import AStar


class Dijkstra(AStar):
    """Dijkstra set the cost as the priority 
    """
    def searching(self):
        """
        Breadth-first Searching.
        :return: path, visited order
        """

        self.PARENT[self.s_start] = self.s_start
        self.g[self.s_start] = 0
        self.g[self.s_goal] = math.inf
        heapq.heappush(self.OPEN,
                       (0, self.s_start))

        while self.OPEN:
            _, s = heapq.heappop(self.OPEN)
            self.CLOSED.append(s)

            if s == self.s_goal:
                break

            for s_n in self.get_neighbor(s):
                new_cost = self.g[s] + self.cost(s, s_n)

                if s_n not in self.g:
                    self.g[s_n] = math.inf

                if new_cost < self.g[s_n]:  # conditions for updating Cost
                    self.g[s_n] = new_cost
                    self.PARENT[s_n] = s

                    # best first set the heuristics as the priority 
                    heapq.heappush(self.OPEN, (new_cost, s_n))

        return self.extract_path(self.PARENT), self.CLOSED
    def info_output_Dijkstra(self,start_time,end_time,path):
        new_data = {
        "地圖大小": [self.Env.get_map_size()],  
        "障礙物總數": [len(self.Env.obs)],
        # "障礙物覆蓋率": [self.Env.obstacle_coverage],
 
        "地圖熵值": [round(self.Env.entropy, 3)],
        "選用路徑規劃算法":["dijkstra"],
        "尺度": [self.scale],
        "路徑長度": [f"{len(path)-1} "],  
        "系統運行時間": [f"{round(end_time - start_time, 3)} 秒"],  
        "目標點誤差值": [round(self.converted_target_point, 3)]
    }

        # 調整列的顯示順序
        columns_order = ["地圖大小","障礙物總數", "地圖熵值", "選用路徑規劃算法", "尺度","路徑長度", "系統運行時間","目標點誤差值"]
         
        # 將字典轉換成DataFrame
        new_df = pd.DataFrame(new_data)
        file_path = 'dijkstra_隨機.xlsx'
        try:

            existing_df = pd.read_excel(file_path)
            # 將新資料附加到現有資料的末尾
            updated_df = pd.concat([existing_df, new_df], ignore_index=True)
        except FileNotFoundError:

            updated_df = new_df
        # 重新排序列
        updated_df.to_excel(file_path, index=False, columns=columns_order)
    def append_memory_usage_peak_to_excel_2(self):
        memory_df = pd.DataFrame({"記憶體高峰使用量(MB)": [self.memory_usage_peak]})
        file_path = 'memory_usage_dijkstra.xlsx'
        
        if os.path.exists(file_path):
            existing_df = pd.read_excel(file_path)
            updated_df = pd.concat([existing_df, memory_df], ignore_index=True)
        else:
            updated_df = memory_df
        
        updated_df.to_excel(file_path, index=False)

def main(scale):
    # s_start = (5, 5)
    # s_goal = (45, 25)
    # start_time = time.time()
    # astar = AStar("euclidean",scale=scale)
    # print("障礙物總數:", len(astar.Env.obs))
    # print("障礙物覆蓋率:",astar.Env.obstacle_coverage)
    # print("地圖大小:", astar.Env.get_map_size())
    # print( "原A尺度出發點與目標點座標",astar.s_start_first, astar.s_goal_first)
    # if astar.scale == 'B':
    #     print("檢驗後B尺度目標點與出發點座標",astar.s_start, astar.s_goal)
    # s_start = astar.s_start
    # s_goal = astar.s_goal
    # environment=astar.Env
    # plot = plotting_test.Plotting(s_start,s_goal,environment)
    # path, visited = astar.searching()
    # print("路徑長度",len(path)-1)
    # plot.animation(path, visited, "A*")  # animation
    # end_time = time.time()

    # print("目標點誤差值",astar.converted_target_point)  
    # print("系統運行時間:", end_time - start_time, "seconds") 

    start_time = time.time()
    dijkstra = Dijkstra('None',scale)
    dijkstra.record_memory_usage()
    print("障礙物總數:", len(dijkstra.Env.obs))
    # print("障礙物覆蓋率:",dijkstra.Env.obstacle_coverage)
    print("地圖大小:", dijkstra.Env.get_map_size())
    print("選用路徑規劃算法:dijkstra")
    print( "原A尺度出發點與目標點座標",dijkstra.s_start_first, dijkstra.s_goal_first)
    if dijkstra.scale == 'B':
        print("檢驗後B尺度目標點與出發點座標",dijkstra.s_start, dijkstra.s_goal)
    s_start = dijkstra.s_start
    s_goal = dijkstra.s_goal
    plotter = dijkstra.plotter
    dijkstra.record_memory_usage()
    path, visited = dijkstra.searching()
    dijkstra.record_memory_usage()
    print("路徑長度",len(path)-1)
    if scale == 'A':
        plotter.plot_grid("Dijkstra's,scale A")
        plotter.animation(path, visited, "Dijkstra's,scale A")
    elif scale == 'B':
        plotter.plot_grid("Dijkstra's,scale B")
        plotter.animation(path, visited, "Dijkstra's,scale B")
    dijkstra.record_memory_usage()
    # plotter.plot_grid("Dijkstra's")
    # plotter.animation(path, visited, "Dijkstra's")
    end_time = time.time()
    if scale == 'B':
        print("目標點誤差值",dijkstra.converted_target_point)
    print(f"系統運行時間: {end_time - start_time:.3f} seconds")
    dijkstra.info_output_Dijkstra(start_time,end_time,path)
    dijkstra.record_memory_usage()
    dijkstra.append_memory_usage_peak_to_excel_2()
    # plot.animation(path, visited, "Dijkstra's")  # animation generate
    
def Dijkstra_main():
    # main(scale='A')
    # # plt.savefig('Dijkstar_A.png',dpi=300, bbox_inches='tight')
    # plt.show(block=False)
    # main(scale='B')
    # # plt.savefig('Dijkstar_B.png',dpi=1000, bbox_inches='tight')
    # plt.show()
    for _ in range(20):
            try:
                main(scale='A')
                plt.savefig('Dijkstar_A.png',dpi=1000)
                # plt.show(block=False)  
                main(scale='B')
                plt.savefig('Dijkstar_B.png',dpi=1000)
                # plt.show()
                plt.close()
            except Exception as e:
                print(e)
                continue

if __name__ == '__main__':
 Dijkstra_main()