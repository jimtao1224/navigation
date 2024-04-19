"""
D_star_Lite 2D
@author: huiming zhou
"""

import os
import sys
import math
import matplotlib.pyplot as plt
import time
sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search_based_Planning/")

from Search_2D import plotting, env


class DStar:
    def __init__(self, s_start, s_goal, heuristic_type):
        self.s_start, self.s_goal = s_start, s_goal
        self.heuristic_type = heuristic_type
# 行程式碼將傳入的啟發式函數類型設置為類的成員變數 self.heuristic_type。
        self.Env = env.Env()  # class Env
        self.Plot = plotting.Plotting(s_start, s_goal)
# 這兩行程式碼創建了環境和繪圖對象的實例，以便在後續的計算和繪圖中使用。
        self.u_set = self.Env.motions  # feasible input set
        self.obs = self.Env.obs  # position of obstacles
        self.x = self.Env.x_range
        self.y = self.Env.y_range
# 這幾行程式碼從環境中獲取了移動集合、障礙物位置、地圖的寬度和高度等信息，以便後續使用。
        self.g, self.rhs, self.U = {}, {}, {}
        self.km = 0
# 這裡創建了用於存儲節點信息的字典，包括 g 值、rhs 值和 U 值等，並初始化了 km 變數。
        for i in range(1, self.Env.x_range - 1):
            for j in range(1, self.Env.y_range - 1):
                self.rhs[(i, j)] = float("inf")
                self.g[(i, j)] = float("inf")
# 這兩個循環初始化了地圖上每個節點的 rhs 值和 g 值為無窮大。
        self.rhs[self.s_goal] = 0.0
        self.U[self.s_goal] = self.CalculateKey(self.s_goal)
        self.visited = set()
        self.count = 0
        self.fig = plt.figure()
# 這幾行程式碼初始化了終點節點的 rhs 值為0，並將其加入了 U 列表中，
# 同時初始化了一些其他的變數，如訪問過的節點集合、計數器和繪圖的 figure。
        
    def run(self):
        self.Plot.plot_grid("D* Lite")
        self.ComputePath()
        self.plot_path(self.extract_path())
        self.fig.canvas.mpl_connect('button_press_event', self.on_press)
        plt.show()
# self.Plot.plot_grid("D* Lite")：繪製地圖的網格，標題為 "D* Lite"。
# self.ComputePath()：計算起點到終點的最短路徑。
# self.plot_path(self.extract_path())：繪製計算出的最短路徑。
# mpl_connect() 方法用於設置事件處理程序。它接受兩個參數：事件名稱和事件處理函式。
# self.fig.canvas.mpl_connect('button_press_event', self.on_press)：
# 綁定鼠標按下事件到 on_press 方法。
# plt.show()：顯示繪製的圖形。
        
    def on_press(self, event):
        # 用於更新節點 s 的相關值
        x, y = event.xdata, event.ydata
        # 獲取鼠標點擊的座標 (x, y)。
        if x < 0 or x > self.x - 1 or y < 0 or y > self.y - 1:
            print("Please choose right area!")
            # 檢查點擊的座標是否在地圖範圍內，如果不在範圍內則輸出提示信息
        else:
            x, y = int(x), int(y)
            print("Change position: s =", x, ",", "y =", y)
# 將座標 (x, y) 轉換為整數。打印提示信息，表示已更改位置。
            s_curr = self.s_start
            s_last = self.s_start
            i = 0
            path = [self.s_start]
# s_curr = self.s_start：將 s_curr 設置為起點 self.s_start，這是當前節點的初始值。
# s_last = self.s_start：將 s_last 設置為起點 self.s_start，這是上一個節點的初始值。
# i = 0：初始化 i 為0，這是一個計數器，用於跟踪在循環中運行的步數。
# path = [self.s_start]：初始化 path 為包含起點 self.s_start 的列表，
# 這是用於存儲計算出的最短路徑的列表，起始節點已經被添加到路徑中。
            while s_curr != self.s_goal:
# 當前節點 s_curr 不等於目標節點 self.s_goal 時，繼續執行循環。
                s_list = {}
                for s in self.get_neighbor(s_curr):
                    s_list[s] = self.g[s] + self.cost(s_curr, s)
                s_curr = min(s_list, key=s_list.get)
                path.append(s_curr)
# 在每次循環中，首先初始化一個空字典 s_list，
# 用於存儲當前節點的鄰居節點和它們的加權值（g 值加上從當前節點到鄰居節點的代價）。
# 然後找到具有最小加權值的鄰居節點，並將其設置為新的當前節點 s_curr。
# 最後，將新的當前節點添加到路徑列表中。
                if i < 1:
                    self.km += self.h(s_last, s_curr)
                    s_last = s_curr
# 將 self.km 增加到 s_last 和 s_curr 之間的啟發式距離。
# 這部分程式碼估計了起始節點到當前節點的最短路徑長度
# 將上一個節點 s_last 設置為當前節點 s_curr。                    
                    if (x, y) not in self.obs:
                        self.obs.add((x, y))
                        plt.plot(x, y, 'sk')
                        self.g[(x, y)] = float("inf")
                        self.rhs[(x, y)] = float("inf")
                    else:
                        self.obs.remove((x, y))
                        plt.plot(x, y, marker='s', color='white')
                        self.UpdateVertex((x, y))
# 檢查點 (x, y) 是否在障礙物集合中。如果不在，則將其添加到障礙物集合中並以黑色 'sk' 標記，
# 並將 self.g[(x, y)] 和 self.rhs[(x, y)] 初始化為無窮大值。
# 如果在障礙物集合中，則將其從集合中移除並以白色 's' 標記，然後更新與該節點相鄰的節點。
                    for s in self.get_neighbor((x, y)):
                        self.UpdateVertex(s)
# 對點 (x, y) 的鄰居節點執行 UpdateVertex 操作。這將更新與 (x, y) 相鄰的節點的值。
                    i += 1
                    self.count += 1
# 增加計數器 i 的值，並增加 self.count 的值。self.count 用於追踪訪問的節點數量。
                    self.visited = set()
                    self.ComputePath()
# 重置已訪問的節點集合，然後重新計算最短路徑。這是為了在更新障礙物位置後，更新最短路徑。
            # self.plot_visited(self.visited)
            self.plot_path(path)
            self.fig.canvas.draw_idle()
# 在每次迴圈結束時，將訪問過的節點繪製在圖形上，並繪製計算出的最短路徑。
# 然後刷新圖形，以使更新的內容顯示在圖形中。
            
    def ComputePath(self):
        # 用於計算最短路徑。
        while True:
            s, v = self.TopKey()
            # print(self.g[self.s_start])
            # print(self.CalculateKey(self.s_start))
# 使用 TopKey 方法找到優先級隊列中最小鍵值對應的節點 s 和其對應的鍵值 v。            
            if v >= self.CalculateKey(self.s_start) and \
                    self.rhs[self.s_start] == self.g[self.s_start]:
                break
# 檢查是否滿足終止條件。該條件檢查起點 self.s_start 的鍵值是否大於或等於起點的計算鍵值，
# 並且起點的右手值是否等於左手值。
            k_old = v
# 將當前節點 s 的鍵值保存為 k_old。
            self.U.pop(s)
            self.visited.add(s)
# 從優先級隊列 U 中移除節點 s。
# 將節點 s 添加到已訪問的集合中，以避免重複訪問。
            if k_old < self.CalculateKey(s):
                self.U[s] = self.CalculateKey(s)
# 如果 k_old 小於節點 s 的新計算鍵值，則將節點 s 添加到優先級隊列 U 中，並更新其鍵值。                
            elif self.g[s] > self.rhs[s]:
                self.g[s] = self.rhs[s]
                for x in self.get_neighbor(s):
                    self.UpdateVertex(x)
# 如果節點 s 的左手值 g 大於右手值 rhs，則將 g 更新為 rhs，並更新所有與 s 相鄰的節點的信息。                    
            else:
                self.g[s] = float("inf")
                self.UpdateVertex(s)
                for x in self.get_neighbor(s):
                    self.UpdateVertex(x)
# 否則，將 g 設置為無窮大，並更新所有與 s 相鄰的節點的信息。

    def UpdateVertex(self, s):
        # 用於D* Lite演算法中的節點更新、計算優先級鍵值和計算啟發式估值
        if s != self.s_goal:
# 確保要更新的節點不是目標節點。因為目標節點的右手值（rhs）始終為0，不需要進行更新。
            self.rhs[s] = float("inf")
# 將節點 s 的右手值初始化為無窮大。這是為了確保在後續運算中可以正確更新右手值。
            for x in self.get_neighbor(s):
# 迭代遍歷節點 s 的鄰居節點 x。                
                self.rhs[s] = min(self.rhs[s], self.g[x] + self.cost(s, x))
# 對於每個鄰居節點 x，計算通過節點 s 到達目標節點的最小成本，
# 並更新節點 s 的右手值為其自身右手值和通過節點 x 到目標節點的成本的最小值。                
        if s in self.U:
            self.U.pop(s)
# 檢查節點 s 是否在優先級隊列 U 中。如果節點 s 在優先級隊列 U 中，則將其從隊列中移除。
        if self.g[s] != self.rhs[s]:
            self.U[s] = self.CalculateKey(s)
# 檢查節點 s 的左手值 g 是否等於右手值 rhs
# 如果不相等，則計算並更新節點 s 的優先級鍵值，並將其添加到優先級隊列 U 中。
            
    def CalculateKey(self, s):
        # 用於計算節點 s 的優先級鍵值
        return [min(self.g[s], self.rhs[s]) + self.h(self.s_start, s) + self.km,
                min(self.g[s], self.rhs[s])]

    
# min(self.g[s], self.rhs[s]): 計算節點 s 的左手值 g 和右手值 rhs 的最小值。
    # 這代表了節點 s 到目標節點的最小成本。

# self.h(self.s_start, s): 計算節點 s 的啟發式估值。
    # 這是從起點 self.s_start 到節點 s 的預估最短路徑成本，通常是曼哈頓距離或歐氏距離。

# self.km: 修正因子，用於處理障礙物動態更新的情況。

# min(self.g[s], self.rhs[s]) + self.h(self.s_start, s) + self.km: 
    # 將上述三個值相加，得到節點 s 的最終優先級鍵值。

# 方法返回一個包含兩個元素的列表，第一個元素是上述計算的優先級鍵值，
    #  第二個元素是 g 和 rhs 的最小值。

    def TopKey(self):
        # 從優先級隊列中獲取最小的鍵值對應的節點和其對應的鍵值。
        """
        :return: return the min key and its value.
        """

        s = min(self.U, key=self.U.get)
        # print(s,"s")
        # print(self.U[s])
        return s, self.U[s]
# s = min(self.U, key=self.U.get): 使用 min 函數和 key 參數來找到
# 優先級隊列 U 中具有最小值的鍵值對應的節點 s。這將返回 U 中鍵值最小的鍵-值對。

# self.U[s]: 從優先級隊列中通過節點 s 獲取其對應的值。這個值是節點 s 的優先級鍵值。

# 方法返回一個包含兩個元素的元組，第一個元素是最小鍵值對應的節點 s，第二個元素是對應的鍵值。
    
    def h(self, s_start, s_goal):
 # 用於計算啟發式估值的。啟發式估值用於估算從起點 s_start 到目標節點 s_goal 的最短路徑成本
        heuristic_type = self.heuristic_type  # heuristic type
# 中獲取啟發式類型。
        if heuristic_type == "manhattan":
            return abs(s_goal[0] - s_start[0]) + abs(s_goal[1] - s_start[1])
# 檢查啟發式類型是否為曼哈頓距離。
# 如果是曼哈頓距離，則計算兩個節點之間的曼哈頓距離。
# 曼哈頓距離是兩點之間水平和垂直方向上的距離之和。
        else:
            return math.hypot(s_goal[0] - s_start[0], s_goal[1] - s_start[1])
# 則計算兩個節點之間的歐幾里得距離。歐氏距離是兩點之間的直線距離，可以通過應用勾股定理計算。

    def cost(self, s_start, s_goal):
        # 用於計算從起始節點 s_start 移動到目標節點 s_goal 的成本
        """
        Calculate Cost for this motion
        :param s_start: starting nodeget_neighbor
        :param s_goal: end node
        :return:  Cost for this motion
        :note: Cost function could be more complicate!
        """

        if self.is_collision(s_start, s_goal):
            return float("inf")
# 檢查移動是否與障礙物碰撞。
# 如果移動與障礙物碰撞，則返回無窮大的成本。這表明這條移動路徑是不可行的。
        return math.hypot(s_goal[0] - s_start[0], s_goal[1] - s_start[1])
# 如果移動不與障礙物碰撞，則計算從起始節點到目標節點的歐氏距離作為成本。
# 這個距離通常用來衡量兩個點之間的直線距離，是一種常見的成本計算方式。
    
    def is_collision(self, s_start, s_end):
        # 用於檢查移動路徑是否與障礙物碰撞
        if s_start in self.obs or s_end in self.obs:
            return True
# 檢查起始節點 s_start 或目標節點 s_end 是否在障礙物集合 self.obs 中。
# 如果起始節點或目標節點任一個在障礙物中，就認為有碰撞。
        if s_start[0] != s_end[0] and s_start[1] != s_end[1]:
# 檢查起始節點和目標節點是否不在同一直線上。如果不在同一直線上，
# 表示移動路徑是一條斜線路徑，需要進一步檢查是否有對角線上的障礙物。
            if s_end[0] - s_start[0] == s_start[1] - s_end[1]:
                s1 = (min(s_start[0], s_end[0]), min(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
            else:
                s1 = (min(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), min(s_start[1], s_end[1]))
# 根據對角線的方向，計算對角線上的兩個點 s1 和 s2。其中 s1 是對角線的起始點，s2 是對角線的終點。
            if s1 in self.obs or s2 in self.obs:
                return True
# 檢查對角線上的兩個點是否有障礙物。如果其中任一個點在障礙物集合中，表示對角線上有障礙物，存在碰撞。
        return False
# 如果以上條件都不滿足，則返回 False，表示移動路徑沒有與障礙物碰撞。

    def get_neighbor(self, s):
        # 用於獲取節點 s 的鄰居節點集合，這些鄰居節點是通過可行的移動方向從節點 s 可以到達的。
        nei_list = set()
# 空的集合來存儲鄰居節點。
        for u in self.u_set:
# 迭代遍歷可行的移動方向列表 self.u_set 中的每個移動方向 u。
            s_next = tuple([s[i] + u[i] for i in range(2)])
# 通過將節點 s 的坐標與移動方向 u 的對應坐標相加，
# 計算出鄰居節點 s_next 的坐標。這將產生一個新的鄰居節點。            
            if s_next not in self.obs:
                nei_list.add(s_next)
# 檢查鄰居節點 s_next 是否在障礙物集合 self.obs中。
# 如果不在障礙物中，表示這個鄰居節點是可行的,將可行的鄰居節點添加到鄰居節點集合中。
        return nei_list

    def extract_path(self):
        """
        Extract the path based on the PARENT set.
        :return: The planning path
        """
        # 從父節點集合中提取規劃路徑
        path = [self.s_start]
        s = self.s_start
# 將起始節點 self.s_start 添加到列表中作為起點。
        for k in range(1000):
            g_list = {}
# 創建一個空的字典 g_list 來存儲可行鄰居節點的成本。這個字典將用於找到下一個最優節點。
            for x in self.get_neighbor(s):
# 對於節點 s 的每個鄰居節點 x，檢查是否有與鄰居節點 x 的移動路徑碰撞。
                if not self.is_collision(s, x):
                    g_list[x] = self.g[x]
            s = min(g_list, key=g_list.get)
# 如果移動路徑不與障礙物碰撞，則將鄰居節點 x 添加到 g_list 中，其成本為 self.g[x]。
            path.append(s)
            if s == self.s_goal:
                break
# 如果選擇的節點 s 等於目標節點 self.s_goal，則跳出迴圈。
        return list(path)

    def plot_path(self, path):
        # 用於在地圖上繪製規劃好的路徑
        px = [x[0] for x in path]
        py = [x[1] for x in path]
        plt.plot(px, py, linewidth=2)
        plt.plot(self.s_start[0], self.s_start[1], "bs")
        plt.plot(self.s_goal[0], self.s_goal[1], "gs")
# 從給定的路徑 path 中提取所有節點的 x 坐標，並將它們存儲在列表 px 中。

# 從給定的路徑 path 中提取所有節點的 y 坐標，並將它們存儲在列表 py 中。

# 使用 plt.plot() 函數將路徑上的所有節點連接起來，這將產生一條表示規劃路徑的折線。

# 使用 plt.plot() 函數在地圖上繪製起始節點，這裡使用藍色方塊表示起點。

# 使用 plt.plot() 函數在地圖上繪製目標節點，這裡使用綠色方塊表示終點。
    def plot_visited(self, visited):
        #  用於在地圖上繪製訪問過的節點。
        color = ['gainsboro', 'lightgray', 'silver', 'darkgray',
                 'bisque', 'navajowhite', 'moccasin', 'wheat',
                 'powderblue', 'skyblue', 'lightskyblue', 'cornflowerblue']

        if self.count >= len(color) - 1:
            self.count = 0

        for x in visited:
            plt.plot(x[0], x[1], marker='s', color=color[self.count])
# 創建了一個顏色列表 color，其中包含了用於繪製不同訪問次數的節點的顏色。

# 如果計數器 self.count 的值大於等於顏色列表 color 的長度減1，則將計數器重置為0，
# 以循環使用顏色列表中的顏色。

# 遍歷訪問過的節點集合 visited 中的每個節點。

# 使用 plt.plot() 函數在地圖上繪製每個訪問過的節點。節點的顏色通過從顏色列表中獲取的顏色來決定，
# 並且使用正方形標記。

def main():
    s_start = (10, 5)
    s_goal = (90, 50)
    dstar = DStar(s_start, s_goal, "euclidean")
    # 創建了一個 DStar 物件 dstar，
    # 並將起始節點、目標節點以及啟發式函數類型 "euclidean" 傳遞給它。
    dstar.run()
 # Print time taken for map generation

if __name__ == '__main__':
    main()
