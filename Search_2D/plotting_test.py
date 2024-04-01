"""
Plot tools 2D
@author: huiming zhou
"""
import time
import os
import sys
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search_based_Planning/")

from Search_2D import env_test


class Plotting:
    def __init__(self, xI, xG,env):
        scale = env_test.Env().scale
        self.scale = scale
        self.Env = env_test.Env()  # class Env
        self.xI, self.xG = xI, xG
        self.env = env 
        self.x_range, self.y_range = self.set_scale(scale)
        self.obs = self.env.obs_map()
        self.fig, self.ax = plt.subplots()
    def update_obs(self, obs):
        self.obs = obs

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
        
    # def animation(self, path, visited, name):
    #     self.plot_grid(name)
    #     # self.plot_visited(visited)
    #     # self.plot_path(path)
    #     plt.show()

    # def animation_lrta(self, path, visited, name):
    #     self.plot_grid(name)
    #     cl = self.color_list_2()
    #     path_combine = []

    #     for k in range(len(path)):
    #         self.plot_visited(visited[k], cl[k])
    #         plt.pause(0.2)
    #         self.plot_path(path[k])
    #         path_combine += path[k]
    #         plt.pause(0.2)
    #     if self.xI in path_combine:
    #         path_combine.remove(self.xI)
    #     self.plot_path(path_combine)
    #     plt.show()

    # def animation_ara_star(self, path, visited, name):
    #     self.plot_grid(name)
    #     cl_v, cl_p = self.color_list()

    #     for k in range(len(path)):
    #         self.plot_visited(visited[k], cl_v[k])
    #         self.plot_path(path[k], cl_p[k], True)
    #         plt.pause(0.5)

    #     plt.show()

    # def animation_bi_astar(self, path, v_fore, v_back, name):
    #     self.plot_grid(name)
    #     self.plot_visited_bi(v_fore, v_back)
    #     self.plot_path(path)
    #     plt.show()

    def plot_grid(self, name):
        obs_x = [x[0] for x in self.obs]
        obs_y = [x[1] for x in self.obs]

        plt.plot(self.xI[0], self.xI[1], "bs")
        plt.plot(self.xG[0], self.xG[1], "gs")
        plt.plot(obs_x, obs_y, "sk")
        # self.plot_scale_b_borders()
        plt.title(name)
        plt.axis("equal")

    # def plot_visited(self, visited, cl='gray'):
    #     if self.xI in visited:
    #         visited.remove(self.xI)

    #     if self.xG in visited:
    #         visited.remove(self.xG)

    #     count = 0

    #     for x in visited:
    #         count += 1
    #         plt.plot(x[0], x[1], color=cl, marker='o')
    #         plt.gcf().canvas.mpl_connect('key_release_event',
    #                                      lambda event: [exit(0) if event.key == 'escape' else None])

    #         if count < len(visited) / 3:
    #             length = 20
    #         elif count < len(visited) * 2 / 3:
    #             length = 30
    #         else:
    #             length = 40
    #         #
    #         # length = 15

    #         if count % length == 0:
    #             plt.pause(0.001)
    #     plt.pause(0.01)

    def plot_path(self, path, cl='r'):
        path_x = [path[i][0] for i in range(len(path))]
        path_y = [path[i][1] for i in range(len(path))]
        # print(path_x, path_y, "path_x, path_y")
        self.ax.set_xlim(0, self.x_range - 1)
        self.ax.set_ylim(0, self.y_range - 1)
        # print(self.env.x_range_A,"self.env.x_range_A")
    
        plt.plot(path_x, path_y, linewidth='1', color='b')

        
        plt.plot(self.xI[0], self.xI[1], "bs")
        plt.plot(self.xG[0], self.xG[1], "gs")
        # print(self.xI[0], self.xI[1],
        #       self.xG[0], self.xG[1], "plot_path")
        # plt.pause(0.01)
    
    def animate_path(self, path):
        x_data, y_data = [], []
        self.ax.set_xlim(0, self.x_range - 1)
        self.ax.set_ylim(0, self.y_range - 1)
        line, = plt.plot([], [], 'yo-', animated=True)
        point, = plt.plot([], [], 'y-', animated=True)
        total_time = len(path) - 1  # 总时间等于路径长度减一（因为从第0秒开始）
        def init():
            self.ax.set_xlim(0, self.x_range - 1)
            self.ax.set_ylim(0, self.y_range - 1)
            return line, point

        def update(frame):
            x_data.append(path[frame][0])
            y_data.append(path[frame][1])
            line.set_data(x_data[-1], y_data[-1])
            point.set_data(x_data, y_data)
            # print(x_data, y_data)
            return line, point

        ani = animation.FuncAnimation(self.fig, update, frames=range(len(path)),
                                      init_func=init, blit=True, repeat=False,interval=100)
        # plt.close()
    # def plot_visited_bi(self, v_fore, v_back):
    #     if self.xI in v_fore:
    #         v_fore.remove(self.xI)

    #     if self.xG in v_back:
    #         v_back.remove(self.xG)

    #     len_fore, len_back = len(v_fore), len(v_back)

    #     for k in range(max(len_fore, len_back)):
    #         if k < len_fore:
    #             plt.plot(v_fore[k][0], v_fore[k][1], linewidth='3', color='gray', marker='o')
    #         if k < len_back:
    #             plt.plot(v_back[k][0], v_back[k][1], linewidth='3', color='cornflowerblue', marker='o')

    #         plt.gcf().canvas.mpl_connect('key_release_event',
    #                                      lambda event: [exit(0) if event.key == 'escape' else None])

    #         if k % 10 == 0:
    #             plt.pause(0.001)
    #     plt.pause(0.01)

    def plot_scale_b_borders(self):
        # 有問題會導致座標軸產生誤差
        """
        Plot the borders of B scale regions (10x10) on the 100x100 map.
        """
        for x in range(0, self.env.self.x_range, 10):
            self.ax.plot([x, x], [0, self.env.self.y_range-1], color='r')  # Vertical lines
        for y in range(0, self.env.self.y_range, 10):
            self.ax.plot([0, self.env.self.x_range-1], [y, y], color='r')
            # 创建新的座标轴，共享原始 x 轴
        ax2 = self.ax.twiny()
        ax2.xaxis.set_ticks_position("bottom")  # 将新 x 轴设置在底部
        ax2.xaxis.set_label_position("bottom")  # 将新 x 轴标签设置在底部
        ax2.spines["bottom"].set_position(("axes", -0.15))  # 调整新 x 轴的位置

        # 设置新座标轴的刻度，假设每个 B 尺度的单位代表 A 尺度下的 10 个单位
        new_tick_locations = np.arange(0, self.env.self.x_range, 10)

        # 设置新座标轴的刻度和刻度标签
        ax2.set_xticks(new_tick_locations)
        ax2.set_xticklabels(new_tick_locations // 10)  # B 尺度的刻度标签
        ax2.set_xlim(self.ax.get_xlim())  
        ax2.set_ylim(self.ax.get_ylim())



    @staticmethod
    def color_list():
        cl_v = ['silver',
                'wheat',
                'lightskyblue',
                'royalblue',
                'slategray']
        cl_p = ['gray',
                'orange',
                'deepskyblue',
                'red',
                'm']
        return cl_v, cl_p

    @staticmethod
    def color_list_2():
        cl = ['silver',
              'steelblue',
              'dimgray',
              'cornflowerblue',
              'dodgerblue',
              'royalblue',
              'plum',
              'mediumslateblue',
              'mediumpurple',
              'blueviolet',
              ]
        return cl