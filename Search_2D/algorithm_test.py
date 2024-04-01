import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search_based_Planning/")

from Search_2D import plotting_test, env_test,test_getneighbor
class algorithm_test:
    def __init__(self):
        self.Env = env_test.Env()
        self.scale = self.Env.scale
        self.x_range, self.y_range = self.set_scale(self.scale)
        self.obs = self.Env.obs_map()
        self.obs_map = self.Env.obs_map()
        self.obs_A = self.Env.obs_map(scale='A')
        self.obs_B = self.Env.obs_map(scale='B')
    def data_set(self):
        data_set = [self.Env.obstacle_coverage(),self.Env.get_map_size(),]
        return data_set





