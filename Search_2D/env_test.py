# class Env:
#     def __init__(self):
#         self.x_range = 10  # size of background
#         self.y_range = 10
#         self.motions = [(-1, 0), (-1, 1), (0, 1), (1, 1),
#                         (1, 0), (1, -1), (0, -1), (-1, -1)]
#         self.obs = self.obs_map()
#         self.sub_maps_obs = self.generate_sub_map_obs()

#     def update_obs(self, obs):
#         self.obs = obs

#     def obs_map(self):
#         """
#         Initialize obstacles' positions
#         :return: map of obstacles
#         """

#         x = self.x_range
#         y = self.y_range
#         obs = set()
        
#         for i in range(x):
#             obs.add((i, 0))
#             obs.add((i, y - 1))

#         for i in range(y):
#             obs.add((0, i))
#             obs.add((x - 1, i))

#         # for i in range(20, 40):
#         #     obs.add((i, 30))
#         # for i in range(30):
#         #     obs.add((40, i))

#         # for i in range(30, 60):
#         #     obs.add((60, i))
#         # for i in range(32):
#         #     obs.add((80, i))

#         return obs
#     def update_generate_sub_map_obs(self,sub_maps_obs):
#         self.sub_maps_obs = sub_maps_obs
#     def generate_sub_map_obs(self, sub_map_size=10):
#         sub_maps_obs = {}
#         for x in range(0, self.x_range, sub_map_size):
#             for y in range(0, self.y_range, sub_map_size):
#                 sub_map_key = (x // sub_map_size, y // sub_map_size)
#                 sub_maps_obs[sub_map_key] = set()

#                 for i in range(x, x + sub_map_size):
#                     for j in range(y, y + sub_map_size):
#                         if (i, j) in self.obs:
#                             # 將障礙物轉換為相對於小地圖的座標
#                             sub_maps_obs[sub_map_key].add((i - x, j - y))
#         return sub_maps_obs


        
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
            obs = set()
            
            for i in range(x):
                obs.add((i, 0))
                obs.add((i, y - 1))

            for i in range(y):
                obs.add((0, i))
                obs.add((x - 1, i))
            return obs
        elif self.scale == 'B':
            x = self.x_range_B
            y = self.y_range_B
            obs = set()
            for i in range(x):
                obs.add((i, 0))
                obs.add((i, y - 1))

            for i in range(y):
                obs.add((0, i))
                obs.add((x - 1, i))
            return obs
            
    def A_to_B(self, x, y):
        # 将尺度A上的坐标转换为尺度B上的坐标
        return x // 10, y // 10

    def B_to_A(self, x, y, strategy='center'):
        # 将尺度B上的坐标转换为尺度A上的一个区域
        if strategy == 'center':
            return (x * 10 + 5, y * 10 + 5)
        elif strategy == 'topleft':
            return (x * 10, y * 10)
    
