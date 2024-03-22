# """
# Env 2D
# @author: huiming zhou
# """


# class Env:
#     def __init__(self):
#         self.x_range = 100  # size of background
#         self.y_range = 60
#         self.motions = [(-1, 0), (-1, 1), (0, 1), (1, 1),
#                         (1, 0), (1, -1), (0, -1), (-1, -1)]
#         self.obs = self.obs_map()

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
#         for i in range(x):
#             obs.add((i, y - 1))

#         for i in range(y):
#             obs.add((0, i))
#         for i in range(y):
#             obs.add((x - 1, i))

#         for i in range(20, 40):
#             obs.add((i, 30))
#         for i in range(30):
#             obs.add((40, i))

#         for i in range(30, 60):
#             obs.add((60, i))
#         for i in range(32):
#             obs.add((80, i))
#         return obs
class Env:
    def __init__(self):
        self.x_range = 1000  # size of background (doubled)
        self.y_range = 800   # size of background (doubled)
        self.motions = [(-1, 0), (-1, 1), (0, 1), (1, 1),
                        (1, 0), (1, -1), (0, -1), (-1, -1)]
        self.obs = self.obs_map()

    def update_obs(self, obs):
        self.obs = obs

    def obs_map(self):
        """
        Initialize obstacles' positions
        :return: map of obstacles
        """

        x = self.x_range
        y = self.y_range
        obs = set()

        # Add boundary obstacles (outer boundaries)
        for i in range(x):
            obs.add((i, 0))
            obs.add((i, y - 1))

        for i in range(y):
            obs.add((0, i))
            obs.add((x - 1, i))

        # Add complex obstacles (inner boundaries)
        for i in range(100, 200):  # Increase size and complexity
            for j in range(100, 200):  # Increase size and complexity
                obs.add((i, j))

        for i in range(300, 400):  # Increase size and complexity
            for j in range(300, 400):  # Increase size and complexity
                obs.add((i, j))

        for i in range(600, 700):  # Increase size and complexity
            for j in range(200, 600):  # Increase size and complexity
                obs.add((i, j))

        for i in range(800, 900):  # Increase size and complexity
            for j in range(100, 700):  # Increase size and complexity
                obs.add((i, j))

        # Add more complex obstacles
        for i in range(200, 400, 20):  # Increase step size
            for j in range(200, 400, 20):  # Increase step size
                obs.add((i, j))

        for i in range(600, 800, 20):  # Increase step size
            for j in range(400, 600, 20):  # Increase step size
                obs.add((i, j))

        for i in range(400, 600, 20):  # Increase step size
            for j in range(100, 300, 20):  # Increase step size
                obs.add((i, j))

        for i in range(700, 900, 20):  # Increase step size
            for j in range(500, 700, 20):  # Increase step size
                obs.add((i, j))
        return obs
    
    def obstacle_density(self, center, radius):
        count = 0
        for x in range(center[0] - radius, center[0] + radius + 1):
            for y in range(center[1] - radius, center[1] + radius + 1):
                if (x, y) in self.obs:
                    count += 1
        area = (2 * radius + 1) ** 2
        density = count / area
        return density

    def open_space_size(self, center, max_radius):
        for radius in range(1, max_radius + 1):
            for x in range(center[0] - radius, center[0] + radius + 1):
                for y in range(center[1] - radius, center[1] + radius + 1):
                    if (x, y) in self.obs or x < 0 or y < 0 or x >= self.x_range or y >= self.y_range:
                        return (radius - 1) ** 2 * 3.14  # Approximate area of open space
        return max_radius ** 2 * 3.14

    def connectivity(self, center):
        connected = 0
        for motion in self.motions:
            next_node = (center[0] + motion[0], center[1] + motion[1])
            if next_node not in self.obs and 0 <= next_node[0] < self.x_range and 0 <= next_node[1] < self.y_range:
                connected += 1
        return connected / len(self.motions)  # Ratio of connected directions

# Create environment
env = Env()

# Test the environment analysis functions
center = (500, 400)  # Center of the environment
radius = 50  # Radius for obstacle density and open space size
max_radius = 100  # Maximum radius for open space size

obstacle_density = env.obstacle_density(center, radius)
open_space_size = env.open_space_size(center, max_radius)
connectivity = env.connectivity(center)

# Print the results of the tests
print(f"Obstacle Density around center {center} with radius {radius}: {obstacle_density}")
print(f"Open Space Size around center {center} with max radius {max_radius}: {open_space_size}")
print(f"Connectivity at center {center}: {connectivity}")

def dynamic_search_range(self, current_node):
    # 環境分析
    density = self.obstacle_density(current_node, radius=10)  # 例如，以半徑 10 單位進行障礙物密度計算
    open_space = self.open_space_size(current_node, max_radius=20)  # 例如，最大半徑為 20 單位的開闊空間大小
    connectivity = self.connectivity(current_node)  # 當前節點的連通性

    # 基於環境分析結果動態調整搜索範圍
    if density < 0.1 and open_space > 400:  # 條件可以根據需要調整
        # 增加搜索範圍
        search_range = ...
    elif connectivity < 0.5:
        # 縮小搜索範圍
        search_range = ...
    else:
        # 默認搜索範圍
        search_range = ...

    # 根據調整後的搜索範圍擴展節點
    for motion in self.adjusted_motions(search_range):
        next_node = (current_node[0] + motion[0], current_node[1] + motion[1])
        self.expand_node(next_node)