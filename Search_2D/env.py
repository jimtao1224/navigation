"""
Env 2D
@author: huiming zhou
"""


class Env:
    def __init__(self):
        self.x_range = 100  # size of background
        self.y_range = 60
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

        for i in range(x):
            obs.add((i, 0))
        for i in range(x):
            obs.add((i, y - 1))

        for i in range(y):
            obs.add((0, i))
        for i in range(y):
            obs.add((x - 1, i))

        for i in range(20, 40):
            obs.add((i, 30))
        for i in range(30):
            obs.add((40, i))

        for i in range(30, 60):
            obs.add((60, i))
        for i in range(32):
            obs.add((80, i))

        return obs



# class Env:
#     def __init__(self):
#         self.x_range = 1000  # size of background (doubled)
#         self.y_range = 800   # size of background (doubled)
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

#         # Add boundary obstacles (outer boundaries)
#         for i in range(x):
#             obs.add((i, 0))
#             obs.add((i, y - 1))

#         for i in range(y):
#             obs.add((0, i))
#             obs.add((x - 1, i))

#         # Add complex obstacles (inner boundaries)
#         for i in range(100, 200):  # Increase size and complexity
#             for j in range(100, 200):  # Increase size and complexity
#                 obs.add((i, j))

#         for i in range(300, 400):  # Increase size and complexity
#             for j in range(300, 400):  # Increase size and complexity
#                 obs.add((i, j))

#         for i in range(600, 700):  # Increase size and complexity
#             for j in range(200, 600):  # Increase size and complexity
#                 obs.add((i, j))

#         for i in range(800, 900):  # Increase size and complexity
#             for j in range(100, 700):  # Increase size and complexity
#                 obs.add((i, j))

#         # Add more complex obstacles
#         for i in range(200, 400, 20):  # Increase step size
#             for j in range(200, 400, 20):  # Increase step size
#                 obs.add((i, j))

#         for i in range(600, 800, 20):  # Increase step size
#             for j in range(400, 600, 20):  # Increase step size
#                 obs.add((i, j))

#         for i in range(400, 600, 20):  # Increase step size
#             for j in range(100, 300, 20):  # Increase step size
#                 obs.add((i, j))

#         for i in range(700, 900, 20):  # Increase step size
#             for j in range(500, 700, 20):  # Increase step size
#                 obs.add((i, j))

        return obs
