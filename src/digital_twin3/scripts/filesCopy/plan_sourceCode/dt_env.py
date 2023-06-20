"""
Webots Env 2D
@author: fuyang cheng
"""


class WebotsEnv:
    def __init__(self,x_range=15,y_range=15,unit_size=0.5):
        self.unit_size = unit_size  # size of unit block
        self.x_range = [int(-x_range /2 / self.unit_size), int(x_range /2 / self.unit_size)]  # size of background
        self.y_range = [int(-y_range /2 / self.unit_size), int(y_range /2 / self.unit_size)]
        
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
        obs = set()

        for i in range(self.x_range[0], self.x_range[1] + 1, 1):
            obs.add((i, self.y_range[0]))
            obs.add((i, self.y_range[1]))

        for i in range(self.y_range[0], self.y_range[1] + 1, 1):
            obs.add((self.x_range[0], i))
            obs.add((self.x_range[1], i))

        return obs
