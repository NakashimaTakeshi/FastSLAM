import math

class Pose:
    def __init__(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0):
        self.x = x
        self.y = y
        self.theta = theta

    def __str__(self):
        return "x:{}, y:{}, theta:{}".format(self.x, self.y, self.theta)

    def __repr__(self):
        return self.__str__()

    @property
    def theta(self):
        return self._theta

    @theta.setter
    def theta(self, theta):
        if theta > math.pi:
            theta -= 2 * math.pi
        elif theta < -math.pi:
            theta += 2 * math.pi
        self._theta = theta
