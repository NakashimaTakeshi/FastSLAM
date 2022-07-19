import math, random

import numpy as np
import scipy.stats as stats

from mcl.pose import Pose
from mcl.mcl import Particle


class FastSlamParticle(Particle):
    def __init__(self, id: int, x: float = 0.0, y: float = 0.0, theta: float = 0.0, weight: float = 1.0):
        super().__init__(id, x, y, theta, weight)
        self._features = []

class FastSlamFeature:
    def __init__(self):
        self._mean = 0.0
        self._covariance = [[][]]