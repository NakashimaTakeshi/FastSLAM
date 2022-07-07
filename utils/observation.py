import math

class Observation:
    def __init__(self,distance: float, angle: float,landmark_id: int):
        self.distance = distance
        self.angle = angle
        self.landmark_id = landmark_id

    def __repr__(self):
        return 'distance:{}, angle:{}, landmark_id:{}'.format(self.distance, self.angle, self.landmark_id)

    @property
    def angle(self):
        return self._angle

    @angle.setter
    def angle(self, angle):
        if angle > math.pi:
            angle -= 2 * math.pi
        elif angle < -math.pi:
            angle += 2 * math.pi
        self._angle = angle