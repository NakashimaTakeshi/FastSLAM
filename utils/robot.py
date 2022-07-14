import math
import copy

from mcl.pose import Pose

class Robot(object):
    """
    Class that holds attributes of the robot pose,sensor specs and motion specs.
    and provides methods of operate atoribute avabu.
    """

    def __init__(self, pose: list = [0.0, 0.0, 0.0], x_region: int = 100, y_region: int = 100):
        """
        Args:
            pose: initial pose of robot.
            x_region,y_region : robot movement area (prefer to be same as range of simulation 2D field.)
        """
        self._x_restriction = x_region / 2
        self._y_restriction = y_region / 2

        self.pose = ActualPose(x=pose[0], y=pose[1], theta=pose[2], x_restriction=self._x_restriction, y_restriction=self._y_restriction)
        self.range_sensor = Sensor(range_min=10, range_max=50)

        self.linear_speed_min = 0.1
        self.linear_speed_max = 10.0
        self.angular_speed_min = 1.0 * math.pi / 180.0
        self.angular_speed_max = 30.0 * math.pi / 180.0

        self._odometry = Odometry(self.pose)

    def move_forward(self, counter: int = 1):
        linear_speed = counter * self.linear_speed_min
        if linear_speed > self.linear_speed_max:
            linear_speed = self.linear_speed_max

        self.pose.x += linear_speed * math.cos(self.pose.theta)
        self.pose.y += linear_speed * math.sin(self.pose.theta)

        self._odometry.update(self.pose)

    def move_backward(self, counter: int = 1):
        linear_speed = counter * self.linear_speed_min
        if linear_speed > self.linear_speed_max:
            linear_speed = self.linear_speed_max

        self.pose.x -= linear_speed * math.cos(self.pose.theta)
        self.pose.y -= linear_speed * math.sin(self.pose.theta)

        self._odometry.update(self.pose)

    def turn_left(self, counter: int = 1):
        angular_speed = counter * self.angular_speed_min
        if angular_speed > self.angular_speed_max:
            angular_speed = self.angular_speed_max

        self.pose.theta += angular_speed

        self._odometry.update(self.pose)

    def turn_right(self, counter: int = 1):
        angular_speed = counter * self.angular_speed_min
        if angular_speed > self.angular_speed_max:
            angular_speed = self.angular_speed_max

        self.pose.theta -= angular_speed

        self._odometry.update(self.pose)

    def return_odometry(self):
        return_odomertry = copy.deepcopy(self._odometry)
        self._odometry.__init__(self.pose)
        return return_odomertry

    def _observe_landmarks(self):
        pass


class ActualPose(Pose):
    def __init__(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0, x_restriction: float = 100.0, y_restriction: float = 100.0):
        self._x_restriction = x_restriction
        self._y_restriction = y_restriction
        super().__init__(x, y, theta)

    # @property
    # def _theta(self):
    #     return self.__theta
    #
    # @_theta.setter
    # def theta(self, theta):
    #     if theta > math.pi:
    #         theta -= 2 * math.pi
    #     elif theta < -math.pi:
    #         theta += 2 * math.pi
    #     self._theta = theta

    @property
    def x(self):
        return self._x

    @x.setter
    def x(self, x):
        if x > self._x_restriction:
            x = self._x_restriction
        elif x < -self._x_restriction:
            x = -self._x_restriction
        self._x = x

    @property
    def y(self):
        return self._y

    @y.setter
    def y(self, y):
        if y > self._y_restriction:
            y = self._y_restriction
        elif y < -self._y_restriction:
            y = -self._y_restriction
        self._y = y


class Sensor():
    def __init__(self,  range_min: float = 10.0,range_max: float = 50.0, angular_range: float = 360.0):
        if range_max <= range_min:
            raise ValueError('range_max must be larger than range_min.')
        if not 0 <= angular_range <= 360:
            raise ValueError('range_max must be in the range 0.0 to 360.0')

        self.distance_range_max = range_max
        self.distance_range_min = range_min

        angular_range = angular_range * math.pi / 180.0
        self.angle_range_min = -angular_range / 2
        self.angle_range_max = angular_range / 2

        self.distance_noise_stddev = 0.0
        self.angular_noise_stddev = 0.0


class Odometry:
    def __init__(self, pose):
        self.previous_pose = copy.deepcopy(pose)
        self.current_pose = pose

    def __str__(self):
        return "delta_x:{}, delta_y:{}, delta_theta:{}".format(self.current_pose.x - self.previous_pose.x, self.current_pose.y - self.previous_pose.y, self.current_pose.theta - self.previous_pose.theta)

    def update(self, pose):
        self.current_pose = pose
