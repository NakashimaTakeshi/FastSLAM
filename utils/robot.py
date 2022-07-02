import math

class Robot(object):
    """
    Class that holds attributes of the robot pose,sensor specs and motion specs.
    and provides methods of operate atoribute avabu.
    """

    def __init__(self, pose: list[float] = [0.0, 0.0, 0.0]):
        """
        Args:
            x_region : x range of simulation 2D field.
            y_region : The second parameter.
            N_landmark: number of landmarks.
        Returns:
            bool: set of range sensor value.
        """
        # To Do
        # 上位クラスの属性(環境の範囲)を条件式に含むValueErrorは設定できるか？
        # Simulation クラスのx_regionとy_regionから外れた場所にロボット位置が来たら、エラーを出すor移動操作をうけないようにしたい。
        self.pose = Pose(x=pose[0], y=pose[1], theta=pose[2])
        self.range_sensor = Sensor(range_min=0, range_max=3.5)

        self.linear_speed_min = 0.1
        self.linear_speed_max = 10.0
        self.angular_speed_min = 1.0 * math.pi / 180.0
        self.angular_speed_max = 30.0 * math.pi / 180.0

    def move_forward(self,counter:int = 1):
        linear_speed=counter*self.linear_speed_min
        if linear_speed > self.linear_speed_max:
            linear_speed = self.linear_speed_max

        self.pose.x += linear_speed * math.cos(self.pose.theta)
        self.pose.y += linear_speed * math.sin(self.pose.theta)
        # To Do
        # シミュレーション環境の範囲外に出た場合にエラーを出すようにしたい。

    def move_backward(self,counter:int = 1):
        linear_speed=counter*self.linear_speed_min
        if linear_speed > self.linear_speed_max:
            linear_speed = self.linear_speed_max

        self.pose.x -= linear_speed * math.cos(self.pose.theta)
        self.pose.y -= linear_speed * math.sin(self.pose.theta)
        # To Do
        # シミュレーション環境の範囲外に出た場合にエラーを出すようにしたい。

    def turn_left(self, counter:int = 1):
        angular_speed=counter*self.angular_speed_min
        if angular_speed > self.angular_speed_max:
            angular_speed = self.angular_speed_max

        self.pose.theta += angular_speed

    def turn_right(self, counter:int = 1):
        angular_speed=counter*self.angular_speed_min
        if angular_speed > self.angular_speed_max:
            angular_speed = self.angular_speed_max

        self.pose.theta -= angular_speed

    def _observe_landmarks(self):
        pass

    def _visualize_robot(self):
        pass


#To Do
#構造体を書く時の注意はあるか、何らかのデコレーション必要？
#setterでx,yを設定する時に、範囲外に出た場合にエラーを出すようにしたい。
# x,y　はクライアントクラスで設定されたシミュレーション環境範囲の情報を取得する必要がある・・・
class Pose():
    def __init__(self, x: float=0.0, y: float=0.0, theta: float=0.0):
        self.x = x
        self.y = y
        self.theta = theta

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

    def __str__(self):
        return f'({self.x}, {self.y}, {self.theta})'

    def __repr__(self):
        return f'Pose({self.x}, {self.y}, {self.theta})'

class Sensor():
    def __init__(self, range_max: float=0.1, range_min:float=3.5,angular_range:float=360.0):
        if range_max <= range_min:
            raise ValueError('range_max must be larger than range_min.')
        if not 0 <= angular_range <= 360:
            raise ValueError('range_max must be in the range 0.0 to 360.0')

        self.distance_range_max = range_max
        self.distance_range_min = range_min
        self.angular_range = angular_range * math.pi / 180.0

        self.distance_range_noise_stddev = 0.0
        self.angular_range_noise_stddev = 0.0

