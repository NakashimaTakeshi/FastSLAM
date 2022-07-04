import math, random
import scipy.stats as stats


class MCL:
    #
    # This code dose not keep trajectory information of robot
    def __init__(self, n_particle: int = 30):
        self._n_particle = n_particle

        # original pose of robot

        # origin
        self.particle_set = [Particle(i, weight=1.0 / self._n_particle) for i in range(n_particle)]

        # random
        # self.particle_set = []
        # for i in range(self._n_particle):
        #     x = random.uniform(-100, 100)
        #     y = random.uniform(-100, 100)
        #     theta = random.uniform(0, 2 * math.pi)
        #     self.particle_set.append(Particle(i, x, y, theta, 1.0 / self._n_particle))

    def update(self, odometry_data):
    # def update(self, odometry_data, sensor_data):

        # This function refer to "Probabilistic Robotics chaper 8.3.2 MCL algorithm"
        particle_set = []
        for particle in self.particle_set:
            particle.pose = self._sample_motion_model_odometory(particle.pose, odometry_data)
            # mesurement_model(particle, sensor_data)
            particle_set.append(particle)

        # resampling

    def _sample_motion_model_odometory(self, priviouse_pose, odometry_data):
        # This function refer to "Probabilistic Robotics chaper 5.4.2 sample motion model odometory algorithm"

        # noise parameter
        alpha1 = 0.0001
        alpha2 = 0.0001
        alpha3 = 0.0001
        alpha4 = 0.0001

        delta_rotation1 = math.atan2(odometry_data.delta_y, odometry_data.delta_x) - priviouse_pose._theta
        delta_translation = math.sqrt(odometry_data.delta_x ** 2 + odometry_data.delta_y ** 2)
        delta_rotation2 = odometry_data.delta_theta-delta_rotation1

        delta_rotation1 += random.gauss(0, alpha1*delta_rotation1**2+alpha2*delta_translation**2)
        delta_translation += random.gauss(0, alpha3*delta_translation**2+alpha4*delta_rotation1**2+alpha4*delta_rotation2**2)
        delta_rotation2 += random.gauss(0, alpha1*delta_rotation2**2+alpha2*delta_translation**2)

        # update pose
        pose = Pose(priviouse_pose._x + delta_translation * math.cos(priviouse_pose._theta + delta_rotation1),
                    priviouse_pose._y + delta_translation * math.sin(priviouse_pose._theta + delta_rotation1),
                    priviouse_pose._theta + delta_rotation1 + delta_rotation2)

        return pose

    def mesurement_model(self, particle, sensor_data):
        pass

    def resampling(self):
        pass


class Particle:
    def __init__(self, id: int, x: float = 0.0, y: float = 0.0, theta: float = 0.0, weight: float = 1.0):
        self._id = id
        self.pose = Pose(x, y, theta)
        self._weight = weight
        self.parent_id = None

    def __str__(self):
        return "id:{}, x:{}, y:{}, theta:{}, weight:{}".format(self._id, self._x, self._y, self._theta, self._weight)

    def __repr__(self):
        return self.__str__()


class Pose:
    def __init__(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0):
        self._x = x
        self._y = y
        self._theta = theta
