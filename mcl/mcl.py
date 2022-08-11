import math, random
import sys

import numpy as np
import scipy.stats as stats
from typing import Final
import yaml

from mcl.pose import Pose


class MCL:
    # Monte Carlo Localization algorithm
    # This code dose not keep trajectory information of robot

    # read config file
    try:
        with open('./config.yml', 'r') as yml:
            config = yaml.safe_load(yml)

        n_particle: Final[int] = config['state_estimation']['n_particle']
        initial_pose: Final[str] = config['state_estimation']['initial_pose']
        if initial_pose == 'unknown':
            x_region: Final = config['state_estimation']['initial_pose']['x_region']
            y_region: Final = config['state_estimation']['initial_pose']['y_region']
        else:
            x_region: Final = 0
            y_region: Final = 0

    except Exception as e:
        print('Exception occurred while loading YAML...', file=sys.stderr)
        print(e, file=sys.stderr)
        sys.exit(1)

    def __init__(self, n_particle: int = n_particle, x_region: int = x_region, y_region: int = y_region):
        self._n_particle = n_particle

        if self.initial_pose == 'known':
            self.particle_set = [Particle(i, weight=1.0 / self._n_particle) for i in range(n_particle)]
        elif self.initial_pose == 'unknown':
            random.seed(95864133587513547965)
            self.particle_set = []
            for i in range(self._n_particle):
                x = random.uniform(-x_region / 2, x_region / 2)
                y = random.uniform(-y_region / 2, y_region / 2)
                theta = random.uniform(0, 2 * math.pi)
                self.particle_set.append(Particle(i, x, y, theta, 1.0 / self._n_particle))

    def update_particles(self, odometry, observations, landmarks):
        # This method refer to "Probabilistic Robotics chaper 8.3.2 MCL algorithm"
        temp_particle_set = []

        for particle in self.particle_set:
            particle.pose = self._sample_motion_model_odometory(particle.pose, odometry)
            particle._weight *= self._measurement_model(particle, observations, landmarks, scalefactor=self.scalefoctor_measurement)
            temp_particle_set.append(particle)

        # resampling
        """
        nomalized_weights = [particle._weight / sum(particle._weight for particle in temp_particle_set) for particle in temp_particle_set]
        n_eff = 1.0 / sum(w ** 2 for w in nomalized_weights)
        if n_eff < 0.5 * self._n_particle:
            self.particle_set = self._resampling(temp_particle_set)
        else:
            self.particle_set = temp_particle_set
        """
        self.particle_set = self._resampling(temp_particle_set)

    @staticmethod
    def _sample_motion_model_odometory(previouse_pose, odometry, alpha1: float = 1.0e-2, alpha2: float = 1.0e-4, alpha3: float = 1.0e-1, alpha4: float = 1.0e-1, scalefactor: float = 1.0):
        # This method refer to "Probabilistic Robotics chaper 5.4.2 sample motion model odometory algorithm"
        alpha1 = alpha1 * scalefactor
        alpha2 = alpha2 * scalefactor
        alpha3 = alpha3 * scalefactor
        alpha4 = alpha4 * scalefactor

        delta_rotation1 = math.atan2(odometry.current_pose.y - odometry.previous_pose.y, odometry.current_pose.x - odometry.previous_pose.x) - odometry.previous_pose.theta
        delta_translation = math.sqrt((odometry.current_pose.x - odometry.previous_pose.x) ** 2 + (odometry.current_pose.y - odometry.previous_pose.y) ** 2)
        delta_rotation2 = odometry.current_pose.theta - odometry.previous_pose.theta - delta_rotation1

        delta_rotation1 -= random.gauss(0, math.sqrt(alpha1 * delta_rotation1 ** 2 + alpha2 * delta_translation ** 2))
        delta_translation -= random.gauss(0, math.sqrt(alpha3 * delta_translation ** 2 + alpha4 * delta_rotation1 ** 2 + alpha4 * delta_rotation2 ** 2))
        delta_rotation2 -= random.gauss(0, math.sqrt(alpha1 * delta_rotation2 ** 2 + alpha2 * delta_translation ** 2))

        # update pose
        pose = Pose(previouse_pose.x + delta_translation * math.cos(previouse_pose.theta + delta_rotation1), previouse_pose.y + delta_translation * math.sin(previouse_pose.theta + delta_rotation1), previouse_pose.theta + delta_rotation1 + delta_rotation2)

        return pose

    @staticmethod
    def _measurement_model(particle, observations, landmarks, scalefactor: float = 1.0):
        # This method refer to "Probabilistic Robotics chaper 6.6.3 landmark model known correspondence algorithm"

        # set standard deviation
        std_distance = 20.0 * scalefactor
        std_angle = 45. * math.pi / 180.0 * scalefactor

        likelihood = 1.0
        for observation in observations:
            j = observation.landmark_id
            landmark = list(filter(lambda landmark: landmark.id == j, landmarks))[0]

            distance = math.sqrt((landmark.x - particle.pose.x) ** 2 + (landmark.y - particle.pose.y) ** 2)
            angle = math.atan2(landmark.y - particle.pose.y, landmark.x - particle.pose.x) - particle.pose.theta

            delta_angle = observation.angle - angle
            delta_angle = (delta_angle + math.pi) % (2 * math.pi) - math.pi  # reset angle value in range -pi to pi

            likelihood *= stats.norm.pdf(observation.distance - distance, 0, std_distance) * stats.norm.pdf(delta_angle, 0, std_angle)

        if math.isnan(likelihood): likelihood = 0.0
        return likelihood

    def _resampling(self, particle_set: list):
        nomalized_weights = [particle._weight / sum(particle._weight for particle in particle_set) for particle in particle_set]
        if sum(nomalized_weights) > 1.0:
            nomalized_weights = [w / sum(nomalized_weights) for w in nomalized_weights]

        rv = stats.multinomial(n=1, p=nomalized_weights)
        print(f"nomlized weight:{sum(nomalized_weights)}, {nomalized_weights}")
        new_particle_set = []
        for i in range(self._n_particle):
            rvs = rv.rvs(size=1)
            new_particle_set.append(Particle(id=i, x=particle_set[np.argmax(rvs[0])].pose.x, y=particle_set[np.argmax(rvs[0])].pose.y, theta=particle_set[
                np.argmax(rvs[0])].pose.theta, weight=1.0 / self._n_particle))

        return new_particle_set


class Particle:
    # read config file
    try:
        with open('./config.yml', 'r') as yml:
            config = yaml.safe_load(yml)

        alpha1: Final[float] = config['motion_model_parameter']['alpha1']
        alpha2: Final[float] = config['motion_model_parameter']['alpha2']
        alpha3: Final[float] = config['motion_model_parameter']['alpha3']
        alpha4: Final[float] = config['motion_model_parameter']['alpha4']

    except Exception as e:
        print('Exception occurred while loading YAML...', file=sys.stderr)
        print(e, file=sys.stderr)
        sys.exit(1)

    def __init__(self, id: int, x: float = 0.0, y: float = 0.0, theta: float = 0.0, weight: float = 1.0):
        self._id = id
        self.pose = Pose(x, y, theta)
        self._weight = weight
        self.parent_id = None

    def __str__(self):
        return "id:{}, x:{}, y:{}, theta:{}, weight:{}".format(self._id, self.x, self.y, self.theta, self._weight)

    def sample_motion_model_odometory(self, odometry, alpha1:float = alpha1, alpha2:float = alpha2, alpha3:float = alpha3, alpha4:float = alpha4):
        # This method refer to "Probabilistic Robotics chaper 5.4.2 sample motion model odometory algorithm"

        delta_rotation1 = math.atan2(odometry.current_pose.y - odometry.previous_pose.y, odometry.current_pose.x - odometry.previous_pose.x) - odometry.previous_pose.theta
        delta_translation = math.sqrt((odometry.current_pose.x - odometry.previous_pose.x) ** 2 + (odometry.current_pose.y - odometry.previous_pose.y) ** 2)
        delta_rotation2 = odometry.current_pose.theta - odometry.previous_pose.theta - delta_rotation1

        delta_rotation1 -= random.gauss(0, math.sqrt(alpha1 * delta_rotation1 ** 2 + alpha2 * delta_translation ** 2))
        delta_translation -= random.gauss(0, math.sqrt(alpha3 * delta_translation ** 2 + alpha4 * delta_rotation1 ** 2 + alpha4 * delta_rotation2 ** 2))
        delta_rotation2 -= random.gauss(0, math.sqrt(alpha1 * delta_rotation2 ** 2 + alpha2 * delta_translation ** 2))

        # update pose
        pose = Pose(previouse_pose.x + delta_translation * math.cos(previouse_pose.theta + delta_rotation1), previouse_pose.y + delta_translation * math.sin(previouse_pose.theta + delta_rotation1), previouse_pose.theta + delta_rotation1 + delta_rotation2)

        # return pose
        return self.__class__(id=self._id, x=self.pose.x, y=self.pose.y, theta=self.pose.theta, weight=self._weight)