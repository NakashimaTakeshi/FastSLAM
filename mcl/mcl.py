import math, random

import numpy as np
import scipy.stats as stats

from mcl.pose import Pose


class MCL:
    # Monte Carlo Localization algorithm
    # This code dose not keep trajectory information of robot
    def __init__(self, n_particle: int = 30, x_region: int = 200, y_region: int = 200, scalefoctor_motion: float = 1.0, scalefoctor_measurement: float = 1.0):
        self._n_particle = n_particle

        # original pose of robot

        # origin
        self.particle_set = [Particle(i, weight=1.0 / self._n_particle) for i in range(n_particle)]

        # random
        """
        self.particle_set = []
        random.seed(95864133587513547965)
        for i in range(self._n_particle):
            x = random.uniform(-x_region / 2, x_region / 2)
            y = random.uniform(-y_region / 2, y_region / 2)
            theta = random.uniform(0, 2 * math.pi)
            self.particle_set.append(Particle(i, x, y, theta, 1.0 / self._n_particle))
        """

        self.scalefoctor_motion = scalefoctor_motion
        self.scalefoctor_measurement = scalefoctor_measurement

    def update_particles(self, odometry, observations, landmarks):
        # This method refer to "Probabilistic Robotics chaper 8.3.2 MCL algorithm"
        temp_particle_set = []

        for particle in self.particle_set:
            particle.pose = self._sample_motion_model_odometory(particle.pose, odometry, scalefactor=self.scalefoctor_motion)
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
    def __init__(self, id: int, x: float = 0.0, y: float = 0.0, theta: float = 0.0, weight: float = 1.0):
        self._id = id
        self.pose = Pose(x, y, theta)
        self._weight = weight
        self.parent_id = None  # self.features = [] # for FastSLAM1.0

    def __str__(self):
        return "id:{}, x:{}, y:{}, theta:{}, weight:{}".format(self._id, self.x, self.y, self.theta, self._weight)
