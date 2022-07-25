import math, random
import copy

import numpy as np
import scipy.stats as stats

from mcl.pose import Pose
from mcl.mcl import Particle
from mcl.mcl import MCL


class FastSlam(MCL):
    # FastSlam1.0 algorithm
    # This code dose not keep trajectory information of robot
    def __init__(self, n_particle: int = 30, x_region: int = 200, y_region: int = 200):
        self._n_particle = n_particle
        self.particle_set = [copy.deepcopy(FastSlamParticle(i, weight=1.0 / self._n_particle)) for i in range(n_particle)]
        self.privious_observed_landmarks = None  # super().__init__(n_particle, x_region, y_region)

    def update_particles(self, odometry, observations, landmarks):
        # This method refer to "Probabilistic Robotics chaper 13.3 known correspondence FastSLAM1.0 algorithm"
        for i, particle in enumerate(self.particle_set):
            # for particle in self.particle_set:
            particle.pose = self._sample_motion_model_odometory(particle.pose, odometry)
            particle._weight *= self._fastslam_measurement_model(particle, observations, landmarks, i)

        print([particle._weight for particle in self.particle_set])

        if sum(particle._weight for particle in self.particle_set) == 0:
            for particle in self.particle_set:
                particle._weight = 1.0 / self._n_particle
        else:
            nomalized_weights = [particle._weight / sum(particle._weight for particle in self.particle_set) for particle in self.particle_set]

        n_eff = 1.0 / sum(w ** 2 for w in nomalized_weights)
        if n_eff < 0.5 * self._n_particle or sum(nomalized_weights) < 1.0e-15:
            print("resampling")
            self.particle_set = self._fastslam_resampling(self.particle_set)  # print([particle._weight for particle in self.particle_set])

    def _fastslam_measurement_model(self, particle, observations, landmarks, i):
        # set standard deviation
        std_distance = 10.0
        std_angle = 22.5 * math.pi / 180.0
        std_correspondence = 1 / math.sqrt(2 * math.pi)

        Q_t = np.diag([std_distance, std_angle]) ** 2  # covariance matrix of measurement noise

        likelihood = 1.0
        for observation in observations:
            z = np.array([observation.distance, observation.angle])
            j = observation.landmark_id

            k = list(filter(lambda feature: feature._landmark_id == j, particle._features))
            if len(k) == 0:
                print("newlandmark!!!!!!!!!!")
                mu = self._inverse_mesurement_model_h(observation, particle.pose)
                H = self._calculate_jacobian(particle.pose, mu)
                Sigma = np.linalg.inv(H) * Q_t * np.linalg.inv(H).T
                particle._features.append(FastSlamFeature(mu, Sigma, j))  # self.particle_set[i]._features.append(FastSlamFeature(mu, Sigma, j))  # TODO: weight の更新が必要かどうか確認する
            else:
                print("oldlandmark!!!!!!!!!!")
                l = ([i for i, feature in enumerate(particle._features) if feature._landmark_id == j])[0]
                feature = k[0]
                z_hat = self._mesurement_model_h(feature, particle.pose)
                H = self._calculate_jacobian(particle.pose, feature._mean)
                Q = H * feature._covariance_matrix * H.T + Q_t
                K = feature._covariance_matrix * H.T * np.linalg.inv(Q)
                particle._features[l]._mean = feature._mean + np.ravel(K * np.matrix(z - z_hat).T).T
                particle._features[l]._covariance_matrix = (np.eye(2) - K * H) * feature._covariance_matrix
                likelihood *= stats.multivariate_normal.pdf(z_hat, z, Q)
                print(likelihood, stats.multivariate_normal.pdf(z_hat, z, Q))
        return likelihood

    def _fastslam_resampling(self, particle_set: list):
        nomalized_weights = [particle._weight / sum(particle._weight for particle in particle_set) for particle in particle_set]
        # nomalized_weights =[0.0007624988072525044, 2.1053482571456744e-06, 1.5227297629110111e-32, 3.4988609303279427e-22, 4.139987460838172e-05, 0.030828725844310043, 0.10545369687158535, 7.67216286321623e-09,0.05673872425836597, 1.4465599636455072e-24, 0.0009409887675221956, 0.10571896586571226, 0.0010420502682387935, 2.1055182623538223e-17, 0.1357807713104042, 0.11118553737085872, 8.585768986358893e-11,4.410069064980013e-112, 5.2699058299385345e-06, 0.11081186370212104, 1.7517020215731466e-07, 4.0258622181137e-249, 4.995118889930072e-05, 3.1862864480702675e-50, 0.05085976840085821, 0.136075354508565, 4.730689448549953e-23, 0.07720382656121431,0.07649831821717393, 3.4873823536214346e-159]
        if sum(nomalized_weights) >= 1.0:
            nomalized_weights = [w / (sum(nomalized_weights) + 1.0e-15) for w in nomalized_weights]
            print("double normalization")

        # nomalized_weights = [w if w > 1.0e-20 else 0.0 for w in nomalized_weights]

        rv = stats.multinomial(n=1, p=nomalized_weights)
        print(sum(nomalized_weights), nomalized_weights)
        # print(max(nomalized_weights),min(nomalized_weights))
        new_particle_set = []
        for i in range(self._n_particle):
            rvs = rv.rvs(size=1)
            survival_index = np.argmax(rvs[0])
            new_particle_set.append(copy.deepcopy(FastSlamParticle(id=i, x=particle_set[survival_index].pose.x, y=particle_set[survival_index].pose.y, theta=particle_set[
                survival_index].pose.theta, weight=1.0 / self._n_particle, features=particle_set[survival_index]._features)))

        return new_particle_set

    @staticmethod
    def _calculate_jacobian(pose, mean):
        # refer to "Probabilistic Robotics chaper 7.4.3"
        dx = pose.x - mean[0]
        dy = pose.y - mean[1]
        q = dx ** 2 + dy ** 2

        H = np.matrix([[-dx / math.sqrt(q), -dy / math.sqrt(q), 0.0], [dy / q, -dx / q, -1.0], [0.0, 0.0, 0.0]])
        return H[:2, :2]

    @staticmethod
    def _mesurement_model_h(feature, pose):

        distance = math.sqrt((feature._mean[0] - pose.x) ** 2 + (feature._mean[1] - pose.y) ** 2)
        angle = math.atan2(feature._mean[1] - pose.y, feature._mean[0] - pose.x) - pose.theta

        angle = (angle + math.pi) % (2 * math.pi) - math.pi  # reset angle value in range -pi to pi

        z = np.array([distance, angle])
        return z

    @staticmethod
    def _inverse_mesurement_model_h(observation, pose):

        x = observation.distance * math.cos(observation.angle + pose.theta) + pose.x
        y = observation.distance * math.sin(observation.angle + pose.theta) + pose.y

        mean = np.array([x, y])
        return mean

    # @staticmethod  # def _observation_to_state(observation):  #     return np.array([observation.distance, observation.angle,observation.landmark_id])


class FastSlamParticle(Particle):
    def __init__(self, id: int, x: float = 0.0, y: float = 0.0, theta: float = 0.0, weight: float = 1.0, features: list = []):
        super().__init__(id, x, y, theta, weight)
        self._features = features


class FastSlamFeature:
    def __init__(self, mean, covariance_matrix, landmark_id):
        self._mean = mean
        self._covariance_matrix = covariance_matrix
        self._landmark_id = landmark_id
