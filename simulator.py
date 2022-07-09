import math, random
import time

import matplotlib.pyplot as plt
import keyboard

from utils.robot import Robot
from mcl.mcl import MCL
from utils.observation import Observation
from utils.landmark import Landmark
from utils.utils import *


class Simulator(object):
    """
    Class that holds attributes of the simulation environment including robots and landmarks,
    and provides methods of operator-interface, visualization and observation.
    """

    # The URL below is referenced.
    # https://github.com/ryuichiueda/probrobo_practice/blob/master/monte_carlo_localization/1.monte_calro_localization.ipynb

    def __init__(self, x_region: int = 200, y_region: int = 200, n_landmark: int = 5):
        """
        Args:
            x_region,y_region : range of simulation 2D field.
            n_landmark: number of landmarks.
        """
        if x_region < 0 or y_region < 0:
            raise ValueError('x_region and y_region must be positive.')

        self._x_region = x_region
        self._y_region = y_region

        self._robot = Robot(pose=[0.0, 0.0, 0.0], x_region=self._x_region, y_region=self._y_region)
        self._generate_random_landmarks(n_landmark)
        self._observing_landmarks = []

        self._mcl = MCL(n_particle=30, x_region=self._x_region, y_region=self._y_region)

        # select operation mode
        # self._launch_operator_interface()
        self._read_operation_data()

    def _generate_random_landmarks(self, n: int):
        self._landmarks = []
        random.seed(123465785432135466854321)
        for i in range(n):
            x = random.randint(-self._x_region / 2.0, self._x_region / 2.0)
            y = random.randint(-self._y_region / 2.0, self._y_region / 2.0)
            self._landmarks.append(Landmark(i, x, y))

    def _setup_visualization(self):
        plt.ion()
        self.fig = plt.figure(num=1)
        sp = self.fig.add_subplot(111, aspect='equal')
        sp.set_xlim(-self._x_region / 2.0 - 5, self._x_region / 2.0 + 5)
        sp.set_ylim(-self._y_region / 2.0 - 5, self._y_region / 2.0 + 5)

    def _update_visualization(self):
        for artist in plt.gca().lines + plt.gca().collections:
            artist.remove()

        # draw particle set
        max_wight = max([particle._weight for particle in self._mcl.particle_set])
        min_wight = min([particle._weight for particle in self._mcl.particle_set])
        if max_wight != min_wight:
            color_list = rescale_list([particle._weight for particle in self._mcl.particle_set], 0.5, 1.0)
        else:
            color_list = [0.8 for _ in self._mcl.particle_set]

        plt.quiver([particle.pose._x for particle in self._mcl.particle_set], [particle.pose._y for particle in self._mcl.particle_set], [math.cos(particle.pose._theta) for particle in
                                                                                                                                          self._mcl.particle_set], [math.sin(particle.pose._theta) for
                                                                                                                                                                    particle in
                                                                                                                                                                    self._mcl.particle_set], color_list, cmap="Greys", clim=(
            0.0, 1.0), label="particles")

        # draw robot
        plt.quiver(self._robot.pose.x, self._robot.pose.y, math.cos(self._robot.pose.theta), math.sin(self._robot.pose.theta), color="red", label="actual robot pose")

        # draw observating landmarks
        plt.scatter([r.x for r in self._observing_landmarks], [r.y for r in self._observing_landmarks], s=50, marker="o", label="observing_landmarks", color="blue")

        # draw landmarks
        plt.scatter([r.x for r in self._landmarks], [r.y for r in self._landmarks], s=100, marker="1", label="landmarks", color="orange")

        plt.legend(loc='right', bbox_to_anchor=(1.75, 0.5))
        self.fig.canvas.flush_events()
        save_png()

    def _return_obsabation(self, robot: Robot, landmarks: list):
        """
        Args:
        """
        obsabation_set: list = []
        self._observing_landmarks = []

        for landmark in landmarks:
            distance = math.sqrt((landmark.x - robot.pose.x) ** 2 + (landmark.y - robot.pose.y) ** 2)
            angle = math.atan2(landmark.y - robot.pose.y, landmark.x - robot.pose.x) - robot.pose.theta

            distance -= random.gauss(0, robot.range_sensor.distance_noise_stddev)
            angle -= random.gauss(0, robot.range_sensor.angular_noise_stddev)

            angle = (angle + math.pi) % (2 * math.pi) - math.pi

            if (robot.range_sensor.distance_range_min <= distance <= robot.range_sensor.distance_range_max) and (robot.range_sensor.angle_range_min <= angle <= robot.range_sensor.angle_range_max):
                obsabation_set.append(Observation(landmark_id=landmark.id, distance=distance, angle=angle))

                self._observing_landmarks.append(landmark)

        return obsabation_set

    def _launch_operator_interface(self):
        long_press_counter = 0
        mcl_calling_counter = 0
        sleep_time = 0.01
        msg = """
        ---------------------------
        Motion:
                   w
              a         d
                   x
        ---------------------------
        """
        print(msg)
        self._setup_visualization()
        self._update_visualization()
        try:
            while True:
                if keyboard.is_pressed("w"):
                    long_press_counter += 1
                    mcl_calling_counter += 1
                    self._robot.move_forward(long_press_counter)
                    time.sleep(sleep_time)
                    self._update_visualization()
                    save_operation("w", long_press_counter)

                elif keyboard.is_pressed("x"):
                    long_press_counter += 1
                    mcl_calling_counter += 1
                    self._robot.move_backward(long_press_counter)
                    time.sleep(sleep_time)
                    self._update_visualization()
                    save_operation("x", long_press_counter)

                elif keyboard.is_pressed("a"):
                    long_press_counter += 1
                    mcl_calling_counter += 1
                    self._robot.turn_left(long_press_counter)
                    time.sleep(sleep_time)
                    self._update_visualization()
                    save_operation("a", long_press_counter)

                elif keyboard.is_pressed("d"):
                    long_press_counter += 1
                    mcl_calling_counter += 1
                    self._robot.turn_right(long_press_counter)
                    time.sleep(sleep_time)
                    self._update_visualization()
                    save_operation("d", long_press_counter)

                # elif keyboard.is_pressed("r"):
                #     long_press_counter = 0
                #     self._mcl.update_particles(self._robot.return_odometry(),self._return_obsabation(self._robot, self._landmarks),self._landmarks)
                #     time.sleep(sleep_time)
                else:
                    long_press_counter = 0
                    time.sleep(sleep_time)

                if mcl_calling_counter >= 3:
                    self._mcl.update_particles(self._robot.return_odometry(), self._return_obsabation(self._robot, self._landmarks), self._landmarks)
                    mcl_calling_counter = 0
                    self._update_visualization()

        except KeyboardInterrupt:
            draw_gif()

    def _read_operation_data(self):
        sleep_time = 0.01
        mcl_calling_counter = 0
        self._setup_visualization()
        self._update_visualization()

        with open("./data/input/operation.csv", "r") as f:
            while True:
                line = f.readline()

                if not line:
                    draw_gif()
                    break

                else:
                    action = line.split(",")[0]
                    long_press_counter = int(line.split(",")[1])

                    if action == "w":
                        self._robot.move_forward(long_press_counter)
                        time.sleep(sleep_time)
                        self._update_visualization()
                        mcl_calling_counter += 1
                    elif action == "x":
                        self._robot.move_backward(long_press_counter)
                        time.sleep(sleep_time)
                        self._update_visualization()
                        mcl_calling_counter += 1
                    elif action == "a":
                        self._robot.turn_left(long_press_counter)
                        time.sleep(sleep_time)
                        self._update_visualization()
                        mcl_calling_counter += 1
                    elif action == "d":
                        self._robot.turn_right(long_press_counter)
                        time.sleep(sleep_time)
                        self._update_visualization()
                        mcl_calling_counter += 1
                    else:
                        time.sleep(sleep_time)

                if mcl_calling_counter >= 3:
                    self._mcl.update_particles(self._robot.return_odometry(), self._return_obsabation(self._robot, self._landmarks), self._landmarks)
                    mcl_calling_counter = 0
                    self._update_visualization()


if __name__ == '__main__':
    clear_output_directory()
    simulator_test = Simulator()
