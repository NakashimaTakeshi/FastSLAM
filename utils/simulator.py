import math, random
import time

import matplotlib.pyplot as plt
import numpy as np
import keyboard

from robot import Robot
from mcl import MCL

class Simulator(object):
    """
    Class that holds attributes of the simulation environment including robots and landmarks,
    and provides methods of operator-interface, visualization and observation.
    """

    # The URL below is referenced.
    # https://github.com/ryuichiueda/probrobo_practice/blob/master/monte_carlo_localization/1.monte_calro_localization.ipynb

    def __init__(self, x_region: int = 200, y_region: int = 200, n_landmark: int=5):
        """
        Args:
            x_region : x range of simulation 2D field.
            y_region : The second parameter.
            n_landmark: number of landmarks.
        """
        if x_region < 0 or y_region < 0:
            raise ValueError('x_region and y_region must be positive.')

        self._x_region = x_region
        self._y_region = y_region

        self._robot = Robot(pose=[0.0, 0.0, 0.0],x_region=self._x_region,y_region=self._y_region)
        self._generate_random_landmarks(n_landmark)

        self._mcl = MCL(n_particle=30)
        # self._mcl = MCL(n_particle=30,x_region,y_region)
        self._launch_operator_interface()

    def _generate_random_landmarks(self, n: int):
        self.landmarks = []
        for i in range(n):
            x = random.randint(-self._x_region/2.0, self._x_region/2.0)
            y = random.randint(-self._y_region/2.0, self._y_region/2.0)
            self.landmarks.append([x, y])

    def _setup_visualization(self):
        plt.ion()
        self.fig = plt.figure(num=1)
        sp =self.fig.add_subplot(111, aspect='equal')
        sp.set_xlim(-self._x_region/2.0-5, self._x_region/2.0+5)
        sp.set_ylim(-self._y_region/2.0-5, self._y_region/2.0+5)

    def _update_visualization(self):
        for artist in plt.gca().lines + plt.gca().collections:
            artist.remove()

        # draw robot
        plt.quiver(self._robot.pose.x, self._robot.pose.y, math.cos(self._robot.pose.theta), math.sin(self._robot.pose.theta), color="red", label="actual robot pose")

        # draw landmarks
        plt.scatter([r[0] for r in self.landmarks], [r[1] for r in self.landmarks], s=100, marker="1", label="landmarks", color="orange")

        # draw particle set
        # plt.scatter([particle.pose._x for particle in self._mcl.particle_set], [particle.pose._y for particle in self._mcl.particle_set], s=10, marker=".", label="particles", color="C2")
        plt.quiver([particle.pose._x for particle in self._mcl.particle_set], [particle.pose._y for particle in self._mcl.particle_set],
                   [math.cos(particle.pose._theta) for particle in self._mcl.particle_set], [math.sin(particle.pose._theta) for particle in self._mcl.particle_set],
                   [particle._weight for particle in self._mcl.particle_set],  label="particles")
                   # color="blue", label="particle", alpha=[particle._weight for particle in self._mcl.particle_set],alpha=0.7)
        plt.legend(loc='right', bbox_to_anchor=(1.5, 0.5))
        self.fig.canvas.flush_events()

    def _return_odometory_and_obsabation(self):
        return

    def _launch_operator_interface(self):
        counter = 0
        sleep_time = 0.01
        msg = """
        ---------------------------
        Motion:
                   w
              a         d
                   x
        Ovservation:
                   r
        ---------------------------
        """
        print(msg)
        self._setup_visualization()
        while True:
            if keyboard.is_pressed("w"):
                counter += 1
                self._robot.move_forward(counter)
                time.sleep(sleep_time)
            elif keyboard.is_pressed("x"):
                counter += 1
                self._robot.move_backward(counter)
                time.sleep(sleep_time)
            elif keyboard.is_pressed("a"):
                counter += 1
                self._robot.turn_left(counter)
                time.sleep(sleep_time)
            elif keyboard.is_pressed("d"):
                counter += 1
                self._robot.turn_right(counter)
                time.sleep(sleep_time)
            elif keyboard.is_pressed("r"):
                counter = 0
                # call mcl algorithm
                # self._return_odometory_and_obsabation(self._robot, self.landmarks)
                self._mcl.update(self._robot.return_odometry())

                time.sleep(sleep_time)
            else:
                counter = 0
                time.sleep(sleep_time)

            self._update_visualization()

if __name__ =='__main__':
    simulator_test = Simulator()
    # mci_test = MCL()
    # simulator_test._visualization()

