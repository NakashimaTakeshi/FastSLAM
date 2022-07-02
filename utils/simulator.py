import math, random
import time

import matplotlib.pyplot as plt
import numpy as np
import keyboard

from robot import Robot
class Simulator(object):
    """
    Class that holds attributes of the simulation environment including robots and landmarks,
    and provides methods of operator-interface, visualization and observation.
    """

    # The URL below is referenced.
    # https://github.com/ryuichiueda/probrobo_practice/blob/master/monte_carlo_localization/1.monte_calro_localization.ipynb

    def __init__(self, x_region: int = 200, y_region: int = 200,N_landmark:int=10):
        """
        Args:
            x_region : x range of simulation 2D field.
            y_region : The second parameter.
            N_landmark: number of landmarks.
        """
        if x_region < 0 or y_region < 0:
            raise ValueError('x_region and y_region must be positive.')

        self._x_region = x_region
        self._y_region = y_region
        self._robot = Robot(pose=[0.0, 0.0, 0.0])
        self._generate_random_landmarks(N_landmark)

        self._launch_operator_interface()
        # self._visualization()

    def _generate_random_landmarks(self, N: int):
        self.landmarks = []
        for i in range(N):
            x = random.randint(0, self._x_region)
            y = random.randint(0, self._y_region)
            self.landmarks.append([x, y])

    def _setup_visualization(self):
        plt.ion()
        self.fig = plt.figure(num=1)
        sp =self.fig.add_subplot(111, aspect='equal')
        sp.set_xlim(-self._x_region/2.0, self._x_region/2.0)
        sp.set_ylim(-self._y_region/2.0, self._y_region/2.0)

    def _update_visualization(self):
        for artist in plt.gca().lines + plt.gca().collections:
            artist.remove()

        # draw robot
        plt.quiver(self._robot.pose.x, self._robot.pose.y, math.cos(self._robot.pose.theta), math.sin(self._robot.pose.theta), color="red", label="actual robot pose")

        self.fig.canvas.flush_events()

        # self._launch_operator_interface()

        # landmarks.draw()

        # plt.legend()
        # plt.show()

    def _return_odometory_and_obsabation(self):
        return

    def _launch_operator_interface(self):
        counter = 0
        sleep_time = 0.01
        msg = """
        ---------------------------
        Motion:
                   w
              a    s    d
                   x
        Ovservation:
            q
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
            elif keyboard.is_pressed("q"):
                counter = 0
                self._return_odometory_and_obsabation(self._robot, self.landmarks)
                time.sleep(sleep_time)
            else:
                counter = 0
                time.sleep(sleep_time)

            self._update_visualization()

if __name__ =='__main__':
    simulator_test = Simulator()
    # mci_test = MCL()
    # simulator_test._visualization()

