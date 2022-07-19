from unittest import TestCase
import math

import matplotlib.pyplot as plt

from utils.robot import Odometry
from utils.utils import *

from mcl.pose import Pose
from mcl.mcl import MCL
# from pose import Pose
# from mcl import MCL

class TestMCL(TestCase):
    def test_sample_motion_model_odometory(self,transration,rotation1,rotation2,alpha1:float,alpha2:float,alpha3:float,alpha4:float):

        n_sumpling= 100

        test_instance=MCL()

        # test odometry
        self._previouse_pose=Pose(0,0,0)

        # transration=20
        rotation1=rotation1 *math.pi/180
        rotation2=rotation2*math.pi/180
        self._later_pose=Pose(self._previouse_pose.x+transration*math.cos(rotation1),self._previouse_pose.y+transration*math.sin(rotation1),self._previouse_pose.theta+rotation1+rotation2)

        odometry=Odometry(self._previouse_pose)
        odometry.update(self._later_pose)

        # Test 1
        self.pose_set=[]
        for i in range(n_sumpling):
        # def _sample_motion_model_odometory(previouse_pose, odometry, alpha1: float = 0.01, alpha2: float = 1.0e-4, alpha3: float = 1.0e-1, alpha4: float = 1.0e-1):
        #     self.pose_set.append(test_instance._sample_motion_model_odometory(self._previouse_pose, odometry, alpha1=0.00, alpha2 = 1.0e-4, alpha3 = 0.0, alpha4 = 0.0))
            self.pose_set.append(test_instance._sample_motion_model_odometory(self._previouse_pose, odometry, alpha1=alpha1, alpha2=alpha2, alpha3=alpha3, alpha4=alpha4))

        self._save_visualization()

        # evaluate the result:
        # ToDo: このテストはどれぐらいが適切な分散かを確認して、せっていするひつようがある？
        # xpose_actual.mean(axis=0)
        # xpose_actual.std(axis=0)
        # xpose_actual.var(axis=0)
        # self.assertTrue(all(xpose_actual == pose_expected))

    def _save_visualization(self):
        _x_region_min,_x_region_max=-2,25
        _y_region_min,_y_region_max=-2,25
        # _x_region_min,_x_region_max=0,25
        # _y_region_min,_y_region_max=-5,20

        fig = plt.figure(num=1)
        sp = fig.add_subplot(111, aspect='equal')
        sp.set_xlim(_x_region_min , _x_region_max )
        sp.set_ylim(_y_region_min , _y_region_max )

        # draw particle set
        plt.quiver([pose.x for pose in self.pose_set], [pose.y for pose in self.pose_set], [math.cos(pose.theta) for pose in self.pose_set], [math.sin(pose._theta) for pose in self.pose_set], cmap="Greys", clim=(0.0, 1.0), label="particles",width=0.004)

        # draw robot
        robot_scale=10
        plt.quiver(self._previouse_pose.x, self._previouse_pose.y, math.cos(self._previouse_pose.theta), math.sin(self._previouse_pose.theta), scale=robot_scale, color="magenta", label="previous pose",width=0.01)
        plt.quiver(self._later_pose.x, self._later_pose.y, math.cos(self._later_pose.theta), math.sin(self._later_pose.theta), scale=robot_scale,color="red", label="next pose",width=0.01)
        # plt.legend(loc='right', bbox_to_anchor=(1.5, 0.5))

        save_png()

if __name__ == '__main__':
    test=TestMCL()
    default_parameter={'alpha1' : 1.0e-2, 'alpha2' : 1.0e-4,'alpha3':1.0e-1,"alpha4":1.0e-1}
    condition_list=[]
    condition_list.append({'alpha1' : 1.0e-2, 'alpha2' : 0.0,'alpha3':0.0,"alpha4":0.0})
    condition_list.append({'alpha1' : 0.0, 'alpha2' : 1.0e-4,'alpha3':0.0,"alpha4":0.0})
    condition_list.append({'alpha1' : 0.0, 'alpha2' : 0.0,'alpha3':1.0e-1,"alpha4":0.0})
    condition_list.append({'alpha1' : 0.0, 'alpha2' : 0.0,'alpha3':0.0,"alpha4":1.0e-1})
    condition_list.append(default_parameter)

    for condition in condition_list:
        test.test_sample_motion_model_odometory(15,0,0,**condition)
        test.test_sample_motion_model_odometory(0,45,45,**condition)
        test.test_sample_motion_model_odometory(15,45,45,**condition)
