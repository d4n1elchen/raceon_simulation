#!/usr/bin/env python

import rospy
import numpy as np
from std_srvs.srv import Empty
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler

class SimulationControl:

    def __init__(self, track):
        self.model_state_proxy = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.reset_simulation_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.pause_proxy = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.unpause_proxy = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.track = track

        self.is_paused = True

    def reset(self):

        # rospy.wait_for_service('/gazebo/reset_simulation')
        # try:
        #     self.reset_simulation_proxy()
        # except (rospy.ServiceException) as e:
        #     print ("/gazebo/reset_simulation service call failed")

        rospy.wait_for_service('/gazebo/set_model_state')
        ms = ModelState(model_name = 'racecar', pose = self.track.start_pose)
        try:
            self.model_state_proxy(model_state = ms)
        except (rospy.ServiceException) as e:
            print ("/gazebo/set_model_state service call failed")

    def pause(self):
        rospy.wait_for_service('/gazebo/pause_physics')

        try:
            self.pause_proxy()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")

        self.is_paused = True

    def unpause(self):
        rospy.wait_for_service('/gazebo/unpause_physics')

        try:
            self.unpause_proxy()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

        self.is_paused = False
