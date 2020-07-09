#!/usr/bin/env python

import rospy
import numpy as np
from std_srvs.srv import Empty
from gazebo_msgs.srv import SetModelState, DeleteModel, SpawnModel
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler

class SimulationControl:

    def __init__(self, track):
        self.model_state_proxy = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.reset_world_proxy = rospy.ServiceProxy('/gazebo/reset_world', Empty)

        # self.spawn_proxy = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        # self.delete_proxy = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

        self.pause_proxy = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.unpause_proxy = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.track = track

        self.is_paused = True

    def reset(self):
        # rospy.wait_for_service('/gazebo/reset_world', 5)
        # try:
        #     self.reset_world_proxy()
        # except (rospy.ServiceException) as e:
        #     print ("/gazebo/reset_world service call failed")

        rospy.wait_for_service('/gazebo/set_model_state', 5)
        ms = ModelState(model_name = 'racecar', pose = self.track.start_pose)
        try:
            self.model_state_proxy(model_state = ms)
        except (rospy.ServiceException) as e:
            print ("/gazebo/set_model_state service call failed")

    def pause(self):

        rospy.wait_for_service('/gazebo/pause_physics', 5)

        try:
            self.pause_proxy()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")

        self.is_paused = True

    def unpause(self):

        rospy.wait_for_service('/gazebo/unpause_physics', 5)

        try:
            self.unpause_proxy()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

        self.is_paused = False
