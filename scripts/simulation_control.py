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
        self.pause_proxy = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.unpause_proxy = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.track = track

    def reset(self):
        rospy.wait_for_service('/gazebo/pause_physics')

        pose = Pose()
        pose.position.x = self.track.start[0]
        pose.position.y = self.track.start[1]
        q = quaternion_from_euler(0, 0, np.radians(180))
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        ms = ModelState(model_name = 'racecar', pose = pose)

        try:
            self.model_state_proxy(model_state = ms)
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")

    def pause(self):
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause_proxy()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")

    def unpause(self):
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause_proxy()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")
