#!/usr/bin/env python

import rospkg
rospack = rospkg.RosPack()

import rospy
import os
import numpy as np
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler

class Segment:

    def __init__(self, type, rotate = 0):
        self.type = type
        self.model_path = os.path.join(rospack.get_path('racecar_description'), 'models')
        self.rotate = rotate
        self.width = 1.21
        self.length = 1.21

    def get_sdf(self):
        with open(os.path.join(self.model_path, 'segment_{}'.format(self.type), 'model.sdf'), 'r') as f:
            sdf = f.read()
        return sdf

class Track:

    def __init__(self, filename):
        self.filename = filename
        self.pattern = []
        self.start = [0, 0]

        self.spawn_proxy = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.load_track()

    def load_track(self):
        with open(self.filename, 'r') as f:
            for i, line in enumerate(f):
                row = []
                for j, cell in enumerate(line[:-1].split(',')):
                    if cell[0] == 'E':
                        row.append(Segment('blank'))
                    elif cell[0] == '-':
                        row.append(Segment('straight', np.radians(90)))
                    elif cell[0] == 'I':
                        row.append(Segment('straight', np.radians(0)))
                    elif cell[0] == '<':
                        row.append(Segment('turn', np.radians(-180)))
                    elif cell[0] == 'V':
                        row.append(Segment('turn', np.radians(-90)))
                    elif cell[0] == '>':
                        row.append(Segment('turn', np.radians(0)))
                    elif cell[0] == '^':
                        row.append(Segment('turn', np.radians(90)))

                    if len(cell) > 1 and cell[1] == 'S':
                        self.start = [i, j]

                self.pattern.append(row)

        print(self.pattern)

    def spawn(self):
        rospy.wait_for_service('/gazebo/spawn_sdf_model')

        x = 0
        y = 0
        for i, row in enumerate(self.pattern):
            for j, seg in enumerate(row):
                model_name = '{}_{}'.format(i, j)
                model_xml = seg.get_sdf()
                initial_pose = Pose()
                initial_pose.position.x = x
                initial_pose.position.y = y
                q = quaternion_from_euler(0, 0, seg.rotate)
                initial_pose.orientation.x = q[0]
                initial_pose.orientation.y = q[1]
                initial_pose.orientation.z = q[2]
                initial_pose.orientation.w = q[3]

                try:
                    res = self.spawn_proxy(model_name = model_name, model_xml = model_xml, initial_pose = initial_pose)
                except (rospy.ServiceException) as e:
                    print ("/gazebo/spawn_sdf_model service call failed")

                y += row[0].width
            y = 0
            x += row[0].length


    def test_terminate(self, position):
        pass

    def test_offcenter(self, position):
        pass

if __name__ == "__main__":
    filename = os.path.join(rospack.get_path('raceon_simulation'), 'track', 'final_track.txt')
    track = Track(filename)
