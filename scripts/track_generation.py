#!/usr/bin/env python

import rospkg
rospack = rospkg.RosPack()

import copy
import rospy
import os
import math
import numpy as np
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler

WIDTH = LENGTH = 1.21
MODEL_PATH = os.path.join(rospack.get_path('racecar_description'), 'models')

def createSegment(identifier):

    if identifier == 'E':
        return BlankSegment()
    elif identifier == '-':
        return StraightSegment(90)
    elif identifier == 'I':
        return StraightSegment(0)
    elif identifier == '<':
        return TurnSegment(-180)
    elif identifier == 'V':
        return TurnSegment(-90)
    elif identifier == '>':
        return TurnSegment(0)
    elif identifier == '^':
        return TurnSegment(90)
    else:
        raise Exception('Invalid segment identifier')

class Segment(object):

    def __init__(self):
        self.rotate = 0
        self.width = WIDTH
        self.length = LENGTH

        self.edge_dist = 0.2
        self.track_width = self.width - self.edge_dist * 2

    def get_sdf(self):
        raise NotImplementedError()

    def test_linecross(self, position):
        offcenter = self.get_offcenter(position)
        return offcenter < (-self.track_width / 2) or offcenter > (self.track_width / 2)

    def get_offcenter(self, position):
        raise NotImplementedError()

class BlankSegment(Segment):

    def get_sdf(self):
        with open(os.path.join(MODEL_PATH, 'segment_blank', 'model.sdf'), 'r') as f:
            sdf = f.read()
        return sdf

    def get_offcenter(self, position):
        # The car shouldn't step on the blank segment, give a big number
        return 1e9

class StraightSegment(Segment):

    def __init__(self, rotate):
        super(StraightSegment, self).__init__()
        self.rotate = rotate

    def get_sdf(self):
        with open(os.path.join(MODEL_PATH, 'segment_straight', 'model.sdf'), 'r') as f:
            sdf = f.read()
        return sdf

    def get_offcenter(self, position):
        if self.rotate == 0:
            return position.y
        elif self.rotate == 90:
            return position.x

class TurnSegment(Segment):

    def __init__(self, rotate):
        super(TurnSegment, self).__init__()
        self.rotate = rotate

    def get_sdf(self):
        with open(os.path.join(MODEL_PATH, 'segment_turn', 'model.sdf'), 'r') as f:
            sdf = f.read()
        return sdf

    def get_offcenter(self, position):
        r, th = self.get_polar((position.x, position.y))
        return r - self.width / 2

    def get_polar(self, point):
        px, py = point

        angle = math.radians(90 - self.rotate)
        qx = math.cos(angle) * px - math.sin(angle) * py
        qy = math.sin(angle) * px + math.cos(angle) * py

        qx += self.width / 2
        qy += self.length / 2

        r = math.sqrt(qx**2 + qy**2)
        theta = math.atan2(qy, qx) * 180 / math.pi

        return r, theta

class Track:

    def __init__(self, filename):
        self.filename = filename
        self.pattern = []
        self.start_pose = Pose()

        self.finish_line_size = [WIDTH, 0.5]
        self.finish_line = [0, 0, 0, 0]

        self.spawn_proxy = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.load_track()

    def load_track(self):
        with open(self.filename, 'r') as f:
            for i, line in enumerate(f):
                row = []
                for j, cell in enumerate(line[:-1].split(',')):
                    row.append(createSegment(cell[0]))

                    if len(cell) > 1 and cell[1] == 'S':
                        if cell[0] not in ['I', '-']:
                            raise Exception('Invalid staring point')

                        self.start_pose.position.x = i * WIDTH
                        self.start_pose.position.y = j * LENGTH

                        if cell[0] == 'I':
                            self.finish_line[0] = self.start_pose.position.x - self.finish_line_size[0] / 2
                            self.finish_line[1] = self.start_pose.position.y - self.finish_line_size[1] / 2
                            self.finish_line[2] = self.start_pose.position.x + self.finish_line_size[0] / 2
                            self.finish_line[3] = self.start_pose.position.y + self.finish_line_size[1] / 2
                            rotate = 180
                        elif cell[0] == '-':
                            self.finish_line[0] = self.start_pose.position.x - self.finish_line_size[1] / 2
                            self.finish_line[1] = self.start_pose.position.y - self.finish_line_size[0] / 2
                            self.finish_line[2] = self.start_pose.position.x + self.finish_line_size[1] / 2
                            self.finish_line[3] = self.start_pose.position.y + self.finish_line_size[0] / 2
                            rotate = 90

                        q = quaternion_from_euler(0, 0, math.radians(rotate))
                        self.start_pose.orientation.x = q[0]
                        self.start_pose.orientation.y = q[1]
                        self.start_pose.orientation.z = q[2]
                        self.start_pose.orientation.w = q[3]

                self.pattern.append(row)

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
                q = quaternion_from_euler(0, 0, math.radians(seg.rotate))
                initial_pose.orientation.x = q[0]
                initial_pose.orientation.y = q[1]
                initial_pose.orientation.z = q[2]
                initial_pose.orientation.w = q[3]

                try:
                    res = self.spawn_proxy(model_name = model_name, model_xml = model_xml, initial_pose = initial_pose)
                except (rospy.ServiceException) as e:
                    print ("/gazebo/spawn_sdf_model service call failed")

                y += LENGTH
            y = 0
            x += WIDTH


    def test_terminate(self, position):
        if position.x >= self.finish_line[0] and position.x <= self.finish_line[2] \
                and position.y >= self.finish_line[1] and position.y <= self.finish_line[3]:
            return True
        return False

    def test_linecross(self, position):
        row = int(position.x / WIDTH + 0.5)
        col = int(position.y / LENGTH + 0.5)

        if row < 0 or col < 0 or row >= len(self.pattern) or col >= len(self.pattern[row]):
            print(row, col)
            return True

        local_pos = copy.deepcopy(position)
        local_pos.x -= row * WIDTH
        local_pos.y -= col * LENGTH

        return self.pattern[row][col].test_linecross(local_pos)

if __name__ == "__main__":
    filename = os.path.join(rospack.get_path('raceon_simulation'), 'track', 'final_track.txt')
    track = Track(filename)
