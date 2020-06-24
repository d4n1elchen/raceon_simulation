#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Int8
from raceon_simulation.msg import LapFinish

class LapTimer:

    def __init__(self, track):
        self.track = track
        self.reset()

    def reset(self):
        self.started = False
        self.start_time = 0
        self.lap_count = 0
        self.success = True

    def start(self):
        self.pub_lap_count = rospy.Publisher('/simulation/lap_count', Int8, queue_size=10)
        self.pub_lap_finish = rospy.Publisher('/simulation/lap_finish', LapFinish, queue_size=10)
        rospy.Subscriber('/vesc/odom', Odometry, self.pos_detector, queue_size=20)

    def lap_finish(self):
        time = rospy.get_time() - self.start_time
        self.pub_lap_finish.publish(LapFinish(self.success, time))
        rospy.loginfo("Lap: {} {}, time = {}".format(self.lap_count, 'success' if self.success else 'failed', time))

    def pos_detector(self, msg):
        position = msg.pose.pose.position

        if self.track.test_linecross(position):
            if self.started and self.success:
                self.success = False
                self.started = False
                self.lap_finish()

        elif self.track.test_terminate(position):
            if self.started:
                self.lap_count += 1
                self.pub_lap_count.publish(Int8(self.lap_count))

                self.success = True
                self.started = False
                self.lap_finish()

        elif self.started == False:
            rospy.loginfo("Lap {} started".format(self.lap_count + 1))
            self.started = True
            self.start_time = rospy.get_time()

if __name__ == "__main__":
    rospy.init_node('lap_timer')
    lap_timer = LapTimer()
    lap_timer.start()
