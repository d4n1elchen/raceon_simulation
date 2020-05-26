#!/usr/bin/python3

import rospy
from nav_msgs.msg import Odometry

class LapTimer:

    def __init__(self):
        self.started = False
        self.start_time = 0

    def start(self):
        rospy.Subscriber('/vesc/odom', Odometry, self.pos_detector, queue_size=10)
        rospy.spin()

    def pos_detector(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if (x > -0.05 and x < 0.05 and y > -0.1 and y < 0.1):
            if self.started:
                rospy.loginfo(str(rospy.get_time() - self.start_time) + "!!!!!")
                self.started = False
        elif self.started == False:
            self.started = True
            self.start_time = rospy.get_time()

if __name__ == "__main__":
    rospy.init_node('lap_timer')
    lap_timer = LapTimer()
    lap_timer.start()
