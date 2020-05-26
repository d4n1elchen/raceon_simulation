#!/usr/bin/python3

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Int8

class LapTimer:

    def __init__(self):
        self.started = False
        self.start_time = 0
        self.lap_count = 0

    def start(self):
        self.pub_lap_count = rospy.Publisher('/simulation/lap_count', Int8, queue_size=10)
        rospy.Subscriber('/vesc/odom', Odometry, self.pos_detector, queue_size=20)
        rospy.spin()

    def pos_detector(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if (x > -0.05 and x < 0.05 and y > -0.1 and y < 0.1):
            if self.started:
                self.lap_count += 1
                self.pub_lap_count.publish(Int8(self.lap_count))
                rospy.loginfo("Lap: {}, time = {}".format(self.lap_count, rospy.get_time() - self.start_time))
                self.started = False
        elif self.started == False:
            rospy.loginfo("Lap {} started".format(self.lap_count + 1))
            self.started = True
            self.start_time = rospy.get_time()

if __name__ == "__main__":
    rospy.init_node('lap_timer')
    lap_timer = LapTimer()
    lap_timer.start()
