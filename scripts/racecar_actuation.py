#!/usr/bin/env python
# license removed for brevity

import rospy
import numpy as np
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped

SERVO_LEFT = -900
SERVO_RIGHT = 900
SERVO_MID = 0

class Actuator():

    def __init__(self):
        self.topic_name_control = rospy.get_param("topic_name_control", "control")
        self.topic_name_vesc_teleop = rospy.get_param("topic_name_vesc_teleop", "/vesc/ackermann_cmd_mux/input/teleop")

    def start(self):
        self.sub_control = rospy.Subscriber(self.topic_name_control, AckermannDrive, self.control_callback)
        self.pub_vesc = rospy.Publisher(self.topic_name_vesc_teleop, AckermannDriveStamped, queue_size=10)
        rospy.spin()

    def stop(self):
        rospy.loginfo("Stop the car ...")
        self.drive(0, 0)

    def control_callback(self, control_msg):
        servo_pos = control_msg.steering_angle
        motor_speed = control_msg.speed

        rospy.loginfo("Control command received: servo_pos = " + str(servo_pos) + ", motor_speed = " + str(motor_speed))

        self.drive(motor_speed, servo_pos)


    def drive(self, motor_speed, servo_pos):
        # servo command map to degree [-30, 30] in radius
        servo_radius = - servo_pos/SERVO_RIGHT * 30 * np.pi/180

        # speed gain
        motor_speed /= 200

        rospy.loginfo("Convert into vesc command: steering_angle = " + str(servo_radius) + ", speed = " + str(motor_speed))

	msg = AckermannDriveStamped();
	msg.header.stamp = rospy.Time.now();
	msg.header.frame_id = "base_link";

	msg.drive.speed = motor_speed
	msg.drive.steering_angle = servo_radius

	self.pub_vesc.publish(msg)

if __name__ == "__main__":

    rospy.init_node("racecar_actuation")
    actuator = Actuator()
    try:
        actuator.start()
    except rospy.ROSInterruptException:
        pass
    actuator.stop()
