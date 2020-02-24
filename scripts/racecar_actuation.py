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

	msg = AckermannDriveStamped();
	msg.header.stamp = rospy.Time.now();
	msg.header.frame_id = "base_link";

	msg.drive.speed = 0
	msg.drive.acceleration = 1
	msg.drive.jerk = 1
	msg.drive.steering_angle = 0
	msg.drive.steering_angle_velocity = 1

	self.pub_vesc.publish(msg)

    def control_callback(self, control_msg):
        servo_pos = control_msg.steering_angle
        control_speed = control_msg.speed

        servo_radius = servo_pos/SERVO_LEFT * np.pi
        # limit [-60, 60] degs
        if servo_radius > 45*np.pi/180:
		servo_radius = 45*np.pi/180
	if servo_radius < -45*np.pi/180:
		servo_radius = -45*np.pi/180

        motor_speed = control_speed / 200

        rospy.loginfo("Control command received: servo_pos = " + str(servo_pos) + ", motor_speed = " + str(control_speed))
        rospy.loginfo("Convert into vesc command: steering_angle = " + str(servo_radius) + ", speed = " + str(motor_speed))

	msg = AckermannDriveStamped();
	msg.header.stamp = rospy.Time.now();
	msg.header.frame_id = "base_link";

	msg.drive.speed = motor_speed
	msg.drive.acceleration = 100 * np.sign(motor_speed)
	msg.drive.jerk = 0
	msg.drive.steering_angle = servo_radius
	msg.drive.steering_angle_velocity = 100 * np.sign(servo_radius)

	self.pub_vesc.publish(msg)

if __name__ == "__main__":

    rospy.init_node("racecar_actuation")
    actuator = Actuator()
    try:
        actuator.start()
    except rospy.ROSInterruptException:
        pass
    actuator.stop()
