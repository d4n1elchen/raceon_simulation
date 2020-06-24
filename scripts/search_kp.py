#!/usr/bin/env python

import rospy
import roslaunch
import numpy as np
from launch_environment import SimulationLauncher

rospy.init_node('simulation_worker')

control_node = roslaunch.core.Node('raceon', 'control.py', 'control')
sl = SimulationLauncher(launch_file='raceon_simulation_pos_est.launch', track_file='final_track.txt', control_node=control_node, gui=True)

speed_list = np.linspace(200, 300, 10)
kp_list = np.linspace(1, 20, 10)

time = []
success = []

for speed in speed_list:
    t = []
    s = []

    for kp in kp_list:
        sl.set_param("/control/motor_speed", float(speed))
        sl.set_param("/control/kp", float(kp))
        sl.start()

        result = sl.get_last_result()
        t.append(result['time'])
        s.append(result['success'])

    time.append(t)
    success.append(s)
