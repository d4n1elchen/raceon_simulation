#!/usr/bin/env python

import rospy
import roslaunch
import numpy as np
from std_msgs.msg import Int8
from raceon_simulation.msg import LapFinish

rospy.init_node('simulation_worker', anonymous=True)

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

launch = roslaunch.scriptapi.ROSLaunch()

# Start timer node
node = roslaunch.core.Node('raceon_simulation', 'lap_timer.py', output='screen')

time = []
success = []
Kp = np.arange(1, 31, 3)

def finish_callback(msg):
    success.append(msg.success)
    time.append(msg.time)
    launch.stop()

rospy.Subscriber('/simulation/lap_finish', LapFinish, finish_callback, queue_size=10)

for kp in Kp:
    cli_args = ['raceon_simulation', 'raceon_simulation_pos_est_pid.launch', '--log', 'speed:=200', 'kp:={}'.format(kp)]
    roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
    roslaunch_args = cli_args[2:]

    launch.parent = roslaunch.parent.ROSLaunchParent(uuid, [(roslaunch_file, roslaunch_args)])

    launch.start()
    launch.launch(node)
    try:
        launch.spin()
    finally:
        # After Ctrl+C, stop all nodes from running
        launch.stop()

print(Kp)
print(success)
print(time)
