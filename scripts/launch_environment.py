#!/usr/bin/env python

import rospkg
rospack = rospkg.RosPack()

import signal
import os
import rospy
import roslaunch
from std_msgs.msg import Int8
from raceon_simulation.msg import LapFinish

from track_generation import Track
from simulation_control import SimulationControl
from lap_timer import LapTimer

class SimulationLauncher:

    def __init__(self, launch_file, track_file, control_node=None, gui=False):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        self.control_node = control_node

        cli_args = ['raceon_simulation', launch_file]
        if gui:
            cli_args.append('gui:=true')
        roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
        roslaunch_args = cli_args[2:]

        self.launch = roslaunch.scriptapi.ROSLaunch()
        self.launch.parent = roslaunch.parent.ROSLaunchParent(uuid, [(roslaunch_file, roslaunch_args)])
        self.launch.start()

        # Lap finish detect
        rospy.Subscriber('/simulation/lap_finish', LapFinish, self.finish_callback, queue_size=10)

        # Initialize track
        filename = os.path.join(rospack.get_path('raceon_simulation'), 'track', track_file)
        self.track = Track(filename)
        self.track.spawn()

        # Start timer
        self.lap_timer = LapTimer(self.track)
        self.lap_timer.start()

        # Simulation control
        self.sim = SimulationControl(self.track)

        # History
        self.episode = 0
        self.result = []

        # Parameter list
        self.param = {}

        # Register interrupt handler
        self.interrupt = False
        def handler(signum, frame):
            self.interrupt = True
            self.stop()
        signal.signal(signal.SIGINT, handler)

    def start(self):
        if self.control_node != None:
            control_process = self.launch.launch(self.control_node)
        self.sim.reset()
        self.lap_timer.reset()
        self.sim.unpause()

        rospy.loginfo('Started a new episode')
        while not self.interrupt and not self.sim.is_paused:
            self.launch.spin_once()
        rospy.loginfo('One episode finished')
        if self.control_node != None:
            control_process.stop()

    def set_param(self, param, value):
        rospy.set_param(param, value)
        self.param[param] = value

    def stop(self):
        self.launch.stop()

    def finish_callback(self, msg):
        self.episode += 1
        self.result.append({'param': self.param, 'success': msg.success, 'time': msg.time})
        self.sim.pause()

    def get_last_result(self):
        return self.result[-1]

if __name__ == "__main__":
    rospy.init_node('simulation_worker')
    sl = SimulationLauncher(launch_file='raceon_simulation_pos_est_pid.launch', track_file='final_track.txt', gui=True)
    sl.set_param("/control/motor_speed", 200)
    sl.set_param("/control/kp", 10)
    try:
        sl.start()
    finally:
        sl.stop()
