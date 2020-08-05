#!/usr/bin/env python

import rospkg
rospack = rospkg.RosPack()

import signal
import time
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

        self.finished = False

    def start(self):
        self.lap_timer.pause()
        self.sim.pause()

        self.lap_timer.reset()
        self.sim.reset()

        self.lap_timer.unpause()
        self.sim.unpause()
        self.finished = False

        if self.control_node != None:
            self.control_process = self.launch.launch(self.control_node)

        rospy.loginfo('Started a new episode')
        while not self.interrupt and not self.finished:
            self.launch.spin_once()

        if self.control_node != None:
            self.control_process.stop()
            time.sleep(1)
            while self.control_process.is_alive():
                pass

        self.lap_timer.pause()
        self.sim.pause()

        rospy.loginfo('One episode finished')

    def set_param(self, param, value):
        rospy.set_param(param, value)
        self.param[param] = value

    def stop(self):
        self.launch.stop()

    def finish_callback(self, msg):
        rospy.loginfo('Finish message received')

        self.episode += 1
        self.result.append({'param': self.param, 'success': msg.success, 'time': msg.time})

        self.finished = True

    def get_last_result(self):
        return self.result[-1]

if __name__ == "__main__":
    rospy.init_node('simulation_worker')
    sl = SimulationLauncher(launch_file='raceon_simulation_pos_est_pid.launch', track_file='final_track.txt', gui=False)
    sl.set_param("/control/motor_speed", 200)
    sl.set_param("/control/kp", 10)
    try:
        sl.start()
    finally:
        sl.stop()
