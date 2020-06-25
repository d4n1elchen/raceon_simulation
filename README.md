# Race On! Simulation

## ROS version

ROS Melodic

Make sure you have gazebo installed. If you install your ROS with `desktop-full` release, you should be fine.

## Make a workspace for simulation

```shell
mkdir -p ~/raceon_sim_ws/src
cd ~/raceon_sim_ws
catkin_make
```

## Source the workspace 

```
source ~/raceon_sim_ws/devel/setup.bash
```

(Optional) Add it to bashrc
```
echo "source ~/raceon_sim_ws/devel/setup.bash" >> ~/.bashrc
```

## Install dependencies

```shell
sudo apt-get install ros-melodic-ros-control ros-melodic-gazebo-ros-control ros-melodic-ros-controllers python-opencv ros-melodic-ackermann-msgs python-empy
pip install pynput
```

## Clone dependencies

```shell
cd ~/raceon_sim_ws/src
git clone https://github.com/wjwwood/serial.git
git clone https://github.com/ros-drivers/ackermann_msgs.git
git clone https://github.com/mit-racecar/racecar.git
git clone https://github.com/mit-racecar/vesc.git
git clone https://github.com/d4n1elchen/raceon.git
git clone https://github.com/d4n1elchen/raceon_simulation.git
git clone https://github.com/d4n1elchen/raceon_visualizer.git
git clone https://github.com/d4n1elchen/racecar_gazebo.git
```

## Build

```
cd ~/raceon_sim_ws
catkin_make
```

## Start simulation

```
cd scripts
python launch_environment.py 
```

If you got python module not found error, install missing packages using `pip`.

## Launch simulation by python api with lap timer and custom track recipe.

Create script in `scripts` folder

### Example
`scripts/search_kp.py`
```python
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
```

## Track recipe

We use one character to represent the type of track segments and put an `S` right after the starting segment. Segments are seperated by comma.

| Track identifier | mearning |
|------------------|----------|
| `E` | Empty |
| `I` | Vertical straight lane |
| `-` | Horizontal straight lane |
| `<` | Turn lane with shape "┐" |
| `V` | Turn lane with shape "┌" |
| `>` | Turn lane with shape "└" |
| `^` | Turn lane with shape "┘" |

Currently, the starting identifier only can be put on straight lane and the car will always face toward north if it is a vertical lane, and face torward east for horizontal one.

### Example
`track/final_track.txt`
```
E,E,E,V,<
E,E,E,I,I
V,-,-,^,IS
>,-,-,-,^
```

#### Result
![](https://imgur.com/yTZc7fE.png)
