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
sudo apt-get install ros-melodic-ros-control ros-melodic-gazebo-ros-control ros-melodic-ros-controllers python3-opencv
```

## Clone dependencies

```shell
cd ~/raceon_sim_ws/src
git clone https://github.com/wjwwood/serial.git
git clone https://github.com/ros-drivers/ackermann_msgs.git
git clone https://github.com/mit-racecar/racecar.git
git clone https://github.com/mit-racecar/vesc.git
git clone https://github.com/d4n1elchen/raceon-ros.git raceon
git clone https://github.com/d4n1elchen/raceon_simulation.git
git clone https://github.com/d4n1elchen/raceon_visualizer.git
git clone https://github.com/d4n1elchen/racecar_gazebo.git
```

## Build

```
cd ~/raceon_sim_ws
catkin_make
```

## Run

```
roslaunch raceon_simulation raceon_simulation.launch speed:=180 kp:=10
```

If you got python module not found error, install missing packages using `pip3`.
