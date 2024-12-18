# Multi-Robot Simulation Environment (Traditional Navi-Stack)
This repository contains the necessary elements to launch and run a multi-robot 
simulation environment in Gazebo using traditional navigation methods provided by the 
ROS Navigation Stack. 

The environment is configured to launch three robots, although this can be further 
extended. One of the robots uses A* as the global planner, and the two other robots 
user Dijkstra. 

## Contents   
1. [Build & Install](#build--install)   
2. [Launch](#launch) 

## Build & Install

From the DRL multi-robot tutorial, you should have ROS Melodic and Ubuntu 18 
installed on your machine. Then, you have to create your own ROS workspace 
and clone your repository. 
```bash
mkdir -p ~/traditional_ws/src
cd ~/traditional_ws/
catkin_make
source devel/setup.bash
cd src
git clone https://github.com/mateopozor02/traditional-multirobot-env.git
```
All the dependencies will be installed from the DRL tutorial. Thus, you don't 
need to reinstall.

## Launch
To launch this workspace, you have to open three terminals. In the first terminal, 
you will initialize roscore: 

```bash
roscore
```

On your second terminal, you will then initialize the simulation in Gazebo with 
the three robots. This is done from `~/traditional_ws/src/turtlebot3_simulations`:

```bash
roslaunch turtlebot3_gazebo multi_turtlebot3_world_track.launch 
```

Finally, on your third terminal, you will launch RViz and the localization 
algorithms for the robot, that will ultimately allow it to move. This is done 
from `~/traditional_ws/src/turtlebot3`:

```bash
roslaunch turtlebot3_navigation turtlebot3_navigation_multi.launch

```
