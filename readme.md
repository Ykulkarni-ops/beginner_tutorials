# ROS Beginner Tutorials - Introduction to Publisher and Subscriber  
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
---

## Overview

This program is a basic introduction to a publisher and a subscriber in ROS. It creates two nodes, a talker (publisher) node and a listener (subscriber) node to demonstarte how they interact with each other.

## Dependencies
This program works on a device running Ubuntu 16.04 and ROS Kinetic Kame.

To install ROS Kinetic Kame in Ubuntu 16.04, follow the steps in this [link](http://wiki.ros.org/kinetic/Installation/Ubuntu).

To install catkin, follow the installation steps in this [link](http://wiki.ros.org/catkin).

## Build Instructions

To run this code in a catkin workspace:
```
cd ~/catkin_ws/
source devel/setup.bash
cd src/
git clone -b Week10_HW --single-branch https://github.com/anirudhtopiwala/beginner_tutorials.git
cd ..
catkin_make
```
If you do not have a catkin workspace:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone -b Week10_HW --single-branch https://github.com/anirudhtopiwala/beginner_tutorials.git
cd ..
catkin_make
```

## Run Instructions 

There are two ways of running the Demo. 

### 1) Using Launch File
* **Without Changing the Default Frequency of talker**

After following the build instructions:
Go to your workspace in terminal
```
source devel/setup.bash
roslaunch beginner_tutorials week10.launch 
```
* **By giving User Input to change the frequency of the talker node**

Go to your workspace in terminal
```
source devel/setup.bash
roslaunch beginner_tutorials week10.launch f:=<desired_frequency>
```

### 2) By running each node separately

After following the build instructions:
Go to your workspace in terminal

Run roscore in a new terminal keeping path as your current workspace:
```
source devel/setup.bash
roscore
```

Then run talker node from a new terminal keeping path as your current workspace:
```
source devel/setup.bash
rosrun beginner_tutorials talker
```
Finally run listener node from a new terminal keeping path as your current workspace:
```
source devel/setup.bash
rosrun beginner_tutorials listener
```
## How to call the service
Follow either one of the run instructions, that is, using a launch file or running each node invidually. This will start both the talker and listener nodes. Now open a new terminal and go to the workspace.
```
source devel/setup.bash
rosservice call /Change_String "input: '<any_string>'"
``` 

