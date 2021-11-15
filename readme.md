# ROS Beginner Tutorials - Introduction to Publisher and Subscriber  

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
git clone -b Week11_HW --single-branch https://github.com/anirudhtopiwala/beginner_tutorials.git
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
git clone -b Week11_HW --single-branch https://github.com/anirudhtopiwala/beginner_tutorials.git
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
roslaunch beginner_tutorials week11.launch 
```
* **By giving User Input to change the frequency of the talker node**

Go to your workspace in terminal
```
source devel/setup.bash
roslaunch beginner_tutorials week11.launch f:=<desired_frequency>
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
From now on, if the directory to run the code is not mentioned, it means it is being run from the workspace directory.

## How to call the service
Follow either one of the run instructions, that is, using a launch file or running each node invidually. This will start both the talker and listener nodes. Now open a new terminal and go to the workspace.
```
source devel/setup.bash
rosservice call /Change_String "input: '<any_string>'"
```  
## How to View the tf frames
The tf broadcaster in the talker node will create a tf transform between the /world frame and the /talk frame. This is a time varying broadcaster. It can be viewed by using tf_echo. Before running the below command, make sure that roscore and the talker node are running in separate terminals. (using steps 1 or 2 as listed above)
```
rosrun tf tf_echo /world /talk
```
To visualize this in a graph form, run the following command.
```
rosrun rqt_tf_tree rqt_tf_tree 
```
To view a similar graphical form in pdf format, run:
```
rosrun tf view_frames
```
To view the pdf generated, run:
```
evince frames.pdf
```
## How to run rostest
There are 2 tests written for this code. The first test checks if the service is being called correctly and the other one checks if the tf is being broadcasted correctly. The tests use rostest and gtest as base. 

To compile and run the tests:
```
catkin_make run_tests_beginner_tutorials
```
Once compiled, the following command can also be used:
```
source devel/setup.bash
rostest beginner_tutorials test.launch
```
or 
```
rosrun beginner_tutorials test_talker 
```
The above code will only work, if the talker node is active.

## How to record bag file by passing argumnets in launch file.

A ros bag file records all the topic and messages being published in the terminal. The below code will record the bag file and save it in the results direcory as talker.bag .

```
 roslaunch beginner_tutorials week11.launch record:=enable
```

## Inspecting the Bag file generated
To get more information about the generated rosbag file, open the results directory in terminal and run the following command:
```
rosbag info talker.bag
```
## Playing the rosbag file
The rosbag file as discussed before, can record all the messages being published on to the terminal. For example, lets play the bag file generated before of the talker node.

First we will need to start the listener node. (assumed that roscore is running)
```
source devel/setup.bash
rosrun beginner_tutorials listener
```
Now open the results folder in a new terminal and run:
```
rosbag play talker.bag
```
You will see that listener will start receiving messages being published on topic /chatter.
