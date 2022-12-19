# 2022 freshman_education

# Gazebo ROS Demos

* Author: Dave Coleman <davetcoleman@gmail.com>
* License: GNU General Public License, version 3 (GPL-3.0)

Example robots and code for interfacing Gazebo with ROS

## Install additional packages
$ sudo apt-get install ros-noetic-joint-state-controller \n
$ sudo apt-get install ros-noetic-effort-controllers \n
$ sudo apt-get install ros-noetic-position-controllers \n

## Tutorials

[ROS URDF](http://gazebosim.org/tutorials/?tut=ros_urdf)

## Quick Start

Rviz:

    roslaunch rrbot_description rrbot_rviz.launch

Gazebo:

    roslaunch rrbot_gazebo rrbot_world.launch

ROS Control:

    roslaunch rrbot_control rrbot_control.launch

Example of Moving Joints:

    rosrun rrbot_control control.py

## Develop and Contribute

We welcome any contributions to this repo and encourage you to fork the project then send pull requests back to this parent repo. Thanks for your help!
