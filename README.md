# 2023 freshman_education

# Gazebo ROS Demos

* Author: Dave Coleman <davetcoleman@gmail.com>
* License: GNU General Public License, version 3 (GPL-3.0)

## Install Gazebo pkgs (20.04/18.04/16.04 -> noetic/melodic/kinetic)
    $ sudo apt-get update
    
    $ sudo apt-get install ros-(distro)-gazebo-ros-pkgs ros-(distro)-gazebo-plugins ros-(distro)-gazebo-ros-control

Example robots and code for interfacing Gazebo with ROS

## Install additional packages 
    $ sudo apt-get install ros-(distro)-joint-state-controller

## Before Start

    $ cd catkin_ws/src/freshman_education/rrbot_control/scripts
    $ chmod +x *

## Quick Start

Gazebo:

    roslaunch rrbot_gazebo rrbot_world.launch

ROS Control:

    roslaunch rrbot_control rrbot_control.launch

Example of Moving Joints:

    rosrun rrbot_control position_control.py

## Develop and Contribute

We welcome any contributions to this repo and encourage you to fork the project then send pull requests back to this parent repo. Thanks for your help!

