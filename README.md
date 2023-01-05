# 2022 freshman_education

# Gazebo ROS Demos

* Author: Dave Coleman <davetcoleman@gmail.com>
* License: GNU General Public License, version 3 (GPL-3.0)

## Install Gazebo
    $ sudo apt-get update
    
    // for ubuntu 16.04
    $ sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-plugins ros-kinetic-gazebo-ros-control
    // for ubuntu 18.04
    $ sudo apt-get install ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-plugins ros-melodic-gazebo-ros-control
    // for ubuntu 20.04
    $ sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-plugins ros-noetic-gazebo-ros-control

Example robots and code for interfacing Gazebo with ROS

## Install additional packages
    $ sudo apt-get install ros-noetic-joint-state-controller
    $ sudo apt-get install ros-noetic-effort-controllers
    $ sudo apt-get install ros-noetic-position-controllers

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

