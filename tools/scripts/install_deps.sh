#!/bin/bash

set -e

# Install package dependencies
cd /home/ros/catkin_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

ls -al /home/ros/catkin_ws/src
ls -al /home/ros/catkin_ws/devel