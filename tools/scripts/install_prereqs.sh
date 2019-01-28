#!/bin/bash

set -e

# Install required system packages
apt-get -y -q update
# apt-get -y install apt-utils && \
apt-get -y -q install sudo vim git wget
apt-get -y -q install ros-${ROS_DISTRO}-gazebo-ros-pkgs

# Install gazebo and its dependencies
# Choose version depending on ROS version

# # Unneeded, installed with gazebo-ros-pkgs
# case $ROS_DISTRO in
#     'kinetic')
#         echo '>>> Installing Gazebo 7'
#         apt-get install -y gazebo7
#         ;;
#     'melodic')
#         echo '>>> Installing Gazebo 9'
#         apt-get install -y gazebo9
#         ;;
#     *)
#         echo ">>> Unsupported ROS version: " $ROS_DISTRO
#         ;;
# esac

# # Create rosdep registry
# source /opt/ros/${ROS_DISTRO}/setup.bash
# rosdep init
    
# if [$ROS_DISTRO = "kinetic"]
# then
#     apt-get install gazebo7
# elif [$ROS_DISTRO = "melodic"]
# then
#     apt-get install gazebo9
# else
#     echo "Unsupported ROS version: " $ROS_DISTRO
# fi

# RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
#     wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - && \
#     apt-get update && \

    # apt-get install -y libignition-math4-dev