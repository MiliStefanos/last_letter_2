FROM ros:melodic-ros-base-bionic

# Use noninteractive debconf frontend since we're installing in a non-interactive way
ENV DEBIAN_FRONTEND noninteractive

ENV ROS_DISTRO melodic

SHELL [ "/bin/bash", "-c" ]

# Install required system packages
RUN apt-get -y update
RUN apt-get -y install apt-utils
RUN apt-get -y install sudo vim git wget
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
    wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - && \
    apt-get update && \
    apt-get install -y libignition-math4-dev

# Create user
RUN useradd -ms /bin/bash ros && \
    adduser ros sudo && \
    adduser root sudo && \
    echo "%sudo   ALL=(ALL:ALL) NOPASSWD: ALL" >> /etc/sudoers
# RUN adduser --gecos "ROS User" --disabled-password ros
# RUN usermod -a -G dialout ros

RUN echo ". /opt/ros/melodic/setup.bash" >> ~/.bashrc && \
    . /opt/ros/melodic/setup.bash && \
    mkdir -p ~/catkin_ws/src && \
    cd ~/catkin_ws/src && \
    catkin_init_workspace && \
    cd ../ && catkin_make -j4 && \
    echo ". /home/ros/catkin_ws/devel/setup.bash" >> ~/.bashrc

# RUN apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Create a catkin workspace with the package under integration.
# RUN mkdir -p ~/catkin_ws/src
# RUN cd ~/catkin_ws/src
# # RUN . /opt/ros/melodic/setup.bash
# RUN catkin_init_workspace
# 
# # Create the devel/setup.bash (run catkin_make with an empty workspace) and
# # source it to set the path variables.
# RUN cd ~/catkin_ws
# RUN catkin_make
# RUN source devel/setup.bash

# Download the package under integration, download from the internet
# RUN cd ~/catkin_ws/src && \
#     git clone https://github.com/Georacer/last_letter_2.git

# Copy the package under integration locally
COPY ./ /home/ros/catkin_ws/src/last_letter_2/
# Install package dependencies
RUN cd /home/ros/catkin_ws && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y
# Intermediate cache point to avoid repeating rosdep install, sadly doesn't work
RUN touch deleteme
RUN cd /home/ros/catkin_ws && \
    . /opt/ros/melodic/setup.bash && \
    catkin_make

# Compile and test (mark the build as failed if any step fails). If the
# CATKIN_OPTIONS file exists, use it as an argument to catkin_make, for example
# to blacklist certain packages.
#
# NOTE on testing: `catkin_make run_tests` will show the output of the tests
# (gtest, nosetest, etc..) but always returns 0 (success) even if a test
# fails. Running `catkin_test_results` aggregates all the results and returns
# non-zero when a test fails (which notifies Travis the build failed).
# RUN source /opt/ros/$ROS_DISTRO/setup.bash
# RUN cd ~/catkin_ws
# RUN catkin_make

# ENTRYPOINT [ "/ros_entrypoint.sh" ]
# CMD ["bash"]

USER ros
# HOME needs to be set explicitly. Without it, the HOME environment variable is
# set to "/"
ENV HOME /home/ros
WORKDIR $HOME

ENTRYPOINT [ "/ros_entrypoint.sh" ]
CMD [ "bash" ]