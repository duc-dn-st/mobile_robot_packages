#!/bin/sh
set -e
basic_dep="python3-catkin-tools \
           python3-pip \
        "
ros_dep="ros-noetic-tf \
        libeigen3-dev \
        "

apt-get update
apt-get upgrade -y
apt-get install -y $basic_dep
DEBIAN_FRONTEND=noninteractive apt-get install -y $ros_dep
apt-get autoremove -y
apt-get clean -y
update-alternatives --install /usr/bin/python python /usr/bin/python3.8 1
python --version
