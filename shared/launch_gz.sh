#!/bin/bash

set -e

cd /mnt/shared/sim_quad/cpp/build/Debug
cmake -DCMAKE_BUILD_TYPE=Debug ../..
ninja -j4
sudo ninja install

cd ~/catkin_ws
catkin_make

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

roslaunch Database gazebo.launch world:=/home/ubuntu/catkin_ws/src/ProVANT-Simulator_Developer/source/Database/worlds/worlds/iuro/quad.world control_strategy:=/home/ubuntu/catkin_ws/src/ProVANT-Simulator_Developer/source/Database/models/quadcopter/config/config.xml
