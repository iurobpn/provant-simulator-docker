#!/bin/bash

set -e

cd /mnt/shared/sim_quad/cpp/
cmake -B build/Release -DCMAKE_BUILD_TYPE=Release
cmake --build build/Release -j 6
cd build/Release
sudo ninja install
# cmake -B build/Release --target install

cd ~/catkin_ws
catkin_make

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

roslaunch Database gazebo.launch world:=/home/ubuntu/catkin_ws/src/ProVANT-Simulator_Developer/source/Database/worlds/worlds/iuro/quad.world control_strategy:=/home/ubuntu/catkin_ws/src/ProVANT-Simulator_Developer/source/Database/models/quadcopter/config/config.xml
