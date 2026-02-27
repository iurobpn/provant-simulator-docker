#!/bin/bash

set -e

cd /mnt/shared/sim_quad/cpp/build/Release
# cmake -DCMAKE_BUILD_TYPE=Debug ../..
ninja -j4
sudo ninja install
sudo ldconfig

source /opt/ros/noetic/setup.bash

cd /mnt/shared/catkin_ws
catkin_make

source devel/setup.bash

# roslaunch Database gazebo.launch world:=/mnt/shared/catkin_ws/src/ProVANT-Simulator_Developer/source/Database/worlds/worlds/iuro/quad_20obs.world control_strategy:=/mnt/shared/catkin_ws/src/ProVANT-Simulator_Developer/source/Database/models/quadcopter/config/config.xml
roslaunch Database gazebo.launch world:=/mnt/shared/catkin_ws/src/ProVANT-Simulator_Developer/source/Database/worlds/worlds/iuro/husky_1obs/husky_rnd_color.world control_strategy:=/mnt/shared/catkin_ws/src/ProVANT-Simulator_Developer/source/Database/models/husky2/config/config.xml
