#!/bin/bash

PROVANT_HOME=/root/catkin_ws/src/ProVANT-Simulator_Developer
mkdir -p $PROVANT_HOME/source/build
cd $PROVANT_HOME/source/build
qtchooser -qt=5 -run-tool=qmake $PROVANT_HOME/source/GUI/GUI.pro -r -spec linux-g++

