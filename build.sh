#!/bin/bash
if [ "$1" = '-h' ]; then
    echo 'usage: $0 [image_name]'
    echo 'image_name: default is provant'
    echo ''
    echo 'builds the docker image with image_name name'
    exit 0
fi
if [ -z "$1" ]; then
    image=provant
else
    image=$1
fi
mkdir -p shared/catkin_ws/src
cd shared/catkin_ws/src
if ! [ -d ./shared/catkin_ws/src/ProVANT-Simulator_Developer ] && ! git clone git@github.com:Guiraffo/ProVANT-Simulator_Developer.git; then
    echo "SSH clone failed, setup ssh keys and add to your github account or use https instead"
    echo "Image will build with dependencies only"
    echo "run ./install.sh after container is running"
fi
cd -
docker build -t $image .
