#!/bin/bash
set -e

if [ "$1" = '-h' ]; then
    echo 'usage: $0 [-u username] [image_name]'
    echo 'image_name: default is provant'
    echo '-u username: add username to the image instead of the default user (ubuntu)'
    echo ''
    echo 'builds the docker image with image_name name'
    exit 0
fi
if [ "$1" = '-u' ]; then
    shift
    user_cmd="--build-arg USER=$1"
    shift
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

podman build -t $image $user_cmd .
