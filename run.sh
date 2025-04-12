#!/bin/bash
if git rev-parse --is-inside-work-tree &>/dev/null; then
    PRJ_DIR=$(git rev-parse --show-toplevel) #repository root is the project directory
else
    PRJ_DIR=$(pwd) #current directory is the project diretory
fi

echo "PRJ_DIR: $PRJ_DIR"

if [ -z "$1" ]; then
    image="prosim:latest"
else
    image=$1
fi

xhost +
docker run --net=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$PRJ_DIR/volume/src:/root/catkin_ws/src" \
    --volume="$PRJ_DIR/volume/config:/root/config" \
    -it $image bash

