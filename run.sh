#!/bin/bash
if git rev-parse --is-inside-work-tree &>/dev/null; then
    PRJ_DIR=$(git rev-parse --show-toplevel) #repository root is the project directory
else
    PRJ_DIR=$(pwd) #current directory is the project diretory
fi

echo "PRJ_DIR: $PRJ_DIR"

if [ -z "$1" ]; then
    image="provsim"
else
    image=$1
fi

xhost +local:root
docker run --net=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --device="/dev/dri" \
    --group-add 44 \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$(pwd)/volume/src:/root/catkin_ws/src" \
    --volume="$(pwd)/volume/config:/root/config" \
    --volume="$HOME/.gazebo:/root/.gazebo" \
    -it $image bash

    # --volume="/usr/lib/x86_64-linux-gnu/dri:/usr/lib/x86_64-linux-gnu/dri" \
    # --volume="/usr/share/glvnd:/usr/share/glvnd" \
    # --volume="/usr/lib/x86_64-linux-gnu/libGL.so.1:/usr/lib/x86_64-linux-gnu/libGL.so.1" \
    # --volume="/usr/lib/x86_64-linux-gnu/libEGL.so.1:/usr/lib/x86_64-linux-gnu/libEGL.so.1" \
