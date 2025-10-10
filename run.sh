#!/bin/bash
if git rev-parse --is-inside-work-tree &>/dev/null; then
    PRJ_DIR=$(git rev-parse --show-toplevel) #repository root is the project directory
else
    PRJ_DIR=$(pwd) #current directory is the project diretory
fi

echo "PRJ_DIR: $PRJ_DIR"

if [ "$1" = '-h' ]; then
    echo 'usage: $0 [-g] [image_name]'
    echo '-g: include gpus passthrough for graphics and Gazebo simulation'
    echo 'image_name: default is provant'
    echo ''
    echo 'launches a container of image_name|provant with graphical integration with the host system'
    echo 'shares $PWD/shared/catkin_ws with the container for persistence'
    exit 0
fi

if [ "$1" = '-g' ]; then
    gpus="--gpus all"
    shift
else
    gpus=""
fi
if [ -z "$1" ]; then
    image=provant
else
    image=$1
    shift
fi


opts="$*"



    # --env="XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR" \
xhost +local:root
docker run --net=host $gpus $opts \
    --env="SDL_VIDEODRIVER=x11" \
    --env="LIBGL_ALWAYS_INDIRECT=0" \
    -v /usr/share/vulkan/icd.d:/usr/share/vulkan/icd.d \
    -v /usr/share/vulkan/implicit_layer.d:/usr/share/vulkan/implicit_layer.d \
    --env="XAUTHORITY=/root/.Xauthority" \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="NVIDIA_VISIBLE_DEVICES=all" \
    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    --device="/dev/dri" \
    --device="/dev/kfd" \
    --volume="/dev/dri:/dev/dri:rw" \
    --group-add 44 \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$HOME/.gazebo:/root/.gazebo" \
    --volume="$XAUTHORITY:/root/.Xauthority" \
    --volume="$PWD/shared/catkin_ws:/root/catkin_ws" \
    -it --privileged $image bash

