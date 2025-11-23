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
if [ -z "$XAUTHORITY" ]; then
    xauth=""
else
    xauth='--env="XAUTHORITY=/home/ubuntu/.Xauthority"'
fi

# [ -e /dev/kfd ] && devs="--device=\"/dev/kfd\""
    # --env="XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR" \
xhost +local:root
podman run $gpus $xauth $opts \
    --env="SDL_VIDEODRIVER=x11" \
    --env="LIBGL_ALWAYS_INDIRECT=0" \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="NVIDIA_VISIBLE_DEVICES=all" \
    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    --device="/dev/dri" $devs \
    --group-add video \
    --volume="/dev/dri:/dev/dri:rw" \
    --volume="/usr/share/vulkan/icd.d:/usr/share/vulkan/icd.d" \
    --volume="/usr/share/vulkan/implicit_layer.d:/usr/share/vulkan/implicit_layer.d" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$HOME/.gazebo:/home/ubuntu/.gazebo" \
    --volume="$PWD/shared/:/mnt/shared/:rw" \
    --volume="$PWD/shared/catkin_ws:/home/ubuntu/catkin_ws:rw" \
    -it --privileged $image bash

