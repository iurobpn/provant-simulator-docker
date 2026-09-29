#!/usr/bin/env bash

if git rev-parse --is-inside-work-tree &>/dev/null; then
    PRJ_DIR=$(git rev-parse --show-toplevel) #repository root is the project directory
else
    PRJ_DIR=$(pwd) #current directory is the project diretory
fi
PRJ=$(basename $PRJ_DIR)

echo "PRJ_DIR: $PRJ_DIR"
image="prov"
# gpus="--device /dev/dri"
net="--net=host --uts=host"
for arg in "$@"; do
    case "$arg" in
        '-h')
            echo 'usage: $0 [-g] [image_name]'
            echo '-g: include gpus passthrough for graphics and Gazebo simulation'
            echo 'image_name: default is provant'
            echo ''
            echo 'launches a container of image_name|provant with graphical integration with the host system'
            echo 'shares $PWD/shared/rmf_ws with the container for persistence'
            exit 0
            ;;
        '-g')
            gpus="--device nvidia.com/gpu=all --env=\"NVIDIA_DRIVER_CAPABILITIES=all\""
            shift
            ;;
        "-d")
            debug="--log-level=debug"
            shift
            ;;
        "-ip")
            net=""
            shift
            ;;
        *)
            image=$arg
            shift
            ;;
    esac
done

# opts="$*"
if [ -z "$XAUTHORITY" ]; then
    xauth=""
else
    xauth='--env="XAUTHORITY=/home/ubuntu/.Xauthority"'
fi
USER_HOME=$HOME
# [ -e /dev/kfd ] && devs="--device=\"/dev/kfd\""
    # --env="XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR" \
echo "PWD: $PWD"
# docker run --user $(id -u):$(id -g) --userns=keep-id\

name=frota
HAS_NAME=$(docker ps --filter "status=exited" --format '{{.Names}}' | grep -x  $name | wc -l)
if [[ "$HAS_NAME" == "1" ]]; then
    echo "docker start ... $HAS_NAME"
    docker start -ia $name
else
    echo "docker run ... $HAS_NAME"
    xhost +local:docker
    docker run  --user $(id -u):$(id -g) \
        --name $name $net $gpus $xauth $debug $opts \
        --env XDG_RUNTIME_DIR=/tmp/runtime-$USER \
        --env="DISPLAY=$DISPLAY" \
        --env="SDL_VIDEODRIVER=x11" \
        --env="LIBGL_ALWAYS_INDIRECT=0" \
        --env="TZ=America/Sao_Paulo" \
        --env="QT_X11_NO_MITSHM=1" \
        --device /dev/dri \
        --security-opt label=disable \
        --group-add video \
        --group-add render \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --volume="$HOME/.gazebo:$USER_HOME/.gazebo:rw" \
        --volume="$PWD/shared/:/mnt/shared/:rw" \
        --privileged \
        -it $image zsh
fi

