#!/bin/bash
if [ "$1" = '-h' ]; then
    echo 'usage: $0'
    echo 'removes all containers and removes exited containers'
    exit 0
fi

docker stop $(sudo docker ps -a -q)
docker rm $(sudo docker ps -a -q)
