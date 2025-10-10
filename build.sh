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

docker build -t $image .
