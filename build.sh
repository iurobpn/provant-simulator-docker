#!/bin/bash

if [ -z "$1" ]; then
    image="provsim"
else
    image=$1
fi

docker build -t $image -f images/$image/Dockerfile .
