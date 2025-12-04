#!/bin/bash

VERSION=3.7.0
set -e

cd /tmp
if ! [ -d /tmp/casadi  ]; then
    git clone https://github.com/casadi/casadi.git
fi
cd /tmp/casadi
git fetch && git checkout 3.7.2
mkdir -p build && cd build
cmake -G Ninja -DWITH_IPOPT=true ..
ninja
sudo ninja install
sudo ldconfig

sudo rm -rf /tmp/casadi
