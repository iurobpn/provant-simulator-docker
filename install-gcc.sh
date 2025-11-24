#/bin/bash

add-apt-repository ppa:ubuntu-toolchain-r/test
apt-get update
apt-get install --yes gcc-13 g++-13
update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-13 110
update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-13 110
rm -rf /var/lib/apt/lists/*
