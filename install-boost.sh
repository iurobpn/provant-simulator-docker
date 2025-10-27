#!/bin/bash

set -e

cd /tmp
apt-get update
apt-get install --yes build-essential python3 libbz2-dev libz-dev libicu-dev libopenmpi-dev
SUB_VER=0
MINOR_VER=81
MAJOR_VER=1

VERSION=$MAJOR_VER"_"$MINOR_VER"_"$SUB_VER
DOT_VERSION=$MAJOR_VER.$MINOR_VER.$SUB_VER
if ! [ -f boost_$VERSION.tar.bz2 ]; then
    wget https://archives.boost.io/release/$DOT_VERSION/source/boost_$VERSION.tar.bz2
fi

tar -xvf boost_$VERSION.tar.bz2
cd boost_$VERSION
./bootstrap.sh --prefix=/usr/local
./b2
sudo ./b2 install --prefix=/usr/local

echo 'export BOOST_ROOT=/usr/local' >> /home/ubuntu/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH'  >> /home/ubuntu/.bashrc
echo 'export CPLUS_INCLUDE_PATH=/usr/local/include:$CPLUS_INCLUDE_PATH'  >> /home/ubuntu/.bashrc

cd /tmp
rm -rf boost_*
