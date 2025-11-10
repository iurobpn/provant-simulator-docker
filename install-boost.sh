#!/bin/bash

set -e

cd /tmp

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
./bootstrap.sh
sudo ./b2 install

cd /tmp
rm -rf boost_*
