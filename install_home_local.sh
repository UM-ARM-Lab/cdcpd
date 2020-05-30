#!/usr/bin/env bash

INSTALL_DIR=~/local
mkdir -p $INSTALL_DIR

mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=RelWithDebInf -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR} .. 
make -j `nproc`
make install
catkin build

