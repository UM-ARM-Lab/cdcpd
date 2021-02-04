#!/usr/bin/env bash

INSTALL_DIR=~/local
SRC_REPO_DIR=$INSTALL_DIR/share

mkdir -p $SRC_REPO_DIR; cd $SRC_REPO_DIR
git clone https://github.com/ipab-slmc/pybind11_catkin.git
mkdir -p pybind11_catkin/build
cd pybind11_catkin/build
cmake .. -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR}
make install

