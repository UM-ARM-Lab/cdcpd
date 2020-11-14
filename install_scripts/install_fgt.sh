#!/usr/bin/env bash

INSTALL_DIR=~/local
SRC_REPO_DIR=$INSTALL_DIR/src

mkdir -p $SRC_REPO_DIR; cd $SRC_REPO_DIR
git clone https://github.com/gadomski/fgt.git
mkdir -p fgt/build
cd fgt/build
git checkout v0.4.6
cmake .. -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR}
#cmake .. -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR} -DWITH_OPENMP=ON
make -j `nproc` -l `nproc`
make install

