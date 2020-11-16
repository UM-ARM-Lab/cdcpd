#!/usr/bin/env bash

INSTALL_DIR=~/local
SRC_REPO_DIR=$INSTALL_DIR/src

mkdir -p $SRC_REPO_DIR; cd $SRC_REPO_DIR
wget https://download.pytorch.org/libtorch/nightly/cpu/libtorch-shared-with-deps-latest.zip
unzip libtorch-shared-with-deps-latest.zip
mkdir -p libtorch/build
cd libtorch/build
cmake .. -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR}
#cmake .. -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR} -DWITH_OPENMP=ON
make -j `nproc` -l `nproc`
make install

