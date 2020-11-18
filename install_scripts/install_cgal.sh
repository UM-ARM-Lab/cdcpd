#!/usr/bin/env bash

INSTALL_DIR=~/local
SRC_REPO_DIR=$INSTALL_DIR/src

sudo apt update
sudo apt-get install -y libgmp-dev libmpfr-dev libqt5svg5-dev

mkdir -p $SRC_REPO_DIR; cd $SRC_REPO_DIR
git clone https://github.com/CGAL/cgal.git
mkdir -p cgal/build
cd cgal/build
git checkout releases/CGAL-5.0.3
cmake -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR} -DCGAL_HEADER_ONLY=OFF -DCMAKE_BUILD_TYPE=Release -DWITH_CGAL_Qt5=ON ..
# cmake .. -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR} -DWITH_OPENMP=ON
make -j `nproc` -l `nproc`
make install

