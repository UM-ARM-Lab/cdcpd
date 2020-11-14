#!/usr/bin/env bash

INSTALL_DIR=~/local
SRC_REPO_DIR=$INSTALL_DIR/src

mkdir -p $SRC_REPO_DIR; cd $SRC_REPO_DIR
git clone https://github.com/CGAL/cgal.git
mkdir -p CGAL/build
cd CGAL/build
git checkout releases/CGAL-5.0.3
cmake -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR} -DCGAL_HEADER_ONLY=OFF -DCMAKE_BUILD_TYPE=Release -DWITH_CGAL_Qt5=ON ..
# cmake .. -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR} -DWITH_OPENMP=ON
make -j `nproc` -l `nproc`
make install

