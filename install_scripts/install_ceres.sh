#!/usr/bin/env bash

# CMake
sudo -u root apt-get install -y cmake
# google-glog + gflags
sudo -u root apt-get install -y libgoogle-glog-dev
# BLAS & LAPACK
sudo -u root apt-get install -y  libatlas-base-dev
# Eigen3
sudo -u root apt-get install -y  libeigen3-dev
# SuiteSparse and CXSparse (optional)
# - If you want to build Ceres as a *static* library (the default)
#   you can use the SuiteSparse package in the main Ubuntu package
#   repository:
sudo -u root apt-get install -y  libsuitesparse-dev
# - However, if you want to build Ceres as a *shared* library, you must
#   add the following PPA:
sudo -u root add-apt-repository ppa:bzindovic/suitesparse-bugfix-1319687
sudo -u root apt-get update
sudo -u root apt-get install -y  libsuitesparse-dev

INSTALL_DIR=~/.local
SRC_REPO_DIR=$INSTALL_DIR/src

mkdir -p $SRC_REPO_DIR; cd $SRC_REPO_DIR
# download from CGAL github release page
git clone https://github.com/ceres-solver/ceres-solver.git
cd ceres-solver
git checkout 1.14.0
mkdir -p build
cd build
cmake -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR} ..
# cmake .. -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR} -DWITH_OPENMP=ON
make -j `nproc` -l `nproc`
make test
make install

