#!/usr/bin/env bash

sudo apt install -y libcgal-qt5-dev libcgal-dev libcgal-demo libeigen3-dev ros-noetic-pybind11-catkin

# Install FAISS
INSTALL_DIR=~/.local
SRC_REPO_DIR=$INSTALL_DIR/src

mkdir -p $SRC_REPO_DIR; cd $SRC_REPO_DIR
git clone https://github.com/facebookresearch/faiss.git
cd faiss
git checkout v1.6.3
./configure --prefix=$INSTALL_DIR --without-cuda
make install

# Install Gurobi
wget https://packages.gurobi.com/9.1/gurobi9.1.1_linux64.tar.gz
tar xvfz gurobi9.1.1_linux64.tar.gz
sudo cp -r gurobi911/ /opt/

rm -rf gurobi911/
rm -rf gurobi9.1.1_linux64.tar.gz

echo 'Adding the following lines to your ~/.bashrc:'
echo 'export GUROBI_HOME=/opt/gurobi911/linux64'
echo 'export PATH=${PATH}:${GUROBI_HOME}/bin'
echo 'export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${GUROBI_HOME}/lib'

echo 'export GUROBI_HOME=/opt/gurobi911/linux64' >> ~/.bashrc
echo 'export PATH=${PATH}:${GUROBI_HOME}/bin' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${GUROBI_HOME}/lib' >> ~/.bashrc
