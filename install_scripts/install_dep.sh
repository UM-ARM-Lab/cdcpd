#!/usr/bin/env bash

INSTALL_DIR=~/.local
SRC_REPO_DIR=$INSTALL_DIR/src
USR_NAME=deformtrack

echo 'export LD_LIBRARY_PATH=~/.local/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
echo 'export PKG_CONFIG_PATH=~/.local/lib/pkgconfig:$PKG_CONFIG_PATH' >> ~/.bashrc
echo 'export PATH=~/.local/bin:${PATH}' >> ~/.bashrc
echo 'export CMAKE_PREFIX_PATH=~/.local:$CMAKE_PREFIX_PATH' >> ~/.bashrc

source ~/.bashrc

sudo -u root add-apt-repository "deb http://us.archive.ubuntu.com/ubuntu focal universe"
sudo -u root apt update
sudo -u root apt install -y libcgal-qt5-dev libcgal-dev libcgal-demo libeigen3-dev

# sudo -u $USR_NAME ./install_opencv.sh
sudo -u $USR_NAME ./install_gurobi.sh
sudo -u $USR_NAME ./install_faiss.sh
# sudo -u $USR_NAME ./install_fgt.sh
# sudo -u $USR_NAME ./install_nomad.sh
# sudo -u $USR_NAME ./install_svm.sh
# sudo -u $USR_NAME ./install_torch.sh
# sudo -u $USR_NAME ./install_ceres.sh
./install_pybind11_catkin.sh

source ~/.bashrc

