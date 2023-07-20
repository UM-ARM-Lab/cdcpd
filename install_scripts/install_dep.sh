#!/usr/bin/env bash
#
# The first argument to this script must be your username
echo "your user is $1"

sudo -u root add-apt-repository "deb http://us.archive.ubuntu.com/ubuntu focal universe"
sudo -u root apt update
sudo -u root apt install -y libcgal-qt5-dev libcgal-dev libcgal-demo libeigen3-dev

sudo -u $1 ./install_gurobi.sh
./install_faiss.sh
./install_pybind11_catkin.sh

# NOTE: running this script multiple times will pollute your .bashrc with duplicate export commands
echo 'export LD_LIBRARY_PATH=~/.local/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
echo 'export PKG_CONFIG_PATH=~/.local/lib/pkgconfig:$PKG_CONFIG_PATH' >> ~/.bashrc
echo 'export PATH=~/.local/bin:${PATH}' >> ~/.bashrc
echo 'export CMAKE_PREFIX_PATH=~/.local:$CMAKE_PREFIX_PATH' >> ~/.bashrc

source ~/.bashrc
