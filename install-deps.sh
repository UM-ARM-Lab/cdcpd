#!/bin/bash

# apt dependencies
wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null
sudo apt update
sudo apt install cmake
sudo apt install libceres-dev swig

cd ~/.local
mkdir -p src_repos

# FGT
git clone git@github.com:gadomski/fgt.git
cd fgt
mkdir build
cmake ..
make -j8
sudo make install

# Faiss
cd ~/.local/src_repos
git clone git@github.com:facebookresearch/faiss.git
cd faiss
mkdir build
cd build
cmake ..
make -j8
sudo make install
