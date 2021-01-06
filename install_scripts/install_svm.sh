#!/usr/bin/env bash
sudo apt install libcgal-qt5-dev -y
INSTALL_DIR=~/.local
SRC_REPO_DIR=$INSTALL_DIR/src
mkdir -p $SRC_REPO_DIR; cd $SRC_REPO_DIR
git clone https://github.com/dmcconachie/libsvm.git
cd libsvm
make install -j `nproc`
