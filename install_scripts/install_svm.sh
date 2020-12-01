#!/usr/bin/env bash
<<<<<<< HEAD
=======
sudo apt install libcgal-qt5-dev -y
>>>>>>> ca33597... trying to build on 20.04 without external deps
INSTALL_DIR=~/.local
SRC_REPO_DIR=$INSTALL_DIR/src
mkdir -p $SRC_REPO_DIR; cd $SRC_REPO_DIR
git clone https://github.com/WangYixuan12/libsvm.git
cd libsvm
<<<<<<< HEAD
make install INSTALL_PATH=$INSTALL_DIR -j `nproc`
=======
make install -j `nproc`
>>>>>>> ca33597... trying to build on 20.04 without external deps
