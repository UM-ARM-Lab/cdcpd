#!/usr/bin/env bash

INSTALL_DIR=~/local
SRC_REPO_DIR=$INSTALL_DIR/src

mkdir -p $SRC_REPO_DIR; cd $SRC_REPO_DIR
git clone https://github.com/facebookresearch/faiss.git
cd faiss
./configure --prefix=$INSTALL_DIR --without-cuda
make install

