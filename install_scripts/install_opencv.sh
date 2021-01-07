#!/bin/bash

INSTALL_DIR=~/.local
SRC_REPO_DIR=$INSTALL_DIR/src

sudo -u root add-apt-repository "deb http://security.ubuntu.com/ubuntu xenial-security main"
sudo -u root apt update
sudo -u root apt install -y libjasper1 libjasper-dev
sudo -u root apt-get install -y build-essential
sudo -u root apt-get install -y cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo -u root apt-get install -y python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev

mkdir -p $SRC_REPO_DIR; cd $SRC_REPO_DIR
git clone https://github.com/opencv/opencv.git
cd opencv; git checkout 3.4; cd ..
git clone https://github.com/opencv/opencv_contrib.git
cd opencv_contrib; git checkout 3.4; cd ..
mkdir -p opencv/build; cd opencv/build
cmake -D BUILD_opencv_cudacodec=OFF -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules -D CMAKE_INSTALL_PREFIX=${INSTALL_DIR} -D OPENCV_GENERATE_PKGCONFIG=True -D CMAKE_BUILD_TYPE=Release -D WITH_QT=True ..
make -j7
make install

