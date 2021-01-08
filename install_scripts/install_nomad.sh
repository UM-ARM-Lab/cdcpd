#!/usr/bin/env bash

wget https://www.gerad.ca/nomad/Downloads/nomad.3.8.zip
unzip -a nomad.3.8.zip
sudo cp -r nomad.3.8.1/ /opt/

sudo /opt/nomad.3.8.1/install/install.sh

rm -rf nomad.3.8.*

echo 'export NOMAD_HOME=/opt/nomad.3.8.1' >> ~/.bashrc
echo 'export PATH=$NOMAD_HOME/bin:$PATH' >> ~/.bashrc
source ~/.bashrc

