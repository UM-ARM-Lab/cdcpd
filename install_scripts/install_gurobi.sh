#!/usr/bin/env bash

wget https://packages.gurobi.com/9.1/gurobi9.1.1_linux64.tar.gz
tar xvfz gurobi9.1.1_linux64.tar.gz
sudo cp -r gurobi911/ /opt/
sudo /opt/nomad.3.8.1/install/install.sh

rm -rf gurobi911/
rm -rf gurobi9.1.1_linux64.tar.gz

echo 'export GUROBI_HOME=/opt/gurobi911/linux64' >> ~/.bashrc
echo 'export PATH=${PATH}:${GUROBI_HOME}/bin' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${GUROBI_HOME}/lib' >> ~/.bashrc
source ~/.bashrc

