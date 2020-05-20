Constrained Deformable Coherent Point Drift (CDCPD)
=============

CDCPD is a implementation of *Occlusion-robust Deformable Object Tracking without Physics Simulation*
by Cheng Chi and Dmitry Berenson.

Requirements
------------
  * OpenCV
  * [Gurobi](https://www.gurobi.com/)
  * Eigen
  * [faiss](https://github.com/facebookresearch/faiss)
  
Installation
------------

Note: these instructions have been tested on Ubuntu 18.04 and ROS Melodic only.

#### Build

Download [cdcpd_ros](https://github.com/UM-ARM-Lab/cdcpd_ros) and put it under the same folder as cdcpd.

Run the following command in `cdcpd/src`
```
./install_home_local.sh
```
Installation directory can be modified in `./install_home_local.sh` by modifying `INSTALL_DIR=~/local`

#### Gurobi Licence

Gurobi is a proprietary optimization package that we use. Please obtain a [free academic license](https://www.gurobi.com/academia/academic-program-and-licenses).
Note that Gurobi licence activation requires a university network. If you are not on campus, follow these instructions to setup a VPN: [UMVPN](https://documentation.its.umich.edu/vpn/vpn-linux-vpn-instructions).

#### Faiss Installation

It is recommended to follow the instruction [here](https://github.com/facebookresearch/faiss/blob/master/INSTALL.md).
The thing is to note that `FAISS_INCLUDE_DIRS` in `CMakeLists.txt` should be changed to the directory in your system.

Demo
------------
To run the demo, you will need to download some [dataset](https://drive.google.com/drive/folders/17_xRbsX6Pnk9KkTxouIu1FLqE1yNmIdW?usp=sharing).
Also, you need to download [cdcpd_ros](https://github.com/UM-ARM-Lab/cdcpd_ros) and put it under the same folder as cdcpd.
You need to change `.bag` file path in `cdcpd_ros/cdcdpd_node.cpp`, where you can find by searching `.bag` in the file.

To run the fully ROS integrated demo start all of the following from individual terminals, without activating the conda workspace:
* Start ROS core: `roscore`
* Start Rviz for visualization: `rviz`
* Start the tracking nodes: `rosrun cdcpd_ros node`

My own running result is [here](https://drive.google.com/open?id=1HovZ9eJMZ1WYyCsdsISw5YndlS-BUwSx), which includes bag files and MP4 files.
