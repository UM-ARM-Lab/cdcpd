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
  * [fgt](https://github.com/gadomski/fgt)
  
Installation
------------

Note: these instructions have been tested on Ubuntu 18.04 and ROS Melodic only.

#### Build

Download [cdcpd_ros](https://github.com/UM-ARM-Lab/cdcpd_ros) and put it under the same folder as cdcpd.

Run the following command
```
catkin build
```

#### Gurobi Licence

Gurobi is a proprietary optimization package that we use. Please obtain a [free academic license](https://www.gurobi.com/academia/academic-program-and-licenses).
Note that Gurobi licence activation requires a university network. If you are not on campus, follow these instructions to setup a VPN: [UMVPN](https://documentation.its.umich.edu/vpn/vpn-linux-vpn-instructions).

#### Faiss Installation

It is recommended to follow the instruction [here](https://github.com/facebookresearch/faiss/blob/master/INSTALL.md).

Demo
------------
To run the demo, you will need to download some [dataset](https://drive.google.com/drive/folders/17_xRbsX6Pnk9KkTxouIu1FLqE1yNmIdW?usp=sharing).
Also, you need to download [cdcpd_ros](https://github.com/UM-ARM-Lab/cdcpd_ros) and put it under the same folder as cdcpd.
You need to specify `.bag` file path in the running command.

To run the bagfile demo start, run the following node:
* `rosrun cdcpd_ros bagfile _bagfile:="NAME_OF_BAG"`
Note that the bag is assumed to exist in `cdcpd_test/datasets/`

My own running result is [here](https://drive.google.com/open?id=1HovZ9eJMZ1WYyCsdsISw5YndlS-BUwSx), which includes bag files and MP4 files.
