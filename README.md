Constrained Deformable Coherent Point Drift (CDCPD & CDCPD2)
=============

CDCPD is an implementation of *Occlusion-robust Deformable Object Tracking without Physics Simulation*
by Cheng Chi and Dmitry Berenson.

CDCPD2 is an implementation of *Tracking Partially-Occluded Deformable Objects while Enforcing Geometric Constraints* 
by Yixuan Wang, Dale McConachie and Dmitry Berenson.

Requirements
------------
  * Environment:
    * Ubuntu 18.04
    * ROS melodic
  * OpenCV
  * [Gurobi](https://www.gurobi.com/)
  * Eigen
  * [faiss](https://github.com/facebookresearch/faiss)
  * [fgt](https://github.com/gadomski/fgt)
  * CGAL (Note that Qt5 should be installed for visualization)
  * [smmap](https://github.com/UM-ARM-Lab/smmap/tree/smmap_cdcpd)
  
Installation
------------

#### Create Workspace (ignore it if you already have a catkin workspace)

Run the following command
```bash
source /opt/ros/melodic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin build
source devel/setup.bash
catkin_init_workspace
```

#### Build prerequisite

We will put dependent catkin packages in src.

```bash
cd src
```

##### smmap

NOTE:
* now smmap and smmap\_utilities are private repo in ARM Lab. To use it, we need to make it public, or extract codes we need.
* comment out `find_package` for MoveIt in `sdf_tools/CMakeLists` if you don't use MoveIt

```bash
git clone https://github.com/UM-ARM-Lab/arc_utilities.git
git clone https://github.com/UM-ARM-Lab/smmap_utilities.git
git clone https://github.com/UM-ARM-Lab/kinematics_toolbox.git
git clone https://github.com/UM-ARM-Lab/deformable_manipulation_interface.git
git clone https://github.com/UM-ARM-Lab/sdf_tools.git
cd sdf_tools; git checkout EightConnected; cd ..;
git clone https://github.com/UM-ARM-Lab/smmap.git
cd smmap
git checkout smmap_cdcpd
cd ..
```

#### Build from source

Run the following command
```
git clone https://github.com/UM-ARM-Lab/cdcpd.git
cd cdcpd
git checkout CDCPD2
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
