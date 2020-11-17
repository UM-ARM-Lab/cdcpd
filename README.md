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
    * ROS Melodic
  * [Gurobi](https://www.gurobi.com/)
  * [Eigen](http://eigen.tuxfamily.org/dox/GettingStarted.html)
  * [faiss-1.6.3](https://github.com/facebookresearch/faiss)
  * [fgt-0.4.6](https://github.com/gadomski/fgt)
  * [CGAL-5.0.3](https://github.com/CGAL/cgal/releases/tag/releases%2FCGAL-5.0.3)
  * [NOMAD-3.8.1](https://www.gerad.ca/nomad/)
  * [libsvm](https://github.com/dmcconachie/libsvm)
  * [PyTorch](https://pytorch.org/cppdocs/installing.html)
  
Installation
------------

#### Installing ROS (ignore it if you already have one)

Run `sudo install_scripts/install_ros_melodic.sh` if you use Ubuntu 18.04, or `sudo install_scripts/install_ros_neotic.sh` if you use Ubuntu 20.04

#### Installing third-party library

Recommended organization of third party library: place the source code under `~/local/src` and install under `~/local`. If you do so, you can link to those libraries easily by adding below to `~/.bashrc`.

```bash
export LD_LIBRARY_PATH=~/local/lib:$LD_LIBRARY_PATH
export PKG_CONFIG_PATH=~/local/lib/pkgconfig:$PKG_CONFIG_PATH
export PATH=~/local/bin:${PATH}
export CMAKE_PREFIX_PATH=~/local:$CMAKE_PREFIX_PATH
```

* Gurobi: follow instructions here `https://www.gurobi.com/documentation/9.0/quickstart_linux/software_installation_guid.html`
* Eigen: `sudo apt install libeigen3-dev`
* faiss-1.6.3: specify your installing directory and run `sudo install_scripts/install_faiss.sh`
* fgt-0.4.6: specify your installing directory and run `sudo install_scripts/install_fgt.sh`
* CGAL-5.0.3: specify your installing directory and run `sudo install_scripts/install_cgal.sh`
* NOMAD-3.8.1:
  * Click 'Download' in [https://www.gerad.ca/nomad/](https://www.gerad.ca/nomad/) and download 3.8.
  * Extract the zip file and put under `/opt`
  * Run `sudo /opt/nomad.3.8.1/install/install.sh`.
  * Add the following to `~/.bashrc`
```bash
export NOMAD_HOME="/opt/nomad.3.8.1"
export PATH=$NOMAD_HOME/bin:$PATH
```
  * Verify the success of installation by running `nomad`. You should see the following
```bash
Run NOMAD      : nomad parameters_file
Info           : nomad -i
Help           : nomad -h keyword(s) (or 'all')
Developer help : nomad -d keyword(s) (or 'all')
Version        : nomad -v
Usage          : nomad -u
```
* libsvm: specify your installing directory and run `sudo install_scripts/install_svm.sh`
* PyTorch: specify your installing directory and run `sudo install_scripts/install_torch.sh`

#### Create catkin workspace (ignore it if you already have a catkin workspace)

Specify workspace name and run `create_ws.sh`

#### Build from source

Run the following command under the directory `$WS_PATH/src`
```
git clone https://github.com/UM-ARM-Lab/cdcpd.git
cd cdcpd
git checkout CDCPD2
catkin build
```

#### Gurobi Licence

Gurobi is a proprietary optimization package that we use. Please obtain a [free academic license](https://www.gurobi.com/academia/academic-program-and-licenses).
Note that Gurobi licence activation requires a university network. If you are not on campus, follow these instructions to setup a VPN: [UMVPN](https://documentation.its.umich.edu/vpn/vpn-linux-vpn-instructions).

Demo
------------
To run the demo, you will need to download some [dataset](https://drive.google.com/drive/folders/17_xRbsX6Pnk9KkTxouIu1FLqE1yNmIdW?usp=sharing).
Also, you need to download [cdcpd_ros](https://github.com/UM-ARM-Lab/cdcpd_ros) and put it under the same folder as cdcpd.
You need to specify `.bag` file path in the running command.

To run the bagfile demo start, run the following node:
* `rosrun cdcpd_ros bagfile _bagfile:="NAME_OF_BAG"`
Note that the bag is assumed to exist in `cdcpd_test/datasets/`

My own running result is [here](https://drive.google.com/open?id=1HovZ9eJMZ1WYyCsdsISw5YndlS-BUwSx), which includes bag files and MP4 files.
