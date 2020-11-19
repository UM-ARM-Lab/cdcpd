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
  * [Ceres Solver-1.14.0](https://github.com/ceres-solver/ceres-solver/tree/1.14.0)
  
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
* Ceres Solver: specify your installing directory and run `sudo install_scripts/install_ceres.sh`

#### Create catkin workspace (ignore it if you already have a catkin workspace)

Specify workspace name and run `create_ws.sh` and add `source ~/YOUR_WORKSPACE/devel/setup.bash` to `~/.bashrc`

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

#### Conflicting LZ4 declaration
The combination of the operating system and ROS may lead to a specific declaration confliction for `LZ4_stream_t` and `LZ4_streamDecode_t`. The solution is described in [https://github.com/ethz-asl/lidar\_align/issues/16](https://github.com/ethz-asl/lidar_align/issues/16). Please just run the following commands in the terminal.

```bash
sudo mv /usr/include/flann/ext/lz4.h /usr/include/flann/ext/lz4.h.bak
sudo mv /usr/include/flann/ext/lz4hc.h /usr/include/flann/ext/lz4.h.bak

sudo ln -s /usr/include/lz4.h /usr/include/flann/ext/lz4.h
sudo ln -s /usr/include/lz4hc.h /usr/include/flann/ext/lz4hc.h
```

Demo
------------
To run the demo, you will need to download some [dataset](https://drive.google.com/drive/folders/1rnmUDIAFOpbrpt6wNurH6x2WF5xm_3ij?usp=sharing). Then run the corresponding scripts under `cdcpd/scripts`. You need to specify `.bag` file path in the shell script.

My own running result is [here](https://drive.google.com/drive/folders/1MZTR-hEaU5czsxzUIKvPnCCAEd29aM4u?usp=sharing), which includes MP4 files.
