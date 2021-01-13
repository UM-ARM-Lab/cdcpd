Constrained Deformable Coherent Point Drift (CDCPD & CDCPD2)
=============

CDCPD is an implementation of *Occlusion-robust Deformable Object Tracking without Physics Simulation*
by Cheng Chi and Dmitry Berenson.

CDCPD2 is an implementation of *Tracking Partially-Occluded Deformable Objects while Enforcing Geometric Constraints*
by Yixuan Wang, Dale McConachie and Dmitry Berenson.

Quick Demo
------------
 You could try out virtual machine linked [here](https://drive.google.com/file/d/1N_l3ZRWVAIY1Mwr8b5D8uyWY96DrnUMm/view?usp=sharing). Password is `123456`. You could run demos under `~/catkin_ws/src/cdcpd/cdcpd_ros/scripts`. Note that not all data is downloaded. Now only `rope_edge_cover_2.bag` is downloaded under `~/catkin_ws/src/cdcpd/cdcpd_ros/dataset`. If you would like to see more demoes, please download [here](https://drive.google.com/drive/folders/1rnmUDIAFOpbrpt6wNurH6x2WF5xm_3ij?usp=sharing).

Requirements
------------
  * Environment:
    * Ubuntu 18 or 20
    * ROS Melodic or Noetic
  * apt dependencies
    * [Eigen](http://eigen.tuxfamily.org/dox/GettingStarted.html)
    * [CGAL-5.0.3](https://github.com/CGAL/cgal/releases/tag/releases%2FCGAL-5.0.3)
  * other dependencies
    * [Gurobi](https://www.gurobi.com/)
    * [faiss-1.6.3](https://github.com/facebookresearch/faiss)
    * [kuka_iiwa_interface](https://github.com/UM-ARM-Lab/kuka_iiwa_interface)
      * [robotiq](https://github.com/UM-ARM-Lab/robotiq) (needed by kuka_iiwa_interface)
    * [arc_utilities](https://github.com/UM-ARM-Lab/arc_utilities)


Installation
------------

#### Installing ROS (ignore it if you already have it)

Run `sudo -u USER_NAME install_scripts/install_ros_melodic.sh` if you use Ubuntu 18.04, or `sudo -u USER_NAME install_scripts/install_ros_noetic.sh` if you use Ubuntu 20.04

#### Installing dependency

Modify USR\_NAME in `install_scripts/install_dep.sh` and run `sudo -u USER_NAME ./install_dep.sh` under `install_scripts`. It will install all dependency listed above in `~/.local`.

NOTE: `source ~/.bashrc` inside `install_dep.sh` might not run successfully according to the platform. If you encounter the problem like `catkinonfig.cmake` not found, please run `source ~/.bashrc` and run `./install_pybind11_catkin.sh`.

#### Create catkin workspace

We assume you have created a catkin workspace. Now clone this repo to that worksace. See `install_scripts/create_ws_ROS_Version.sh` or the ROS wiki on how to setup a catkin workspace.

#### Gurobi Licence

Gurobi is a proprietary optimization package that we use. Please obtain a [free academic license](https://www.gurobi.com/academia/academic-program-and-licenses).

#### Building

```
# in the src directory
git clone https://github.com/UM-ARM-Lab/cdcpd.git
```

Once you've cloned, it might be a good idea to `rosdep install -r --from-paths cdcpd -y` to get any ROS packages you might be depending on.


Demo
------------
To run the demo, you will need to download some [dataset](https://drive.google.com/drive/folders/1rnmUDIAFOpbrpt6wNurH6x2WF5xm_3ij?usp=sharing). Then run the corresponding scripts under `cdcpd/scripts`. You need to specify `.bag` file path in the shell script.

My own running result is [here](https://drive.google.com/drive/folders/1MZTR-hEaU5czsxzUIKvPnCCAEd29aM4u?usp=sharing), which includes MP4 files.

# FAQ & Misc Notes

**Q:** It runs without error but doesn't seem to be processing images, help!

**A:** We use a time synchronizer with "exact" time policy (the deafult). Therefore if your depth, color, and camera_info messages do not have exactly the same time stamps, the synchronizer will ignore it and nothing will happen.
