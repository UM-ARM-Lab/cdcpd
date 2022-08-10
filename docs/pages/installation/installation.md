---
title: Installation
nav_order: 1
---

# Installation
{:.no_toc}

Tutorials to show how to install CDCPD.

* TOC
{:toc}

## Requirements
  * Environment:
    * Ubuntu 18 or 20 (20 is strongly preferred)
    * ROS Melodic or Noetic (Noetic strongly preferred)
  * apt dependencies
    * [Eigen](http://eigen.tuxfamily.org/dox/GettingStarted.html)
    * [CGAL-5.0.3](https://github.com/CGAL/cgal/releases/tag/releases%2FCGAL-5.0.3)
  * other dependencies
    * [Gurobi](https://www.gurobi.com/)
    * [faiss-1.6.3](https://github.com/facebookresearch/faiss)
    * [kuka_iiwa_interface](https://github.com/UM-ARM-Lab/kuka_iiwa_interface)
      * [robotiq](https://github.com/UM-ARM-Lab/robotiq) (needed by kuka_iiwa_interface)
    * [arc_utilities](https://github.com/UM-ARM-Lab/arc_utilities)




### Installing ROS (ignore it if you already have it)

Run `sudo -u USER_NAME install_scripts/install_ros_melodic.sh` if you use Ubuntu 18.04, or `sudo -u USER_NAME install_scripts/install_ros_noetic.sh` if you use Ubuntu 20.04

### Installing Dependencies

Modify USR\_NAME in `install_scripts/install_dep.sh` and run `sudo -u USER_NAME ./install_dep.sh` under `install_scripts`. It will install all dependency listed above in `~/.local`.

NOTE: `source ~/.bashrc` inside `install_dep.sh` might not run successfully according to the platform. If you encounter the problem like `catkinonfig.cmake` not found, please run `source ~/.bashrc` and run `./install_pybind11_catkin.sh`.

### Create catkin workspace

We assume you have created a catkin workspace. Now clone this repo to that worksace. See `install_scripts/create_ws_ROS_Version.sh` or the ROS wiki on how to setup a catkin workspace.

### Gurobi Licence

Gurobi is a proprietary optimization package that we use. Please obtain a [free academic license](https://www.gurobi.com/academia/academic-program-and-licenses).

## Building

```
# in the src directory
git clone https://github.com/UM-ARM-Lab/cdcpd.git
```

Once you've cloned, it might be a good idea to `rosdep install -r --from-paths cdcpd -y` to get any ROS packages you might be depending on.

## Verifying Your Installation

Check out the [Testing page](../testing/testing) for more information on how to verify your installation and how to add unit tests to the repository.