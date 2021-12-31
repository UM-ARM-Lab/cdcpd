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
    * Ubuntu 20
    * ROS Noetic
  * apt dependencies
    * [Eigen](http://eigen.tuxfamily.org/dox/GettingStarted.html)
    * [CGAL-5.0.3](https://github.com/CGAL/cgal/releases/tag/releases%2FCGAL-5.0.3)
  * other dependencies
    * [Gurobi](https://www.gurobi.com/)
    * [faiss-1.6.3](https://github.com/facebookresearch/faiss)

Installation
------------

#### Installing ROS (ignore it if you already have it)

Please refer to [ROS installation](http://wiki.ros.org/ROS/Installation)

#### Installing dependency

Run `sudo -u USER_NAME ./install_for_noetic_20.sh` under `install_scripts`. It will install all dependency listed above in `~/.local`.

NOTE: remove packages you have had under `external`.

#### Create catkin workspace

Please refer to [catkin](http://wiki.ros.org/catkin#Installing_catkin). NOTE: using `catkin build` instead of `catkin_make`

#### Gurobi Licence

Gurobi is a proprietary optimization package that we use. Please obtain a [free academic license](https://www.gurobi.com/academia/academic-program-and-licenses).

#### Building

```
cd ~/catkin_ws/src/
mv ~/cdcpd ~/catkin_ws/src
```

Once you've cloned, it might be a good idea to `rosdep install -r --from-paths cdcpd -y` to get any ROS packages you might be depending on. Then run `catkin build` at any folder inside `catkin_ws`.

Demo
------------
To run the demo, you will need to download some [dataset](https://drive.google.com/drive/folders/1rnmUDIAFOpbrpt6wNurH6x2WF5xm_3ij?usp=sharing).

Then run `roscore`, `rosrun cdcpd cdcpd_node`, `rosbag play $BAG_NAME`, and `rviz` concurrently.

My own running result is [here](https://drive.google.com/drive/folders/1MZTR-hEaU5czsxzUIKvPnCCAEd29aM4u?usp=sharing), which includes MP4 files.

# FAQ & Misc Notes

**Q:** It runs without error but doesn't seem to be processing images, help!

**A:** We use a time synchronizer with "exact" time policy (the deafult). Therefore if your depth, color, and camera_info messages do not have exactly the same time stamps, the synchronizer will ignore it and nothing will happen.
