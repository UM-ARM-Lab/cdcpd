Constrained Deformable Coherent Point Drift (CDCPD)
=============

CDCPD is a implementation of *Occlusion-robust Deformable Object Tracking without Physics Simulation*
by Cheng Chi and Dmitry Berenson.

This library includes object-oriented API for tracker, visualization utilities and a ROS node that subscribes 
and publishes PointCloud2.

Requirements
------------
  * Numpy
  * Scipy
  * Numexpr
  * scikit-learn
  * OpenCV
  * [Gurobi](https://www.gurobi.com/)
  * matplotlib\*
  * [glplotlib](https://github.com/cheng-chi/glplotlib)\*
  * rospy
  * ros_numpy
  
\*: (optional) for visualization only

Installation
------------

Note: these instructions have been tested on Ubuntu 18.04 and ROS Melodic only.

#### Core Package

Clone the following repositories into your catkin workspace:
```
cd path/to/catkin_ws/src/
git clone git@github.com:UM-ARM-Lab/cdcpd.git
git clone git@github.com:eric-wieser/ros_numpy.git
```
Build the project (tested with `catkin build`, not `catkin_make`)
```
catkin build
```

If you need tracking failure recovery, you will also need to compile the VFH feature library.
Make sure you have PCL-1.8 installed. If you have ROS installed, it should be already there.
```
cd src/pcl_features
mkdir build
cd build
cmake ..
make
cd ..
ln -s build/libfeatures.so
```

#### Conda configuration

Install miniconda https://docs.conda.io/en/latest/miniconda.html; use the python 3 version

Create conda virtual enviornment with the packages that conda knows about, you must name it as shown for the roslaunch/scripts to run correctly. This environment is used to isolate python3 code from the default python2 code on Ubuntu 18.04 and ROS Melodic
```
conda create -c gurobi --name catkin_cdcpd python=3.7 opencv gurobi numpy scipy numexpr scikit-learn matplotlib ipython
conda activate catkin_cdcpd
```

For the following operations, make sure the newly created virtual environment is activated. The command line should show the name of virtual environment in brackets.
Install the rest of the requirements via pip:
```
(catkin_cdcpd) user@host:path/to/catkin_ws/src/cdcpd$ pip install -r cdcpd/requirements.txt
```

#### Gurobi Licence

Gurobi is a proprietary optimization package that we use. Please obtain a [free academic license](https://www.gurobi.com/academia/academic-program-and-licenses).
Note that Gurobi licence activation requires a university network. If you are not on campus, follow these instructions to setup a VPN: [UMVPN](https://documentation.its.umich.edu/vpn/vpn-linux-vpn-instructions).

Ensure that the conda environment is active, and install a Gurobi key with `grbgetkey` [following the instructions here](https://www.gurobi.com/documentation/9.0/quickstart_mac/retrieving_a_free_academic.html)
```
(catkin_cdcpd) user@host:~$ grbgetkey 253e22f3-...
```


You may have noticed that a package named [glplotlib](https://github.com/cheng-chi/glplotlib) is installed by pip. glplotlib is another opensource library developed by Cheng Chi for visualizing 3D point cloud with better performance (compared to matplotlib).

If you use a IDE such as PyCharm or VS Code, remember to set project interpreter to this virtual environment.

Demo
------------
To run the demo, you will need to download some [data](https://drive.google.com/drive/folders/1QSmSOw0JvQl9xnbVnBNogk0OcNo0Rn4_?usp=sharing) into <project_root>/data folder.
The files in the `v0.1.0` folder are intended for use with [the original version of this library](https://github.com/UM-ARM-Lab/cdcpd/tree/v0.1.0). Follow instructions in the README there to use them.

To run the fully ROS integrated demo start all of the following from individual terminals, without activating the conda workspace:
* Start ROS core: `roscore`
* Start Rviz for visualization: `rviz -d path/to/catkin_ws/src/cdcpd/config/cdcpd.rviz`
* Start the tracking nodes: `roslaunch cdcpd tracker.launch`
* Start playing the rosbag: `rosbag play -l data/placemat_stationary.bag`

The following messages are a normal part of the initialization process:
```
[rosout]: Error: itemsize cannot be zero in type
/path/to/catkin_ws/src/cdcpd/src/cdcpd/cv_utils.py:68: RuntimeWarning: divide by zero encountered in true_divide
  projected[:, 0] /= projected[:, 2]
/path/to/catkin_ws/src/cdcpd/src/cdcpd/cv_utils.py:68: RuntimeWarning: invalid value encountered in true_divide
  projected[:, 0] /= projected[:, 2]
/path/to/catkin_ws/src/cdcpd/src/cdcpd/cv_utils.py:69: RuntimeWarning: divide by zero encountered in true_divide
  projected[:, 1] /= projected[:, 2]
/path/to/catkin_ws/src/cdcpd/src/cdcpd/cv_utils.py:69: RuntimeWarning: invalid value encountered in true_divide
  projected[:, 1] /= projected[:, 2]
  ```
