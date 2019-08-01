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

#### Core Package

Install miniconda https://docs.conda.io/en/latest/miniconda.html; use the python 3 version

Create conda virtual enviornment with the packages that conda knows about, you must name it as shown for the roslaunch/scripts to run correctly. This environment is used to isolate python3 code from the default python2 code on Ubuntu 18.04 + ROS Melodic
```
conda create -c gurobi --name catkin_cdcpd python=3.7 opencv gurobi numpy scipy numexpr scikit-learn matplotlib
conda activate catkin_cdcpd
```

Gurobi as a proprietary optimization package that we use. Please obtain a [free academic license](https://user.gurobi.com/download/licenses/free-academic)
Install gurobi key with [grbgetkey](https://www.gurobi.com/documentation/8.1/quickstart_mac/retrieving_a_free_academic.html).
Note that gurobi installation requires university network. If you are not on campus, follow this instruction to setup [UMVPN](https://documentation.its.umich.edu/vpn/vpn-linux-vpn-instructions).

If you use a IDE such as pycharm or vscode, remember to set project interpretor to this virtual enviornment.

For all following oprations, make sure the newly crearted virtual enviornment is activated. The command line should show the name of virtual enviornment in brackets:
`(catkin_cdcpd) username@hostname:`

clone repositories into your catkin workspace:
```
git clone git@github.com:UM-ARM-Lab/cdcpd.git
git clone git@github.com:eric-wieser/ros_numpy.git
```

install the rest of the requirements via pip:
```
(catkin_cdcpd) username@hostname: pip install -r cdcpd/requirements.txt 
```
You may noticed that a package named [glplotlib](https://github.com/cheng-chi/glplotlib) is installed by pip. glplotlib is another opensource library developed by Cheng Chi for virualizing 3D point cloud with better performance (comparing to matplotlib).


If you need tracking failure recovery, you will also need to compile VFH feature library.
Make sure you have PCL-1.8 installed. If you have ROS installed, its should be already there.
```
cd src/pcl_features
mkdir build
cd build
cmake ..
make
cp libfeatures.so ../libfeatures.so
cd ..
rm -r build
```

Demo
------------
To run the demo, you need download some [data](https://drive.google.com/drive/folders/1QSmSOw0JvQl9xnbVnBNogk0OcNo0Rn4_?usp=sharing) into <project_root>/data folder.
Files ends with ".bag" are rosbag files and are used for testing the ROS node.
Files ends with ".pk" are python pickle files extracted from corresponding ".bag" files, used in examples.

To run cdcpd demo:
```
python examples/basic_cdcpd_example.py
```

To run ROS node:

All following in commands need to be running at the same time, you may start one terminal for each:\
Start ROS core: `roscore`\
Start Rviz for visualization: `rviz`\
Start tracking node: `python ros_nodes/simple_cdcpd_node.py`\
Start playing rosbag: `rosbag play -l data/rope_simple.bag`
