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
  * rospy\**
  * ros_numpy\**
  
\*: (optional) for visualization only

\**: (optional) for ROS node only

Installation
------------

#### Core Package

install miniconda https://docs.conda.io/en/latest/miniconda.html

create conda virtual enviornment, you can name it whatever you want:
```
conda create --name cpdenv python
conda activate cpdenv
```

If you use a IDE such as pycharm or vscode, remember to set project interpretor to this virtual enviornment.

For all following oprations, make sure the newly crearted virtual enviornment is activated. The command line should show the name of virtual enviornment in brackets:
`(cpdenv) username@hostname:`

clone repository:
```
git clone git@github.com:UM-ARM-Lab/cdcpd.git --branch v0.1.0 --single-branch
```

install pip package:
```
cd cdcpd
pip install -e .
```

-e installs the library in edit mode, thus changes made in the repository will be affected globally
. means install package (defined by setup.py) in current directory.

You may noticed that a package named [glplotlib](https://github.com/cheng-chi/glplotlib) is installed by pip. glplotlib is another opensource library developed by Cheng Chi for virualizing 3D point cloud with better performance (comparing to matplotlib).

install opencv:
```
conda install opencv
```

Gurobi as a propriotoray optimization package that we use. Please obtain a [free academic license](https://user.gurobi.com/download/licenses/free-academic)

install gurobi:
```
conda install gurobi -c gurobi
```
install gurobi key with [grbgetkey](https://www.gurobi.com/documentation/8.1/quickstart_mac/retrieving_a_free_academic.html).
Note that gurobi installation requires university network. If you are not on campus, follow this instruction to setup [UMVPN](https://documentation.its.umich.edu/vpn/vpn-linux-vpn-instructions).


If you need tracking failure recovery, you will also need to compile VFH feature library.
Makesure you have PCL-1.8 installed. If you have ROS installed, its should be already there.
```
cd cdcpd/pcl_features
mkdir build
cd build
cmake ..
make
cp libfeatures.so ../libfeatures.so
cd ..
rm -r build
```

If you need the ROS node, ROS related installation are needed:
```
pip install rosdep
git clone git@github.com:eric-wieser/ros_numpy.git
cd ros_numpy
pip install .
```
Note that ROS related code are only tested with ROS Melodic on Ubuntu 18.04.

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
