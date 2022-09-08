# Constrained Deformable Coherent Point Drift (CDCPD & CDCPD2)  <!-- omit in toc -->

CDCPD2 is an implementation of *Tracking Partially-Occluded Deformable Objects while Enforcing Geometric Constraints*
by Yixuan Wang, Dale McConachie and Dmitry Berenson.

The master branch is the version the users outside the lab should use.

## Table of Contents <!-- omit in toc -->

- [Requirements](#requirements)
- [Installation](#installation)
  - [Installing ROS (ignore it if you already have it)](#installing-ros-ignore-it-if-you-already-have-it)
  - [Installing Dependencies](#installing-dependencies)
  - [Create catkin workspace](#create-catkin-workspace)
  - [Gurobi Licence](#gurobi-licence)
  - [Building](#building)
  - [Testing](#testing)
- [Usage](#usage)
  - [Rope Tracking](#rope-tracking)
  - [Cloth Tracking](#cloth-tracking)
- [Demos](#demos)
  - [Virtual Demos](#virtual-demos)
    - [Downloading The rosbags](#downloading-the-rosbags)
    - [Running The Demos](#running-the-demos)
    - [Demo Descriptions](#demo-descriptions)
  - [Demos With Sensors](#demos-with-sensors)
- [Adding gtest Unit Tests](#adding-gtest-unit-tests)
- [FAQ & Misc Notes](#faq--misc-notes)

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


## Installation

### Installing ROS (ignore it if you already have it)

Run `sudo -u USER_NAME install_scripts/install_ros_melodic.sh` if you use Ubuntu 18.04, or `sudo -u USER_NAME install_scripts/install_ros_noetic.sh` if you use Ubuntu 20.04

### Installing Dependencies

Modify USR\_NAME in `install_scripts/install_dep.sh` and run `sudo -u USER_NAME ./install_dep.sh` under `install_scripts`. It will install all dependency listed above in `~/.local`.

NOTE: `source ~/.bashrc` inside `install_dep.sh` might not run successfully according to the platform. If you encounter the problem like `catkinonfig.cmake` not found, please run `source ~/.bashrc` and run `./install_pybind11_catkin.sh`.

### Create catkin workspace

We assume you have created a catkin workspace. Now clone this repo to that worksace. See `install_scripts/create_ws_ROS_Version.sh` or the ROS wiki on how to setup a catkin workspace.

### Gurobi Licence

Gurobi is a proprietary optimization package that we use. Please obtain a [free academic license](https://www.gurobi.com/academia/academic-program-and-licenses).

### Building

```
# in the src directory
git clone https://github.com/UM-ARM-Lab/cdcpd.git
```

Once you've cloned, it might be a good idea to `rosdep install -r --from-paths cdcpd -y` to get any ROS packages you might be depending on.

### Testing

CDCPD uses gtest for its testing framework. This is convenient because catkin offers very easy gtest integration with ROS. To run all unit tests for CDCPD, execute the following from your `cdcpd` directory:

```
./test_cdcpd.sh
```

This will build and execute all CDCPD unit tests to ensure the package was installed without error.

Note: there isn't much special about the `test_cdcpd.sh` bash script, it simply starts a roscore and executes the `catkin test cdcpd` command with some added flags for simplifying things.

## Usage

To configure CDCPD for tracking rope versus cloth, several rosparams must be set (recommended to be in your launch file).

### Rope Tracking

ROS params and descriptions:
- "deformable_object_type"
  - The type of the deformable object being tracked. In this case, a rope.
  - Type: string
  - Value: "rope"
- "num_points"
  - The number of points the tracked rope will have.
  - Type: int
- "max_rope_length"
  - The maximum length of the rope in meters.
  - Type: float

### Cloth Tracking

ROS params and descriptions:
- "deformable_object_type"
  - The type of the deformable object being tracked. In this case, a cloth.
  - Type: string
  - Value: "cloth"
- "length_initial_cloth"
  - The initial length of the cloth in meters.
  - Type: float
- "width_initial_cloth"
  - The initial width of the cloth in meters.
  - Type: float
- "grid_size_initial_guess_cloth"
  - The size (in meters) of one square of the cloth template grid. Note! This is only providing an initial guess for the cloth template grid size. The actual grid size will be adjusted based on how well the grid size guess divides the supplied initial length and width.
  - Type: float

## Demos

### Virtual Demos

We offer several rosbag files to showcase what nominal function of CDCPD should look like given a proper install and sensor configuration.

#### Downloading The rosbags

Before running the virtual demos you must download and decompress the rosbags. To do this:

1. Download the demos folder to your local machine.
   1. The link is: https://www.dropbox.com/sh/4nsnxu4a2cxm8ko/AAC0-FsuWTHUB8FWrvp5BqR0a?dl=0
   2. Simply click the link and download the "rosbags_compressed" zip folder.
2. Extract the folder in the link (rosbags_compressed) to `<your_cdcpd_repo>/demos`. This should result in a folder structure that looks like `<your_cdcpd_repo>/demos/rosbags_compressed/` that has all of the compressed rosbags in this folder.
3. Change directory to `<your_cdcpd_repo>/demos`
4. Run the rosbag decompression script with:
    ```
    ./unpack_rosbags.sh
    ```
    This unpacks all of the compressed rosbags in `demos/rosbags_compressed/` to the `demos/rosbags/`.

#### Running The Demos

To run the virtual demos:

1. Start a `roscore`
   1. If you already have a `roscore` running, make sure to `rosnode cleanup` before running the demos.
2. In another terminal, navigate to `<your_cdcpd_repo>/demos/`
3. Run the desired demo script, e.g.
    ```
    ./launch_demo1.sh
    ```

This will display an example of CDCPD running.

#### Demo Descriptions

Note that demos loop! If you see the tracking jump around in rviz, it's likely due to the demo starting over.

- Demo 1
  - A static rope. This is a good place to start to see how CDCPD converges to tracking a short, static rope.


### Demos With Sensors

To run with a realsense, without the obstacle or gripper constraints, try this:

```
roslaunch realsense2_camera rs_camera.launch enable_pointcloud:=true
rviz -d cdcpd/rviz/realsense.rviz  # From local cdcpd directory
roslaunch cdcpd realsense.launch
```


We also provide an example for the kinect, in our case the kinectv2, but as long as your camera node published RGB/D or point clouds it should be easy to adapt the launch file.

```
roslaunch kinect2_calibration_files kinect2_bridge_tripodA.launch  # ARMLab internal usage
rviz -d cdcpd/rviz/kinect.rviz  # From local cdcpd directory
roslaunch cdcpd kinect.launch
```

## Adding gtest Unit Tests

This project uses the `googletest` (`gtest` for short) framework for testing. This is the process of writing a new test suite:

Note: Test suite is used to refer to a new group of tests indepenedent from other tests. In this repository, we're making a new header file for each test suite and writing a new test suite for each class.

1. Make a new header file in the `cdcpd/tests/include` directory of the repository.
  - The name of the new header file should be the name of the class (or functionality) under test concatenated with Test.h, e.g. if you're testing a class named `MyClass`, the name of the header file would be `MyClassTest.h`.
2. In the new <test_suite>.h file, write the following as boilerplate:
    ```
    #include "<where_the_class_you're_testing_is_declared>"
    #include "gtest/gtest.h"

    // insert any test fixtures here.

    TEST(<test_suite_name>, <test_name>) {
      // insert code for test here using:
      //    ASSERT_*(x, y) when you want the test to halt on failure.
      //    EXPECT_*(x, y) when you want the test to continue upon failure.
      //                   The EXPECT_* is best practice.
    }
    ```
    where the "`*`" in `ASSERT_*` and `EXPECT_*` is meant to be filled in with whatever you would like to assert or expect, e.g. `EXPECT_TRUE(XXX)` if `XXX` should evaluate to be `true`.
    - Note that neither `<test_suite_name>` nor `<test_name>` should have any underscores in the name. This doesn't play nicely with `gtest`'s internal macro naming scheme.
    - You can read more about test fixtures in the google test documentation. They're *very* handy!
3. Write the first test that you would like.
4. In tests/main.cpp, `#include` your new test suite header file.

From here, the `cdcpd/tests/main.cpp` file will automatically discover all tests in your newly written test suite. No need to manually add function calls!

## FAQ & Misc Notes

**Q:** It runs without error but doesn't seem to be processing images, help!

**A:** We use a time synchronizer with "exact" time policy (the deafult). Therefore if your depth, color, and camera_info messages do not have exactly the same time stamps, the synchronizer will ignore it and nothing will happen.
