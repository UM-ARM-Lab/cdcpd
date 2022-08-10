---
title: Testing
nav_order: 3
has_children: True
---

# Testing

CDCPD uses gtest for its testing framework. This is convenient because catkin offers very easy gtest integration with ROS. To run all unit tests for CDCPD, execute the following from your catkin_ws/src directory:

```
catkin test cdcpd
```

This will build and execute all CDCPD unit tests to ensure the package was installed without error.