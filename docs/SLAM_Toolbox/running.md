---
layout: page
title: Running SLAM Toolbox
parent: Understanding SLAM Toolbox
nav_order: 2
---

<details open markdown="block">
  <summary>
    Table of contents
  </summary>
  {: .text-delta }
1. TOC
{:toc}
</details>

# Debugging

## Map saver timeout

This happens when the map is large and needs more time to save. Default is 5s but to override
we can run the following command:

```bash
ros2 run nav2_map_server map_saver_cli -f full_map_one --ros-args -p save_map_timeout:=10000
```

Note. **Remember you need to save a serialized map as well to use for localization later!**

```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "filename: 'full_map_serial'"
```

## Installing the rf2o_laser_odometry

This package acts weird when trying to build it at the root folder of the workspace.

It throws error **"catkin_pkg not found"**

**The Solution :** Just navigate to the package folder itself and build there  

# Appendix

## robot_localization vs AMCL in the context of ROS2

robot_localization is somewhat poorly named - at this point it is mostly an Extended Kalman Filter (EKF), usually it is used to merge multiple sources of odometry information (most commonly, IMU and wheel odometry, although it also provides some tools for GPS). Thus, robot_localization is used to create the odom->base_link transform.

AMCL is actually localization software - it provides the map->odom transform using a particle filter to determine where the robot is in within a map (usually using a planar laser scanner and the odometry information).

Therefore, these two can actually work together - by combining multiple sources of odometry information you can minimize the odometry drift that occurs in your odom->base_link estimation. Then AMCL can be used to correct for that drift by keeping the robot properly localized within the map.

## Installing RQT Graph

```bash
sudo apt install ros-humble-rqt-graph
```