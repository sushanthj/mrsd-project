---
layout: page
title: AMCL
parent: Robot Localization
nav_order: 1
---

<details open markdown="block">
  <summary>
    Table of contents
  </summary>
  {: .text-delta }
1. TOC
{:toc}
</details>


# Requirements to run AMCL

To understand what's needed to run AMCL let's take a look at the launch file in Nav2

# Overview

1. Start Odometry, VLP, and all_static_tf nodes
2. Run AMCL
    ```bash
    ros2 launch nav2_bringup localization_launch.py map:=/root/neobotix_workspace/src/neo_simulation2/maps/neo_workshop.yaml params_file:=/root/neobotix_workspace/src/slam_toolbox_launch/config/nav2_params.yaml initial_pose_x:=-100 initial_pose_y:=-100 initial_pose_z:=0 use_sim_time:=true
    ```
3. Give AMCL an init pose (All these localization methods need a prior!)
    ```bash
    ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped  "{header: {stamp: {sec: 0}, frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
    ```
4. Run RVIZ2 from neo_nav2_bringup (clone neo_nav2_bringup if you don't have it)
  ```bash
  ros2 launch neo_nav2_bringup rviz_launch.py
  ```

## Next steps:

1. Do localization using SLAM toolbox
2. Document neo_localization
3. Build Velodyne from source and try!