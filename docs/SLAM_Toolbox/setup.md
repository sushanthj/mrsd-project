---
layout: page
title: Setup for SLAM Toolbox
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


# Nodes to Start

- velodyne
- laser odometry
- slam_toolbox_launch
- slam_toobox_tf2
- repub_velo

# Sensors, Frames, and Pipeline

The SLAM toolbox is super picky in that it needs TF2 frames and sensor data published in specific
topics to even start running.

This document highlights some of the pipelining required to make slam toolbox run.

## TF2 Transforms

[Github Repo of Package](https://github.com/DockDockGo/slam_toolbox_tf2){: .btn .fs-3 .mb-4 .mb-md-0 }

The SLAM toolbox needs few specific transformations with the TF tree looking like:

![](/images/slam_toolbox/TF_tree.png)

However, the above picture was taken from simulation where multiple links were shown. We will
only worry about the following frames:

```
├── odom
│   ├── base_footprint
|       ├── base_link
|           ├── lidar_1_link
```

The below frames I'm guessing is necessary only during initialization of slam_toolbox node.
This is becasue the slam_toolbox is meant to publish it's own map frame as well. However for
**some weird stupid reason** it needs this map -> lidar_1_link as well.
```
├── map
│   ├── lidar_1_link
```

### Motivation

- odom frame is just our **initial robot pose** which is instantiated the moment we start
    the robot
- base_footprint frame is necessary to map where the robot would lie w.r.t the ground plane
    - Since our robot moves only in 2D, it would always stick to the ground plane
- base_link is the body frame of the robot. Since we move in 2D base_link = base_footprint
    - Since base_link = base_footprint, we use a static transform of 0 rotation and 0 translation

- odom -> base_footprint has to be updated by whichever node is publishing odometry information
- Since we are currently using LIDAR odometry, the base_link = lidar_1_link and is therefore
    another 0 rotation and 0 translation transform

### Interfacing with slam_toolbox

The slam_toolbox utilizes the above existing TF tree and when it's running it should look like
below:

```
├── map
|    ├── odom
│       ├── base_footprint
|           ├── base_link
|               ├── lidar_1_link
```

- As we move the robot around, the odom -> lidar_1_link transform will get continually updated
even without the slam_toolbox.
- However, it's the job of the slam_toolbox to correct the map -> lidar_1_link frame and thereby
    also correct the odom frame.
- A simple explanation is present in the starting part of this [video](https://www.youtube.com/watch?v=ZaiA3hWaRzE&t=1033s&ab_channel=ArticulatedRobotics) and a small vizualization is shown below:

![](/images/slam_toolbox/tf2_slam_toolbox.png)

## Launching SLAM toolbox with Config Files

[Github Repo of Package](https://github.com/DockDockGo/slam_toolbox_launch){: .btn .fs-3 .mb-4 .mb-md-0 }

This package only contains launch and config files which call the slam_toolbox executables.
Note. presently the slam_toolbox is installed as binaries from the apt repository

### Which launch file to use?

1. The online_sync file would be preferred as it allows for better loop closure
2. The async is less computationally intensive as it does not process every frame that it receives

### Config file nitty-gritties

The config file has a few parameters which must be set right. As per the TF tree we developed
in [slam_toolbox_tf2](https://github.com/DockDockGo/slam_toolbox_tf2) and [repub_velo](https://github.com/DockDockGo/repub_velo) repositories, we configure the slam_tooblox to look at specific topics as shown below:

```yaml
  slam_toolbox:
  ros__parameters:

    # Plugin params
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG
    ceres_loss_function: None

    # ROS Parameters
    odom_frame: odom
    map_frame: map
    base_frame: base_footprint
    scan_topic: /scan_new
    mode: mapping #localization
```

The mode here is set to mapping, but as shown it can also be set to localization. However,
in localization mode, a pre-existing map and the path to that map should be configured in the
same config file (not shown in here, since I'm yet to do localization myself).

## Republishing Velodyne Topic

