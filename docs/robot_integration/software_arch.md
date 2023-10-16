---
layout: page
title: High Level Navigation and Comms Integration
parent: Robot Software Integration
nav_order: 3
---

<details open markdown="block">
  <summary>
    Table of contents
  </summary>
  {: .text-delta }
1. TOC
{:toc}
</details>

# High Level Information Flow

![](/images/robot_bringup/High%20Level.png)

# State Machine and Motion Action Servers

![](/images/robot_bringup/low_level_v2.png)

## State Machine

TBD

## Motion Action Server

Each node of motion action server represents one form of Navigation

- Navigation Action Server
- Dock/Undock Action Server

Run:

```bash
ros2 launch robot_motion_server motion_servers.launch.py
# or
ros2 run robot_motion_server robot_motion_server_node

# then
ros2 run robot_action_interfaces robot_pose_server.py

# then two action client calls
ros2 action send_goal /Navigate robot_action_interfaces/action/Navigate "{secs: 1.0}"

ros2 action send_goal /DockUndock robot_action_interfaces/action/DockUndock "{secs: 1.0}"
```


# Navigation, Planner and Localization Setup to test Integration

```bash
ros2_foxy_docker

ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py

ros2 launch pcl_localization_ros2 pcl_localization_with_odom.launch.py

ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py # optional

ros2 launch nav2_map_server_start nav2_custom_bringup.launch.py

ros2 launch rviz2
```

The nav2_configs has an rviz config which should be used