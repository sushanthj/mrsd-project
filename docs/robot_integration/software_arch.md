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

## State Machine IRL

![](/images/dock_undock.gif)

## Motion Action Server

Each node of motion action server represents one form of Navigation

- Navigation Action Server
- Dock/Undock Action Server


### Configured Setup on Robots

- Each **robots on startup will launch all necessary nodes** (mentioned in detailed section below)
- The nodes will also be namespaced automatically
- Only the **Robot Mission Control** node will have to run manually on the server computer as of now

### Commands to Test State Machine and Mission Control

```bash
# sample state machine action goal to test simple navigation (ensure goal positions are valid in map)
ros2 action send_goal /StateMachine robot_action_interfaces/action/StateMachine "{start_dock_id: 1, end_dock_id: 2, goals: [{header: {frame_id: 'map'}, pose: {position: {x: -4.0, y: -1.7627, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.6626, w: 0.7489}}}]}"

# Send example command
ros2 action send_goal /MissionControl robot_action_interfaces/action/MissionControl "robot_specific_dock_ids: [1, 2]"
```

# Integration is Tiring

![](/images/robot_bringup/robot-dining.jpg)

### Detailed List of Nodes to Run

```bash
# Launch Nav2
ros2 launch neo_mp400-2 navigation.launch.py

# Launch DDG Waypoint Follower (one per robot)
# cd to the workspace
# source install/setup.bash
ros2 launch ddg_waypoint_follower ddg_waypoint_follower.launch.py use_sim:=True namespace:="robot1"

# Launch CBS Planner
ros2 launch ddg_multi_robot_planner multi_robot_planner.launch.py

# Launch motion action servers
ros2 launch robot_motion_server motion_servers.launch.py
# or
ros2 run robot_motion_server robot_motion_server_node

# Launch State Machine
# With Namespacing (multi robot)
ros2 launch robot_state_machine state_machine.launch.py use_namespace:=True namespace:="robot1"
# or
ros2 run robot_state_machine robot_state_machine_node use_namespace:=True namespace:="robot1"

# Without Namespacing (single robot)
ros2 launch robot_state_machine state_machine.launch.py

# Launch Mission Control
# With Namespacing (multi robot)
ros2 launch robot_mission_control mission_control.launch.py

# For Single Robot
ros2 launch robot_mission_control mission_control_single.launch.py
```


# Navigation, Planner and Localization Nodes to Start

```bash
ros2_foxy_docker

ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py

ros2 launch pcl_localization_ros2 pcl_localization_with_odom.launch.py

ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py # optional

ros2 launch nav2_map_server_start nav2_custom_bringup.launch.py

ros2 launch rviz2
```