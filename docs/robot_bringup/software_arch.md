---
layout: page
title: Localization-Planner-Offboard Comms Integration
parent: Robot Bringup
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

# State Change Handling

The nodes shown below live on the robot execpt global planner and offboard comms
which are on the offboard server

![](/images/Software_Arch/Robot_control_arch.png)

Run:
```bash
ros2 run robot_action_interfaces robot_pose_server.py

ros2 run robot_motion_server robot_motion_server_node

ros2 action send_goal /DockUndock robot_action_interfaces/action/DockUndock "{secs: 1.0}"
```