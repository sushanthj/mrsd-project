---
layout: page
title: Working on Neobotix Compute
parent: Robot Bringup
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


# Testing Mapping using the Neobotix's 2D LIDAR

1. Set the robot namespace (ros2 domain ID) by ```export ROS_DOMAIN_ID=2```
2. Check if correct namespace by ```printenv ROS_DOMAIN_ID```
3. colcon build --packages-select-regex

4. ros2 launch neo_mp_400-2 bringup.launch.py
5. ros2 launch nwo_mp_400-2 rviz.launch.py robot_namespace:=robot2 use_namespace:=True
6. ros2 launch neo_mp_400-2 mapping.launch.py robot_namespace:=robot2

# Running Teleop

