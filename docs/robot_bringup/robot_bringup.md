---
layout: default
title: Robot Bringup
nav_order: 1
has_children: true
permalink: /docs/robot_bringup
---

# Working with the Actual Neobotix Robot

The robot arrived last week (Sep 19th) and looks to be in great shape!

![](/images/robot_bringup/neobotix_first_look.jpg)

# Highlights

- The robot has a onboard compute which can be accessed by SSH (```ssh -X neobotix@192.168.0.100``` or 101 depending on the robot)
- Alternatively, one can use the HDMI port and the usb ports to connect the robot to a monitor and keyboard
- It runs some version of Ubuntu internally
- It has ROS2 Humble, Neobotix Packages built and ready to use