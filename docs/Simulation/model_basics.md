---
layout: page
title: Understand Model and World Files
parent: Simulation Development
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

# What is an SDF file? What's the difference between SDF and URDF?

SDF is an XML format that describes objects and environments for robot simulators, visualization, and control. Now it was developed originally for Gazebo, but URDF is the type of file format that ROS supports. 

One post said ```My understanding can be boiled down to this: URDF specifies a robot, but SDF also specifies a world for the robot to live in, which is a much larger set of things. Based on this premise, SDF is designed to represent a superset of everything that can be represented in URDF.```

# If URDF is better then?
