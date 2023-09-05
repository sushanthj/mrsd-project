---
layout: page
title: Simulation Development
nav_order: 1
has_children: true
permalink: /docs/Simulation
---

# Building the Simulation environment in Gazebo

## Running the Neobotix simulation

The neo_simulation2 package uses ros2 Humble and will not run on the foxy docker which the
team will be using for most of the development. Therefore, a separate docker container was
setup to run this simulation natively.

[This Repository for Reference](https://github.com/DockDockGo/robot-setup-tool)

## References Used in Building This

1. [Neobotix Documentation](https://neobotix-docs.de/ros/ros2/simulation.html#configuration-and-launch)
2. [SLAM_toolbox](https://www.youtube.com/watch?v=rZOxPGCn4QM&ab_channel=TheConstruct)