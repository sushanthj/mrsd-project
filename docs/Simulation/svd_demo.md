---
layout: page
title: Creating Custom World for SVD
parent: Simulation Development
nav_order: 6
---

<details open markdown="block">
  <summary>
    Table of contents
  </summary>
  {: .text-delta }
1. TOC
{:toc}
</details>

# Before you Begin

Please use [This Repository for Reference](https://github.com/DockDockGo/robot-setup-tool)

Also, a base setup has already been done on a docker container which can be pulled
as ```docker pull sushanthj/humble_sim_mapping_built:latest```

The docker is not complete, but all files that need to be added to the docker container can
be found [in this location](https://github.com/DockDockGo/robot-setup-tool/tree/ros2/world_files/todo_copy_to_docker)
or is also present [here](https://drive.google.com/file/d/1RXruH6-E_zZsusRcH2XPh03xjsMYtpxM/view?usp=drive_link)

# Folder Structure and New Files Added

The workspace presenent in root/neobotix_workspace in the docker conatiner mentioned above will
look like the following:

![](/images/Simulation/neobotix_ws_folder_struct.png)

As you can see, we have expanded only the neo_simulation2 package. To add the custom world
we built onto the simulation we will need to make the following changes:

1. Find where [this folder](https://github.com/DockDockGo/robot-setup-tool/tree/ros2/world_files/todo_copy_to_docker) is located
2. Copy all the contents of the ```to_copy_sdfs``` folder (excluding svd_demo.world or any world files) to the
```/root/neobotix_workspace/src/neo_simulation2/models``` folder. This is where the simulator will
look for all the sdf files in the world
3. Copy the ```svd_demo.world``` present (or any other world file you created) to ```/root/neobotix_workspace/src/neo_simulation2/worlds```
4. Run ```export MY_ROBOT=mp_400```
5. Run ```export MAP_NAME=svd_demo```
6. Run ```colcon build --symlink-install```

# Testing

Trying out the above changes can be done as shown in the prior sections by running either:

- ```ros2 launch neo_simulation2 simulation.launch.py```
- ```ros2 launch neo_simulation2 navigation.launch.py map:=/root/neobotix_workspace/src/neo_mp_400-2/configs/navigation/sush_map.yaml```
- ```ros2 launch neo_nav2_bringup rviz_launch.py```

# Sample Output in New World File

![](/images/Simulation/sample_svd_output.png)