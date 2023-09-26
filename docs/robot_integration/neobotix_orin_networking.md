---
layout: page
title: PCL Localization Integration
parent: Robot Software Integration
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

# Setup for Localization

Created by: Sushanth Jayanth
Created time: September 25, 2023 11:30 AM

This instruction set assumes that you are inside the `mfi_amr_ws` docker image that lies inside the host machine

- Initialize the docker image and hop into it

```bash
ros2_foxy_docker
```

- Once inside the docker launch multiple windows and `source install/setup.bash` in each of them.
- For the pcl_localization package, check the localization.yaml file and ensure the right path to the .pcd file has been used. Additionally, ensure it is subscribing to the right odom topic (here rf2o is the source of odometry)
- Then run the following commands in the below order:

```bash
ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py

ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py

ros2 launch pcl_localization_ros2 pcl_localization_with_odom.launch.py

rviz2
```

Once the above nodes are running rviz will show the pcl-localization pointcloud along with the pre-made map (saved as a .pcd)

A few caveats for pcl_localization_ros2

- We need to make sure that velodyne (LIDAR) frame is tied somewhere to the robot (here we set it as static transform). This is found in the the launch file for `pcl_localization_ros2`

```python

def generate_launch_description():

    ld = launch.LaunchDescription()

    lidar_tf = launch_ros.actions.Node(
        name='lidar_tf',
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0','1','robot2/base_link','velodyne']
        )
```

![](/images/robot_bringup/Localization_Integration/tf_tree_1.png)

- Further, to connect the odom to the robot’s base link, we will need to do

```bash
#include <pcl_localization/pcl_localization_component.hpp>
PCLLocalization::PCLLocalization(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("pcl_localization", options),
  broadcaster_(this)
{
  declare_parameter("global_frame_id", "map");
  declare_parameter("odom_frame_id", "/robot2/odom");
  declare_parameter("base_frame_id", "velodyne");
  declare_parameter("registration_method", "NDT");
```

- The changes results in the following TF tree as shown below

![](/images/robot_bringup/Localization_Integration/tf_tree_2.png)