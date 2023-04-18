---
layout: page
title: AMCL
parent: Robot Localization
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


# Requirements to run AMCL

To understand what's needed to run AMCL let's take a look at the launch file in Nav2

# Overview (Quickly setup AMCL)

1. Start Odometry, VLP, and all_static_tf nodes
2. Run AMCL
    ```bash
    ros2 launch nav2_bringup localization_launch.py map:=/root/neobotix_workspace/src/neo_simulation2/maps/neo_workshop.yaml params_file:=<path_to>params.yaml use_sim_time:=true
    ```
3. Give AMCL an init pose (All these localization methods need a prior!)
    ```bash
    ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped  "{header: {stamp: {sec: 0}, frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
    ```
4. Run RVIZ2 from neo_nav2_bringup (clone neo_nav2_bringup if you don't have it)
  ```bash
  ros2 launch neo_nav2_bringup rviz_launch.py
  ```


## Which package to use for localization in general? (Better way to run AMCL)

Above we use the nav2's package directly to launch amcl. However, we can also use neobotix's
package which does essentially the same thing.

This package is **neo_nav2_bringup**. However, the launch files for this are at **neo_simulation2**.

### Running AMCL

- modify the param file which we will use for AMCL (located in ```/src/neo_nav2_bringup/config/navigation.yaml```)
- modify navigation.launch.py in neo_simulation2 to launch AMCL
- run ```ros2 launch neo_simulation2 navigation.launch.py map:=/root/neobotix_workspace/src/neo_simulation2/maps/neo_workshop.yaml```
- Give init pose and launch rviz as shown above
- Configure the ParticleCloud topic in rviz by modifying the arrow max_len to 2 and min_lin to 0.5

#### Modified param File
The param file mentioned above ```navigation.yaml``` needs to be modified to run AMCL
and is shown below:

```yaml
amcl:
  ros__parameters:
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_footprint"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05
    scan_topic: scan
```

#### Modified Launch File

Set the 'use_amcl' param to true to use AMCL or 

```python
# Neobotix GmbH
# Author: Pradheep Padmanabhan

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch.conditions import IfCondition


MY_NEO_ROBOT = os.environ['MY_ROBOT']
MY_NEO_ENVIRONMENT = os.environ['MAP_NAME']

def generate_launch_description():
    use_multi_robots = LaunchConfiguration('use_multi_robots', default='False')
    use_amcl = LaunchConfiguration('use_amcl', default='False')
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    namespace = LaunchConfiguration('namespace', default='')
    use_namespace = LaunchConfiguration('use_namespace', default='False')
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('neo_simulation2'),
            'maps',
            MY_NEO_ENVIRONMENT+'.yaml'))

    param_file_name = 'navigation.yaml'
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('neo_simulation2'),
            'configs/'+MY_NEO_ROBOT,
            param_file_name))

    nav2_launch_file_dir = os.path.join(get_package_share_directory('neo_nav2_bringup'), 'launch')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/localization_neo.launch.py']),
            condition=IfCondition(PythonExpression(['not ', use_amcl])),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'use_multi_robots': use_multi_robots,
                'params_file': param_dir,
                'namespace': namespace}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/localization_amcl.launch.py']),
            condition=IfCondition(use_amcl),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'use_multi_robots': use_multi_robots,
                'params_file': param_dir,
                'namespace': namespace}.items(),
        ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([nav2_launch_file_dir, '/navigation_neo.launch.py']),
        #     launch_arguments={'namespace': namespace,
        #                       'use_sim_time': use_sim_time,
        #                       'params_file': param_dir}.items()),
        ])

```

### Running localization_neo

- The procedure for this is to use the same launch file shown above
- However, we don't need to modify the config file
- Only ensure the ```use_amcl``` flag is set to false in launch file

## Next steps:

1. Do localization using SLAM toolbox
2. Document neo_localization
3. Build Velodyne from source and try!