---
layout: page
title: Navigating in Simulation
parent: Simulation Development
nav_order: 5
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

A lot of this setup has already been done on a docker container which can be pulled
as ```docker pull sushanthj/humble_sim_mapping_built:latest```

Further information is available on [This Repository for Reference](https://github.com/DockDockGo/robot-setup-tool)


# Start by Fixing neo_simulation2's local planner

Neobotix's neo_simulation2 has a local planner defined as **controller** in
[this folder](https://github.com/neobotix/neo_simulation2/blob/humble/configs/mp_400/navigation.yaml)

The same is shown below. All the changes described will need to be replicated in the
neo_simulation2 package folder which you would have cloned

```yaml
controller_server:
  ros__parameters:
    # controller server parameters (see Controller Server for more info)
    use_sim_time: True
    controller_frequency: 50.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    controller_plugins: ["FollowPath"]
    goal_checker_plugins: ["general_goal_checker"]
    progress_checker_plugin: "progress_checker"
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 100.0
    general_goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.05
      yaw_goal_tolerance: 0.05
      stateful: True
    # RegulatedPurePursuitController Parameters
    FollowPath:
      plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      desired_linear_vel: 0.5
      max_linear_accel: 0.2
      max_linear_decel: 0.2
      lookahead_dist: 0.6
      min_lookahead_dist: 0.3 #0.3
      max_lookahead_dist: 0.9 #0.9
      lookahead_time: 1.5 #1.5
      rotate_to_heading_angular_vel: 0.6
      transform_tolerance: 0.1
      use_velocity_scaled_lookahead_dist: false
      min_approach_linear_velocity: 0.01
      use_approach_linear_velocity_scaling: true
      max_allowed_time_to_collision: 1.0
      use_regulated_linear_velocity_scaling: true
      use_cost_regulated_linear_velocity_scaling: false
      regulated_linear_scaling_min_radius:  0.9
      regulated_linear_scaling_min_speed: 0.25
      use_rotate_to_heading: true
      rotate_to_heading_min_angle: 0.785
      max_angular_accel: 0.6
      allow_reversing: false

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 10.0
      publish_frequency: 1.0
      global_frame: odom
      robot_base_frame: base_link
      footprint_padding: 0.
      footprint:  "[ [0.30,0.30],[-0.30,0.30],[-0.30,-0.3],[0.30,-0.3] ]"
      use_sim_time: True
      rolling_window: true
      width: 5
      height: 5
      resolution: 0.02
      # robot_radius: 0.22
      plugins: ["obstacle_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 5.0
        inflation_radius: 0.8
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          obstacle_max_range: 5.0
          max_obstacle_height: 2.0
          obstacle_min_range: 0.0
          raytrace_max_range: 8.0
          raytrace_min_range: 0.0
          clearing: True
          marking: True
          data_type: "LaserScan"
      always_send_full_costmap: True

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      footprint_padding: 0.0
      footprint:  "[ [0.30,0.30],[-0.30,0.30],[-0.30,-0.3],[0.30,-0.3] ]"
      use_sim_time: True
      resolution: 0.05
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          obstacle_max_range: 5.0
          max_obstacle_height: 2.0
          obstacle_min_range: 0.0
          raytrace_max_range: 8.0
          raytrace_min_range: 0.0
          clearing: True
          marking: True
          data_type: "LaserScan"
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
        enabled: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 4.5
        inflation_radius: 0.8
      always_send_full_costmap: True
```

However the above planner is weird and does not work well. Hence we can replace it with the planner
found [Here in mp700 package](https://github.com/neobotix/neo_simulation2/blob/humble/configs/mpo_700/navigation.yaml)

The same is shown below:

```yaml
controller_server:
  ros__parameters:
    # controller server parameters (see Controller Server for more info)
    controller_plugins: ["FollowPath"]
    controller_frequency: 100.0
    controller_plugin_types: ["neo_local_planner::NeoLocalPlanner"]
    goal_checker_plugins: ["general_goal_checker"]
    progress_checker_plugin: "progress_checker"
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 100.0
    general_goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.05
      yaw_goal_tolerance: 0.05
      stateful: True
    FollowPath:
      plugin: "neo_local_planner::NeoLocalPlanner"
      acc_lim_x : 0.25
      acc_lim_y : 0.25
      acc_lim_theta : 0.8
      max_vel_x : 0.8
      min_vel_x : -0.2
      max_vel_y : 0.5
      min_vel_y : -0.5
      max_rot_vel : 0.8
      min_rot_vel : -0.8
      max_trans_vel : 0.8
      min_trans_vel : 0.1
      yaw_goal_tolerance : 0.005
      xy_goal_tolerance : 0.01
      goal_tune_time : 0.5
      lookahead_time : 0.4
      lookahead_dist : 1.0
      start_yaw_error : 0.5
      pos_x_gain : 1.0
      pos_y_gain : 1.0
      static_yaw_gain : 3.0
      cost_x_gain : 0.1
      cost_y_gain : 0.1
      cost_y_lookahead_dist : 0.0
      cost_y_lookahead_time : 0.3
      cost_yaw_gain : 2.0
      low_pass_gain : 0.2
      max_cost : 0.95
      max_curve_vel : 0.4
      max_goal_dist : 0.5
      max_backup_dist : 0.0
      min_stop_dist : 0.2
      differential_drive : false
      allow_reversing: false

      # plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      # desired_linear_vel: 0.5
      # max_linear_accel: 0.8
      # max_linear_decel: 0.8
      # lookahead_dist: 0.6
      # min_lookahead_dist: 0.3 #0.3
      # max_lookahead_dist: 0.9 #0.9
      # lookahead_time: 1.5 #1.5
      # rotate_to_heading_angular_vel: 0.9
      # transform_tolerance: 0.1
      # use_velocity_scaled_lookahead_dist: false
      # min_approach_linear_velocity: 0.01
      # use_approach_linear_velocity_scaling: true
      # max_allowed_time_to_collision: 1.0
      # use_regulated_linear_velocity_scaling: true
      # use_cost_regulated_linear_velocity_scaling: false
      # regulated_linear_scaling_min_radius:  0.9
      # regulated_linear_scaling_min_speed: 0.25
      # use_rotate_to_heading: false
      # rotate_to_heading_min_angle: 0.785
      # max_angular_accel: 1.0
      # allow_reversing: true

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 10.0
      publish_frequency: 1.0
      global_frame: odom
      robot_base_frame: base_footprint
      footprint_padding: 0.
      footprint:  "[ [0.45,0.4],[-0.45,0.4],[-0.45,-0.4],[0.45,-0.4] ]"
      use_sim_time: True
      rolling_window: true
      width: 5
      height: 5
      resolution: 0.02
      # robot_radius: 0.22
      plugins: ["obstacle_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 4.0
        inflation_radius: 1.0
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan scan1
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
        scan1:
          topic: /scan2
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
      always_send_full_costmap: True

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 0.4
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_footprint
      footprint_padding: 0.
      footprint:  "[ [0.45,0.4],[-0.45,0.4],[-0.45,-0.4],[0.45,-0.4] ]"
      use_sim_time: True
      # robot_radius: 0.22
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan scan1
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
        scan1:
          topic: /scan2
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 4.0
        inflation_radius: 1.0
      always_send_full_costmap: True
```


# Running Navigation

We will open three tmux panes and run the following command (in same order and in quick
succession)

1. colcon build --symlink-install
2. source install/setup.bash
3. export MY_ROBOT=mp_400
4. export MAP_NAME=neo_workshop
5. ros2 launch neo_simulation2 simulation.launch.py
6. ros2 launch neo_simulation2 navigation.launch.py map:=/root/neobotix_workspace/src/neo_mp_400-2/configs/navigation/sush_map.yaml
7. ros2 launch neo_nav2_bringup rviz_launch.py


# Expected Output

![](/images/Simulation/navigation_1.png)