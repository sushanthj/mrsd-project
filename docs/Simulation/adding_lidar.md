---
layout: page
title: Adding Lidar in Simulation
parent: Simulation Development
nav_order: 7
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

# Understanding the Neobotix Simulation Codebase

If you look into the neo_simulation2 repository, you'll see that the information needed
to build the simulation is slightly spread apart.

![](/images/Simulation/adding_lidar/folder_struct.png)

When working with simulation we need three types of files:
1. **.sdf** which holds information on objects in the simulation like chairs, tables
2. **.world** files which describe the floorplan and the layout also in SDF format
3. **.dae** files which are meshes used for visualization
4. **.urdf** I'm not fully sure how sdf and urdf are different. But for this tutorial, I'm
going to be using them interchangebly

*Note. Both the SDF (model files) and the .world are xml files. Further, the SDF files are
more specifically URDF files (which have .urdf extension) but is inherently written in xml*

In neo_simulation2 pacakge these files are a bit spread apart, but each of them are used in
the simulation.

1. The LIDAR meshes used are present in neo_simulation2/components/sensors
2. The robot's meshes is present in neo_simulation2/robots/mp_400/meshes
3. The robot's SDF file (urdf) is present in neo_simulation2/robots/mp_400 with .urdf extension
4. The .world file which we choose by doing ```export MAP_NAME=svd_demo``` is saved in neo_simulation2/worlds

Those above files and paths are the only things you will need to worry about when modifying
the simulation.



# How to add any sensors to an extisting robot's URDF?

This was the first question I asked myself after figuring out the folder structure. Sadly,
there isn't any great online resource whcih gives a birds eye view of what needs to be done.

**To add any sensor and get sensor readings in ROS, we will need to create a plugin in gazebo
for that sensor. That's the first thing you need to know.**

Check [this out](https://classic.gazebosim.org/tutorials?tut=ros_gzplugins) to understand what types of plugins exist. Note that there are also third
party plugins like velodyne which are not mentioned here too.

## The hard way

Firstly, let's look at the hard way of adding a lidar, i.e. creating our own plugin. There's
a [great tutorial by gazebo](http://classic.gazebosim.org/tutorials?cat=guided_i&tut=guided_i1)
which I followed for this step.

A summary of the above tutorial is we write a velodyne.world file which will store all the
information on the geometry, a ray sensor, joints, and links all in one .world file.

![](/images/Simulation/adding_lidar/velodyne_rays.jpg)

Then, we'll add 3D meshes and noise to the sensor and finally upload it to the gazebo repo.

We will be using such a model which was already uploaded with the correct meshes (.dae files)
and the .world files already in place. This can be downloaded as a package into any ros
workspace from [Here](https://github.com/osrf/gazebo_tutorials/raw/master/guided_i/files/velodyne_hdl32.tar.gz)

The package called velodyne_hdl32 will have the following files:

![](/images/Simulation/adding_lidar/package.png)

Hence, by skipping a few steps in the looong tutorial, we will finally start from
[This Page](https://classic.gazebosim.org/tutorials?cat=guided_i&tut=guided_i5)

The tutorial will consist of creating and modifying two files
- velodyne.world
- velodyne_plugin.cc

### The Plugin

The plugin is not our focus here. The boilerplate of a plugin for this example is:
```cpp
#ifndef _VELODYNE_PLUGIN_HH_
#define _VELODYNE_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
  /// \brief A plugin to control a Velodyne sensor.
  class VelodynePlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: VelodynePlugin() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Just output a message for now
      std::cerr << "\nThe velodyne plugin is attach to model[" <<
        _model->GetName() << "]\n";
    }
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(VelodynePlugin)
}
#endif
```

The plugin will undergo modifications to give it an API and make it configurable, but in the end
we will be running cmake on this plugin.cc file and creating a **libvelodyne_plugin.so** file.

**This .so file is what interests us the most and is what will be used in any urdf or .world file
which we will be modifying**

### The .world file (can be .urdf also)

```xml
<?xml version="1.0" ?>
<sdf version="1.5">
  <world name="default">
    <!-- A global light source -->
    <include>
      <uri>model://sun</uri>
    </include>
    <!-- A ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- A testing model that includes the Velodyne sensor model -->
    <model name="my_velodyne">
      <include>
        <uri>model://velodyne_hdl32</uri>
      </include>

      <!-- Attach the plugin to this model -->
      <plugin name="velodyne_control" filename="libvelodyne_plugin.so"/>
    </model>

  </world>
</sdf>
```

Here we see how the ```"libvelodyne_plugin.so"``` file is useful. We will be directly
callling the plugin using that reference in the above .world file.

## The Easier Way : Using pre-built plugins

### Neobotix Example

If we look at the URDF for the robot present in ```/root/neobotix_workspace/src/neo_simulation2/robots/mp_400/mp_400.urdf```
the urdf file will reference the 2D lidar. Mainly, the following parts in the urdf are
noteworthy:

#### Sensor plugin

```xml
<gazebo reference="lidar_1_link">
    <sensor name="lidar_1_sensor" type="ray">
      <always_on>true</always_on>
      <pose>0 0 0 0 0 0</pose>
      <visualize>true</visualize>
      <update_rate>10</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>720</samples>
            <resolution>1</resolution>
            <min_angle>-1.48</min_angle>
            <max_angle>1.48</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.10</min>
          <max>30.0</max>
          <resolution>0.05</resolution>
        </range>
        <noise>
          <type>gaussian</type>
          <!-- Noise parameters based on published spec for Hokuyo laser
                   achieving "+-30mm" accuracy at range < 10m.  A mean of 0.0m and
                   stddev of 0.01m will put 99.7% of samples within 0.03m of the true
                   reading. -->
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </ray>
      <plugin filename="libgazebo_ros_ray_sensor.so" name="lidar_1">
      <ros>
        <!-- <namespace>   </namespace> -->
        <argument>~/out:=scan</argument>
      </ros>
      <!-- Set output to sensor_msgs/LaserScan to get same output type as gazebo_ros_laser -->
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>lidar_1_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>
```

#### Links and joints for sensor

```xml
<!--+++++++++++++++++++laserscanner_link++++++++++++++++++++++++-->
  <joint name="laser_1_joint" type="fixed">
    <axis xyz="0 1 0"/>
    <origin rpy="0 3.14 3.14" xyz="0.230 0 0.110"/>
    <parent link="base_link"/>
    <child link="lidar_1_link"/>
  </joint>
  <link name="lidar_1_link" type="laser">
    <inertial>
      <mass value="0.001"/>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.000001" iyz="0" izz="0.0001"/>
    </inertial>
    <visual>
      <origin rpy="1.57 0 0" xyz="-0.0 0 -0.06"/>
      <geometry>
        <mesh filename="package://neo_simulation2/components/sensors/SICK-MICROSCAN3.dae" scale="0.001 0.001 0.001"/>
      </geometry>
    </visual>
    <collision>
      <origin rpy="1.57 0 0" xyz="-0.0 0 -0.06"/>
      <geometry>
        <mesh filename="package://neo_simulation2/components/sensors/SICK-MICROSCAN3.dae" scale="0.001 0.001 0.001"/>
      </geometry>
    </collision>
  </link>
```

### Using pre-built lidar geometry and pre-built sensor plugin

To do this we can either:
1. Create a new mesh model of a lidar we want to use
2. Use a pre-built mesh model like [pre_built lidar](https://app.gazebosim.org/OpenRobotics/fuel/models/Lidar%203d%20v1)
3. Be lazy and use the model for 2D LIDAR for the 3D Lidar too :)


We'll go with the 3rd option here since we don't care about the visualization of the 3D lidar
but only the output of it. However, we'll be using the specific 3D velodyne plugin. The changes
necessary to install this is as follows:

- Before using the pre-built plugin, install it using
  ```bash
  sudo apt update
  sudo apt install ros-humble-velodyne-gazebo-plugins
  ```
- Change the launch file of neo_simulation2 (or use the modified launch file present 
  [here](https://github.com/DockDockGo/robot-setup-tool/blob/ros2/world_files/todo_copy_to_docker/to_copy_lidar/simulation.launch.py)). This new launch file only calls the modified URDF.

Similar to the Neobotix example, we'll split this work into a sensor plugin and links/joints:

#### Sensor Plugin

```xml
<!--++++++++++++++++++++++++++++++3D_LIDAR_SENSOR_PLUGIN_ADDITION+++++++++++++++++++++++++-->
  <gazebo reference="lidar_2_link">
    <sensor name="sensor_ray" type="ray">
      <always_on>true</always_on>
      <pose>0.0 0.0 0.0 0.0 0.0 0.0</pose>
      <visualize>true</visualize>
      <ray>
        <scan display="true">
            <horizontal>
                <samples>100</samples>
                <resolution>1.0</resolution>
                <min_angle>-3.14</min_angle>
                <max_angle>3.14</max_angle>
            </horizontal>
            <vertical>
                <samples>16</samples>
                <resolution>1.0</resolution>
                <!-- These min and max angle values may need to be changed according to lidar -->
                <min_angle>-0.5236</min_angle>
                <max_angle>0.5236</max_angle>
            </vertical>
        </scan>
        <range>
            <min>0.05</min>
            <!-- Range is controlled by the below max value -->
            <max>2.0</max>
            <resolution>0.05</resolution>
        </range>
        <noise>
          <type>gaussian</type>
          <!-- Noise parameters TBD
                  reading. -->
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </ray>
      <plugin filename="libgazebo_ros_velodyne_laser.so" name="lidar_2">
      <ros>
        <!-- <namespace>   </namespace> -->
        <argument>~/out:=velodyne_filtered</argument>
      </ros>
      <!-- Set output to sensor_msgs/LaserScan to get same output type as gazebo_ros_laser -->
      <frame_name>lidar_2_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>
```

#### Links and joints for sensor

The link definition was arbitrarily chosen. Simple trial and error may be needed to find
the right position

```xml
<!-- Note: lidar_1_link = 2D lidar, lidar_2_link = 3D lidar -->
  <joint name="lidar_1_joint" type="fixed">
    <axis xyz="0 0 1"/>
    <!-- Note: lidar height must be changed by the below xyz (z dictating the height) -->
    <origin rpy="0 3.14 3.14" xyz="0.230 0 0.610"/>
    <parent link="base_link"/>
    <child link="lidar_2_link"/>
  </joint>
  <link name="lidar_2_link" type="laser">
    <inertial>
      <mass value="0.001"/>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.000001" iyz="0" izz="0.0001"/>
    </inertial>
    <visual>
      <origin rpy="1.57 0 0" xyz="0.0 0 -0.36"/>
      <geometry>
        <mesh filename="package://neo_simulation2/components/sensors/SICK-MICROSCAN3.dae" scale="0.001 0.001 0.001"/>
      </geometry>
    </visual>
    <collision>
      <origin rpy="1.57 0 0" xyz="0.0 0 -0.36"/>
      <geometry>
        <mesh filename="package://neo_simulation2/components/sensors/SICK-MICROSCAN3.dae" scale="0.001 0.001 0.001"/>
      </geometry>
    </collision>
  </link>
```

### Visualizing the Added LIDAR

Run the following commands

1. colcon build --symlink-install
2. source install/setup.bash
3. export MY_ROBOT=mp_400
4. export MAP_NAME=neo_workshop
5. ros2 launch neo_simulation2 simulation.launch.py
6. rviz2
7. In rviz2 set the frame to lidar_2_link

![](/images/Simulation/adding_lidar/rviz_viz.png)

![](/images/Simulation/adding_lidar/gazebo.png)

As you can see, the lidar model and the actual point from which rays are eminating is
different. However, this should not affect the pointcloud received. If any changes in
lidar position needs to be made the URDF will need to be altrered.

# References

pre_built_plugin
- [ROS Block Sensor Example for 3D lidar](http://docs.ros.org/en/electric/api/gazebo_plugins/html/group__GazeboRosBlockLaser.html) outputs PointCloud msg type
- [3rd party velodyne plugin](https://answers.gazebosim.org//question/23928/block-laser-plugin-for-pointcloud2/) outputs
Publishing in PointCloud2 topic
- [Aux ref 1](https://www.youtube.com/watch?v=JJDebiniDBw&ab_channel=TheConstruct)
- [Aux ref 2](https://robotics.stackexchange.com/questions/26349/parameters-for-ros-gazebo-block-laser-plugin?rq=1)

