---
layout: page
title: Just the Environemnt in Gazebo
parent: Simulation Development
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

# Building Environment From a Floorplan

Gazebo provides a [**'Building Editor'**](https://classic.gazebosim.org/tutorials?cat=build_world&tut=building_editor)
to make floorplans in 2D and autogenerate walls, doors and windows.

![](/images/Simulation/blank_building_editor.png)

We will then use a floorplan (eg. from MFI Lego Testbed) as a reference (import using the button shown above):

![](/images/Simulation/feb28.png)

## Editing the Environment

1. We build walls everywhere required (click on the 'Wall' button and then click on workspace to begin drawing)
2. Add doors accordingly where walls are not present
3. Windows can be placed on any walls

![](/images/Simulation/building_floor_walls.png)

After placing the initial walls, double clicking on the walls gives us the option to edit the size and position of these walls

![](/images/Simulation/edited_walls.png)

## Saving the Floorplan

You can do ```File``` and ```Save As``` to save the floorplan. This will save three files:

- MODEL_NAME (given by user) eg. **mfi_floor_trial1**
  - model.config
  - model.sdf
  - world.sdf

Additionally, if you ever download a model from ```git clone https://github.com/osrf/gazebo_models``` then the following files will be present for:

- model_1 : A directory for model_1
  - model.config : Meta-data about model_1
  - model.sdf : SDF description of the model
  - model.sdf.erb : Ruby embedded SDF model description
  - meshes : A directory for all COLLADA and STL files
  - materials : A directory which should only contain the textures and scripts subdirectories
  - textures : A directory for image files (jpg, png, etc).
  - scripts : A directory for OGRE material scripts
  - plugins: A directory for plugin source and header files

## Example model.sdf (simulator description format)

```xml
<?xml version='1.0'?>
<sdf version='1.7'>
  <model name='mfi_floor_trial1'>
    <pose>0.011095 -0.309692 0 0 -0 0</pose>
    <link name='Wall_10'>
      <collision name='Wall_10_Collision'>
        <geometry>
          <box>
            <size>1.9 0.05 1</size>
          </box>
        </geometry>
        <pose>0 0 0.5 0 -0 0</pose>
      </collision>
      <visual name='Wall_10_Visual'>
        <pose>0 0 0.5 0 -0 0</pose>
        <geometry>
          <box>
            <size>1.9 0.05 1</size>
          </box>
        </geometry>
        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Grey</name>
          </script>
          <ambient>1 1 1 1</ambient>
        </material>
        <meta>
          <layer>0</layer>
        </meta>
      </visual>
      <pose>-6.0041 -1.14431 0 0 -0 3.14159</pose>
    <static>1</static>
  </model>
</sdf>
```

# Creating world files from the model we designed above

Now that we have a model in the **mfi_floor_trial1** folder, let's change the folder structure
a little bit.

```
├── worlds
├── models
│   ├── mfi_floor_trial1
|       ├── model.config
|       ├── model.sdf
|       ├── world.sdf
```


Now, open gazebo and load the above model **mfi_floor_trial1**. Then follow the below steps:

- Add any new objects onto this model (like sofa, chair, trashcan)
- save this as ```trial_world.world``` file in the worlds folder.

The final folder structure should look like below:

```
├── worlds
|   ├── trial_world.world
├── models
│   ├── mfi_floor_trial1
|       ├── model.config
|       ├── model.sdf
|       ├── world.sdf
```

# Adding models to existing neobotix world files

![](/images/Simulation/neo_workshop_custom.png)

## Barebones view of neobotix_workshop.world

This world file is provided by neobotix
[here](https://github.com/neobotix/neo_simulation2/blob/rolling/worlds/neo_workshop.world).
We will use this world file as our template and add models (walls, tables, trashcan and sofa)
to this world as required.

### Process Overview

1. The world file you created in the saving floorplan section called world.sdf will contain the
    info on walls and tables which we will need to use
2. The neobotix world file "neo_workshop.world" looks like this:
    ![](/images/Simulation/neo_workshop.orig.png)
3. We need to modify it to the below format

```xml
<sdf version='1.6'>
  <world name='default'>
    <light name='sun' type='directional'>
    </light>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type='adiabatic'/>
    <physics name='default_physics' default='0' type='ode'>
    </physics>
    <scene>
    </scene>
    <wind/>
    <spherical_coordinates>
    </spherical_coordinates>
    <model name='mfi_floor_trial1'>
      <pose>1.22275 -0.717257 0 0 -0 0</pose>
      <link name='Wall_10'>
      </link>
      <link name='Wall_14'>
      </link>
      <link name='Wall_16'>
      </link>
      <link name='Wall_20'>
      </link>
      <link name='Wall_21'>
      </link>
      <link name='Wall_4'>
      </link>
      <link name='Wall_5'>
      </link>
      <link name='Wall_6'>
      </link>
      <link name='Wall_7'>
      </link>
      <link name='Wall_8'>
      </link>
      <link name='Wall_9'>
      </link>
      <static>1</static>
    </model>
    <model name='table'>
    </model>
    <model name='table_0'>
    </model>
    <state world_name='default'>
      <sim_time>12155 846000000</sim_time>
      <real_time>151 139215044</real_time>
      <wall_time>1592070340 176042788</wall_time>
      <iterations>15065</iterations>
      <model name='mfi_floor_trial1'>
        <pose>1.22275 -0.717257 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='Wall_10'>
        </link>
        <link name='Wall_14'>
        </link>
        <link name='Wall_16'>
        </link>
        <link name='Wall_20'>
        </link>
        <link name='Wall_21'>
        </link>
        <link name='Wall_4'>
        </link>
        <link name='Wall_5'>
        </link>
        <link name='Wall_6'>
        </link>
        <link name='Wall_7'>
        </link>
        <link name='Wall_8'>
        </link>
        <link name='Wall_9'>
        </link>
      </model>
      <model name='table'>
      </model>
      <model name='table_0'>
      </model>
      <light name='sun'>
        <pose>0 0 10 0 -0 0</pose>
      </light>
    </state>
    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose frame=''>-1.29545 -2.55987 1.20273 0 0.241796 -2.66221</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>
    <audio>
      <device>default</device>
    </audio>
    <model name='neobotix_ground_plane'>
      <static>1</static>
    </model>
  </world>
</sdf>
```

- After modifying the ```neo_workshop.world``` file, we can see that there's a line in the
    above file called ```<model name='mfi_floor_trial1'>``` which refers to the model we saved
    in the "Saving the floorplan" section (we saved the model as mfi_floor_trial1 folder). We
    simply need to copy this folder to the
    [models](https://github.com/neobotix/neo_simulation2/tree/rolling/models)
    folder in the neo_simulation2 workspace.
- Run the neobotix simulation as shown in the next section

# Running the Neobotix simulation

The neo_simulation2 package uses ros2 Humble and will not run on the foxy docker which the
team will be using for most of the development. Therefore, a separate docker container was
setup to run this simulation natively.

The instructions to run this are present in [this repository](https://github.com/sushanthj/robot-setup-tool)

