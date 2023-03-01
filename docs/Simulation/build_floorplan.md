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

- MODEL_NAME (given by user)
  - model.config
  - model.sdf

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