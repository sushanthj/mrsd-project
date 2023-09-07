---
layout: page
title: Frame Design
parent: Mechanical Development
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


# 8020 Frame

We are using an Autonomous Mobile Robot (AMR) to transport LEGO bricks as part of MFI’s Lego
Testbed. We will be procuring the [Neobotix MP400 AMR](https://www.neobotix-robots.com/products/mobile-robots/mobile-robot-mp-400).
After the sensors and motors lab, I began designing
a modular frame which can house various sensors, power supply systems and on-board compute.

## Frame Design

![](/images/Mechanical/Frame/1_mod.png)

The Frame is designed to be modular, i.e. it can be mounted on any mobile platform for testing. Addi-
tionally, the frame is also designed to allow for minimal tolerance stack-up on the sensor mountings.

- Flat plate joints for the bottom links ensure perpendicularity between columns and beams.
- Base-sheet mounting holes are precision machined. Additionally the laser cut base-sheets imposes
  strict constraints on position of sensors and consequently the accuracy of the calculated transforma-
  tion matrices would improve.
- matching mounting points on top and bottom base-sheets ensures frame links are joined accurately.
- floating corner connectors allow fixing the level-2 sheet at a flexible height.

The [CAD Files](https://drive.google.com/file/d/1p7STYoNwQ2TOB133yYeL374bRsbvibkU/view?usp=sharing)
are available on drive for reference.

### Design Tools

The [8020 Dynamic Library](https://8020.net/downloads/index/designfiles) is useful to quickly
prototype on building the frame using the 8020 library.

## Base Sheet

![](/images/Mechanical/Frame/3_mod.png)

- The base sheet  mounting holes will be laser-cut and additionally the origin of the base sheet
  was kept aligned to the robot’s origin throughout the design stage.
- Mounts for the RealSense camera were designed to be 3D printed
- Four camera mounting positions were provided to allow for future additions.
- LIDAR was placed acoording to the figure below (to ensure the rays don't get blocked by the 8020 frame in front)

![](/images/Mechanical/Frame/frame_cross_section.png)

## RealSense Mount

![](/images/Mechanical/Frame/4_mod.png)

## 3D Models for Mounts

![Please Use This Link to Download](https://drive.google.com/drive/folders/1k8uoDXNNj3c8tyo9GgG58MCTG6aO6WGX?usp=sharing)

# Final Machining and Results

The mounting holes on the 8020 frame were precision CNC cut using a Haas 5 axis CNC. At time
of writing, this is the only 5 axis CNC available at CMU. (We don't need 5 axis, but this was
a super fancy machine!)

![](/images/Mechanical/Frame/machining.gif)

## Finished Product

![](/images/Mechanical/Frame/8020_frame.jpg)
