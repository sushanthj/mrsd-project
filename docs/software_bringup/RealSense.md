---
layout: default
title: RealSense Setup
parent: Systems Setup
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

# RealSense Drivers for Linux

## Add Server's public keys and to list of repositories
1. ```sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE```
2. ```sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE```
3. ```sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u```

## Install Libraries
1. ```sudo apt-get install librealsense2-dkms```
2. ```sudo apt-get install librealsense2-utils```

## Verify Installation

```modinfo uvcvideo | grep "version:"``` should include ```realsense``` string

## Camera Trial

After above steps, connect camera and try: ```realsense-viewer```

# RealSense Drivers for Jetson

[Reference](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation_jetson.md)


# ROS2 Wrappers

[Reference](https://github.com/IntelRealSense/realsense-ros)

# SLAM with ROS2

Realsense has some Turtlebot3 robot images for example which can build a map

[Reference](https://intel.github.io/robot_devkit_doc/pages/rs_slam.html)