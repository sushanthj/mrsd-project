---
layout: page
title: Neobotix-Orin Networking
parent: Robot Bringup
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

# Orin - AMR - LIDAR Networking

Created by: Sushanth Jayanth
Created time: September 25, 2023 11:05 AM

# Starting Issue

To begin, the setup looks like the following:

- Orin and LIDAR is connected over Ethernet (utilizing the Ethernet port)
- Orin has internal wifi card to connect to WiFi network
- The AMR’s compute is connected to WiFi to begin, but has Ethernet ports and USB ports avialable

Hence, because the Ethernet port of the Orin is occupied with the LIDAR, we need to utilize ‘USB Networking’ to enable the Orin to share data over usb.

# Current Fix

The ‘USB Networking’ has to be setup to share data as shown below. I specifically used an Ethernet cable to connect the Ethernet port on the AMR to the type-C USB port on the rearside (opposite side as HDMI/DP port) of the Orin. **This required the usage of a Ethernet-to-Type-C adapter (passive device)**

## On the Orin (Host PC)

1. Open your Network Manager via the Network Icon on the Unity Panel:
   ![](/images/robot_bringup/networking/image.png)

2. Go all the way down to the **Edit Connections** option and click on it.
   ![](/images/robot_bringup/networking/image%20(1).png)

3. You will appear on the Wired Tab. Each Wired connection is by default related to each Ethernet Wired NIC card you have. For example if you have 2 Wired NICs you will see 2 options here. In the image below you see one since this is a Laptop and they normally have one. Select the Wired connection you wish to edit and double click on it or select the EDIT button.
   ![](/images/robot_bringup/networking/image%20(2).png)

4. You will appear on the Wired Tab (Again). This time, go to the IPv4 Settings if you are using IPv4 or IPv6 if you are using that one. On the **Method** option select **Share to Other Computers**. Now SAVE. You are done.
   ![](/images/robot_bringup/networking/image%20(3).png)

## On AMR PC (Client PC)

This computer can be accessed by connecting the AMR’s Display Port to a monitor and setting the wired connection settings to what is shown below:

![](/images/robot_bringup/networking/image(4).png)

The Additional DNS servers may need to be changed (to the Orin’s IP), but setting it to the default shown in the image seems to work fine right now.

# Benefits/Drawbacks of Above Networking

1. Because of no intermediate switch (only a passive adapter to convert Ethernet to type-C USB), low latency on comms
2. All ROS2 topics generated on the AMR are available on the Orin
3. Connecting the Orin to the Internet allows the AMR PC to also have network access (although via the Orin)