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

# RealSense Drivers for Computer (x86 and Ubuntu 20.04)

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

# RealSense Drivers for Xavier AGX

```bash
sudo apt-get update && sudo apt-get -y upgrade
sudo apt-get install -y --no-install-recommends \
    python3 \
    python3-setuptools \
    python3-pip \
    python3-dev

sudo apt-get install -y git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev

sudo apt-get install -y libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev

sudo apt-get install libusb-1.0-0-dev

# This is sort of the most important step
sudo apt-get install python3.9-dev

git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense/
mkdir build && cd build
cmake ../ -DFORCE_RSUSB_BACKEND=false -DBUILD_PYTHON_BINDINGS=true -DCMAKE_BUILD_TYPE=release -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=true
sudo make uninstall && make clean && make && sudo make install
```

# Setup RealSense wrappers to publish RealSense Images

[Reference](https://github.com/IntelRealSense/realsense-ros)

# View RealSense on rqt

1. open rqt by typing ```rqt``` in the terminal
2. Navigate to Plugins -> Visualization -> Image View
3. You should then see something like this \
   ![](/images/ros_setup/rqt.png)