---
layout: default
title: ROS2 on the Jetson and Desktop
parent: Systems Setup
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

# Xavier AGX Setup

Setting up the AGX involves the following steps:
1. The AGX has an inbuilt eMMC which can hold the OS (the SD card probably can store data)
2. The Nvidia SDK manager needs to be installed
3. AGX can be powered (even through a min 20W charger and through the usb type-c port which is
   adjacent to the HDMI port)
4. The other type-c port can be used to connect it to the keyboard or usb-hub

## Links for above steps

- [SDK Manager Download](https://developer.nvidia.com/drive/sdk-manager)
- [SDK Manager Usage](https://docs.nvidia.com/sdk-manager/install-with-sdkm-jetson/index.html)
- [Initial Kit Setup](https://www.youtube.com/watch?v=-nX8eD7FusQ&ab_channel=NVIDIADeveloper)

# ROS Setup via Dockerfile on Desktop

I watched a few videos online and managed to write a Dockerfile to build ros. 

1. [ROS2 on Docker from scratch](https://devanshdhrafani.github.io/blog/2021/04/15/dockerros2.html)
2. [ROS2 on Docker using osrf image](https://www.youtube.com/watch?v=EU-QaO6xTv4&t=941s&ab_channel=xLABforSafeAutonomousSystems)

A dockerfile containing a hodge-podge of both is shown below:

```dockerfile
FROM osrf/ros:foxy-desktop
MAINTAINER Sush sush@cmu.edu

# Necessary to source things
SHELL ["/bin/bash", "-c"]

RUN apt-get update --fix-missing && \
    apt-get install -y \
    git \
    nano \
    python3-pip \
    tmux \
    python3-matplotlib \
    python3-ipdb \
    unzip \
    wget \
    zip

RUN pip3 install numpy
RUN pip3 install wandb

# create a ws for tutorials or trial scripts
# RUN mkdir /home/dev_ws 
# RUN cd /home/dev_ws/ && git clone https://github.com/ros/ros_tutorials.git -b foxy-devel

# copy all contents of current dir (mfi-amr repo files) into docker
RUN mkdir /home/mfi-amr
COPY . /home/mfi-amr

# cleanup
RUN apt-get -qy autoremove

#ADD .bashrc /root/.bashrc
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc

# create a beginner workspace for now
WORKDIR /home/dev_ws/src
RUN git clone https://github.com/ros/ros_tutorials.git -b foxy-devel
WORKDIR /home/dev_ws

# after workspace is defined, run the 
RUN apt-get install python3-rosdep -y
RUN rosdep update
RUN rosdep install -i --from-path src --rosdistro foxy -y
RUN apt install python3-colcon-common-extensions -y

# source setup.bash in WORKDIR
WORKDIR '/home/mfi-amr'
RUN source /opt/ros/foxy/setup.bash

ENTRYPOINT ["/bin/bash"]
RUN source /opt/ros/foxy/setup.bash
```

## Why use Docker?

A Docker image is like a class and a Docker Container is an object (or instantiation) of the image. A Docker Image contains everything needed to run a container: code, libraries, environment variables, configuration files, etc. It serves as a blueprint which can be used to create an instance, ie, a Docker Container. Once a Docker Container is created, you can tinker with it as much as you like, and it won’t affect the image from which it was built.

![](/images/ros_setup/docker_th.png)

You can find prebuilt Docker Images for many different applications on the DockerHub1, which uses a GitHub like cloud solution where you can pull images to your local computer. These prebuilt images have relevant libraries, environment variables, etc. already setup so you can simply create a Container from the Image and get started on your work.

If you can’t find a suitable image for your use case on DockerHub, you can create your own Docker Image using a Dockerfile. A Dockerfile is a set of instructions to build a Docker Image. You can learn more about the syntax and standard practices of writing a Dockerfile from the documentation2. For the purposes of this guide, I will explain the commands that I used as we go.

eg. OSRF (Open Source Robotics Foundation's Docker Image for ROS2 foxy)

```dockerfile

# This is an auto generated Dockerfile for ros:desktop
# generated from docker_images_ros2/create_ros_image.Dockerfile.em
FROM ros:foxy-ros-base-focal

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-foxy-desktop=0.9.2-1* \
    && rm -rf /var/lib/apt/lists/*
```

## Steps to setup ROS

1. Clone the Repository [mfi-amr](https://github.com/sushanthj/mfi-amr)
2. Enter the cloned repo and run the following command \
   ```docker build -t trial_ros_image .``` \
   This command creates a docker image with a tag as trial-ros-image. The docker \
   images present on a system can be found via a simple ```docker images``` command on bash
3. Once the above has completed, we'll have to create a docker container from the above image. \
   One can do this manually, but I created a bash script to make it simple
   ```bash
   #!/bin/bash
    cd ~/mfi/mfi-amr/
    docker run \
        -it \
        --gpus all \
        --rm \
        --name mfi \
        --shm-size=8g \
        --network host \
        -e DISPLAY=$DISPLAY \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v /var/run/docker.sock:/var/run/docker.sock \
        -v /home/sush/mfi/mfi-amr_docker_save/:/home/workspace/ \
        -v /home/sush/mfi/mfi-amr/:/home/mfi-amr/ \
        trial_ros_image:latest
    ```
4. Now, the last line of that script requests docker to build the latest image with the
   tag ```trial_ros_image```
