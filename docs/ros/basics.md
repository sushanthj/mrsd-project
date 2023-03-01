---
layout: default
title: Basics
parent: ROS Learnings
nav_order: 3
---

<details open markdown="block">
  <summary>
    Table of contents
  </summary>
  {: .text-delta }
1. TOC
{:toc}
</details>

# Basics of Setting up with ROS2

## Changing Between ROS versions (ROS1 and ROS2)

Ensure the environment variables are set correctly:

```printenv | grep -i ROS```

Change any environment variables to the right path if required.

Note. The variable ```ROS_LOCALHOST_ONLY``` should be set to ```ROS_LOCALHOST_ONLY=1``` for Foxy
and ```ROS_LOCALHOST_ONLY=0``` for Noetic (I think)


## Create a Workspace and Package

- Create WS
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws
    ```


- Now, go into the src directory to clone some example packages
    ```bash
    cd src
    git clone https://github.com/ros2/examples src/examples -b foxy
    ```

- Building the workspace (and packages inside)
    ```bash
    colcon build --symlink-install
    ```

    The --symlink-install allows us to make changes to backend by updated non-compiled files
    like python files

- In the root directory of our workspace (i.e. inside ros2_ws) source the environment 
  (this will also source the workspace internally)
    ```bash
    . install/setup.bash
    ```

- Add sources to .bashrc
    ```bash
    echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
    echo "export _colcon_cd_root=/opt/ros/foxy/" >> ~/.bashrc
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
    ```

## Adding more packages

This is simple once we have a workspace setup. To add another new package into existing workspace
just do:

1. Go to your ```ros2_ws/src``` folder
2. Add any new repo (ros package) to this src folder 
    ```bash
    git clone https://github.com/ros/ros_tutorials.git -b foxy-devel
    ```
3. Resolve Dependencies:
    - ```cd ..```
    - ```rosdep install -i --from-path src --rosdistro foxy -y```

4. Build the workspace again (remember you should NOT be in ```src``` folder when doing this)

## Creating your own package

Previously, we added existing packages to our /src/ folder. Now, let's build a package from scratch.

We can do so in two ways:
1. Copy paste an existing package and then make changes to the package.xml and CMakeLists.txt
    (if you choose to use Cmake)
2. Copy paste an existing package and then make changes to the package.xml, setup.py, setup.cfg
    and another folder with same name as package (with an ```__init__``` file) (if you choose to use Python)
3. Use ```ros2 pkg create``` to make our lives easier

Let's use step 3 and with Python for simplicity:

- ```cd ~/ros2_ws/src```
- ```ros2 pkg create --build-type ament_python <package_name>```
- ```cd ~/ros2_ws```
- ```colcon build``` or ```colcon build --packages-select my_package```
- Source again ```. install/setup.bash```

Running the node is then just ```ros2 run my_package my_node```

Note: You could also configure the **package.xml and setup.py present inside each node**. But
everything will work without this also



# Learning about TF2

