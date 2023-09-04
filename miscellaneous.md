---
layout: page
title: Miscellaneous yet Useful
permalink: miscellaneous
nav_order: 15
---

# ROS2

## Time Handling

ROS2 removed the ```ros.time.now``` option which we used to use previously. Now, to get
access of the time, we do the following:

```python
t.header.stamp = self.get_clock().now().to_msg()
```

This was taken from [this example](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Py.html)

```python
def handle_turtle_pose(self, msg):
    t = TransformStamped()

    # Read message content and assign it to
    # corresponding tf variables
    t.header.stamp = self.get_clock().now().to_msg()
    t.header.frame_id = 'world'
    t.child_frame_id = self.turtlename
```

## Installing RQT Graph

```bash
sudo apt install ros-humble-rqt-graph
```

## Get package directory in launch files

This is typically useful to get the path of packages in ROS workspaces

### Get share directory

```python
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='robot_localization',
            executable='ukf_node',
            name='ukf_filter_node',
            output='screen',
            parameters=[os.path.join(get_package_share_directory("robot_localization"), 'params', 'ukf.yaml')],
           ),
])
```

### Get package directory

![](/images/miscellaneous/get_pkg_directory.png)


## Installing Realsense Driver for Tracking camera and D435i

Clone the required packages

1. Build the package realsense2_camera_msgs without any modifications first
2. If there's a dependency issue, install rosdeps
    ```bash
    rosdep install --from-paths src --ignore-src -r -y
    source /opt/ros/foxy/setup.bash
    ```


# Velodyne

- To run the velodyne driver first connect the VLP ethernet to your computer
- Then open 192.168.1.201 on a browser. Scroll down and note the IP address the VLP is set
    - usually it's set to 192.168.1.71
- Go to your **wired connection settings** and add set it as shown below
- ![](/images/miscellaneous/vlp_ifconfig.png)

Now, restart your velodyne driver.


# Getting GUI acces on your docker

I follow this template for any docker file to allwo for GUI access

```bash
# Map host's display socket to docker
DOCKER_ARGS+=("-v /tmp/.X11-unix:/tmp/.X11-unix")
DOCKER_ARGS+=("-v $HOME/.Xauthority:/home/admin/.Xauthority:rw")
DOCKER_ARGS+=("-e DISPLAY")
DOCKER_ARGS+=("-e NVIDIA_VISIBLE_DEVICES=all")
DOCKER_ARGS+=("-e NVIDIA_DRIVER_CAPABILITIES=all")
DOCKER_ARGS+=("-e FASTRTPS_DEFAULT_PROFILES_FILE=/usr/local/share/middleware_profiles/rtps_udp_profile.xml")

# Run container from image
# print_info "Running humblesim"
docker run -it --rm \
    --privileged \
    --network host \
    ${DOCKER_ARGS[@]} \
    --runtime nvidia \
    ############# Only change this portion according to your needs ##############
    -v /home/sush/mfi/robot-setup-tool/world_files:/home/admin/worlds \
    -v /dev/*:/dev/* \
    --name "humble_sim_docker" \
    $@ \
    sushanthj/humble_sim_mapping_built:latest \
    #############################################################################
    /bin/bash
```

## Error Handling after above case

Now, after running the above command maybe within a shell script, you might encounter some
weird errors which are display related.

Eg. ```glfw error 65544: X11: Failed to open display :1.0
failed to initialize GLFW```

Now, all such errors can be fixed by giving the docker container access to the GUI by doing

```xhost +local:docker```

## Changing File Ownership

This occasionally is a problem when using docker which treats files as sudo. When moving files out of docker
The normal user will not be able to access files created inside the docker because of sudo permissions

Easy Fix 
- Find the username of the current ubuntu system you are using (eg. mine is always sush)
- ```sudo chown sush trial.txt```
- This will change the ownership back to the default user
