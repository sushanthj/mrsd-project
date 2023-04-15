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