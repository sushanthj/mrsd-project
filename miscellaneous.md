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


# Velodyne

- To run the velodyne driver first connect the VLP ethernet to your computer
- Then open 192.168.1.201 on a browser. Scroll down and note the IP address the VLP is set
    - usually it's set to 192.168.1.71
- Go to your **wired connection settings** and add set it as shown below
- ![](/images/miscellaneous/vlp_ifconfig.png)

Now, restart your velodyne driver.