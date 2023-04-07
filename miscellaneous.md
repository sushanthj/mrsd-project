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