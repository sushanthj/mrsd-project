---
layout: default
title: Building Subscribers and Publishers
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

# Before You Begin

## Bare Minimum
Any subscriber we build can be run from any directory if it's a python file as shown below

```bash
python3 /home/test_subscriber.py
```

## Building in a ROS Package Setup

### Create Workspace and Clone an Existing Package

It's good practice to develop all ros nodes, even the simplest ones within a package. To do \
so, we'll need to follow the following steps:

1. Create a workspace in the following manner: ```mkdir /home/ddg/dev_ws/src```
2. The above step will create a basic folder structure we'll need to add new packages
3. If you're cloning an existing repository do the following (example using realsense ros2 wrappers)
   [Reference](https://github.com/IntelRealSense/realsense-ros)
   - ```cd /home/ddg/dev_ws/src```
   - ```git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-development```
   - ```cd ~/ddg/dev_ws```
   - ```bash
      sudo apt-get install python3-rosdep -y
      sudo rosdep init # "sudo rosdep init --include-eol-distros" for Eloquent and earlier
      rosdep update # "sudo rosdep update --include-eol-distros" for Eloquent and earlier
      rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys=librealsense2 -y
      ```
   - ```colcon build```
   - ```source /opt/ros/$ROS_DISTRO/setup.bash```
   - ```cd ~/ddg/dev_ws```
   - ```. install/local_setup.bash```
4. Note that in the above step, colcon build setup the install folder since a package was created \
5. If we don't have any packages setup using colcon build, you'll only have dev_ws/src which you created

### Now, let's create our own package!

A typical folder structure for a ros2 package will look like

```bash
my_package/
      setup.py
      package.xml
      resource/my_package
```

However, since we already have a workspace it should look more like 

```bash
workspace_folder/
    src/
      package_1/
          CMakeLists.txt
          package.xml

      package_2/
          setup.py
          package.xml
          resource/package_2
      ...
      package_n/
          CMakeLists.txt
          package.xml
```

#### Creating the Package

1. ```cd ~/ddg/dev_ws/src```
2. ```ros2 pkg create --build-type ament_python <package_name>```
3. ```cd ~/ddg/dev_ws/```
4. ```colcon build --packages-select my_package``` or to build all packages in workspace```colcon build```
5. ```. install/setup.bash``` (uses the install directory created by colcon build)

To run any node we develop in the package we can do

```bash
ros2 run my_package my_node
```

# Simple Example Subscriber from ROS

[Reference](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html#id1)

```python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

# Subscriber for RealSense Images

```python
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge


class ImageDumperSubscriber(Node):

    def __init__(self, image_):
        super().__init__('minimal_subscriber')

        self._bridge = CvBridge()

        self._color_subscription = self.create_subscription(
            Image, #msg_type 
            '/camera/color/image_raw', #topic
            self._color_listener_callback,  # callback
            10)

        self.depth_subscription = self.create_subscription(
            Image, #msg_type 
            '/camera/depth/image_rect_raw', #topic
            self._depth_listener_callback,  # callback
            10)
        

    def _color_listener_callback(self, msg):
        image_cv2 = self._bridge.imgmsg_to_cv2(msg, "bgr8") 
        self.counter += 1
        self.get_logger().info(f'I heard: {msg.header.frame_id}')
        
        
    def _depth_listener_callback(self, msg):
        image_cv2 = self._bridge.imgmsg_to_cv2(msg, "passthrough")
        self.get_logger().info(f'I heard: {msg.header.frame_id}')


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = ImageDumperSubscriber()
    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```