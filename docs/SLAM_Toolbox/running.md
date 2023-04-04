---
layout: page
title: Running SLAM Toolbox
parent: Understanding SLAM Toolbox
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

# Debugging

## Map saver timeout

This happens when the map is large and needs more time to save. Default is 5s but to override
we can run the following command:

```bash
ros2 run nav2_map_server map_saver_cli -f full_map_one --ros-args -p save_map_timeout:=10000
```

Note. **Remember you need to save a serialized map as well to use for localization later!**

```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "filename: 'full_map_serial'"
```