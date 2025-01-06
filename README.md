ROS2 control hardware package for the padman project

This is heavily under construction and very much in a prototyping stage. The code in here does not represent good practice in any means. I am currently jotting down a few things very quickly and will refactor soon.

Install:

```
colcon build --symlink-install --packages-select padman_hw
```


Test URDF/xacro:

```
ros2 launch padman_hw view_robot.launch.py
```
