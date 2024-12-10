ROS2 control hardware package for the padman project

Install:

```
colcon build --symlink-install --packages-select padman_hw
```


Test URDF/xacro:

```
ros2 launch padman_hw view_robot.launch.py
```