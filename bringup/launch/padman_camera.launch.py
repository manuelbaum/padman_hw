import os
from launch import LaunchDescription
from launch_ros.actions import Node


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            #namespace='turtlesim1',
            executable='static_transform_publisher',
            name='padman_camera_tf_publisher',
            arguments=["0.075", "0.0", "0.055", "1.57", "0.0", "0.0", "base_link", "camera_link"]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('realsense2_camera'), 'launch'),
         '/rs_launch.py']),
         launch_arguments={'pointcloud.enable': 'true', 'enable_gyro':'true', 'enable_accel':'true'}.items(),
      )
    ])