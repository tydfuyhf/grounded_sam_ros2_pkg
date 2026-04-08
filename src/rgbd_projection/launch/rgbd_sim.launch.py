"""
rgbd_sim.launch.py

  1. Gazebo Sim   — worlds/rgbd_world.sdf (ground + RGBD camera)
  2. ros_gz_bridge — Gazebo → ROS 2
       /rgbd_camera/image        sensor_msgs/Image
       /rgbd_camera/camera_info  sensor_msgs/CameraInfo
       /rgbd_camera/depth_image  sensor_msgs/Image  (float32, meters)
       /rgbd_camera/points       sensor_msgs/PointCloud2
  3. RViz2        — rviz/rgbd.rviz

Usage:
  ros2 launch rgbd_projection rgbd_sim.launch.py
  ros2 launch rgbd_projection rgbd_sim.launch.py rviz:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_this = get_package_share_directory('rgbd_projection')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_file = os.path.join(pkg_this, 'worlds', 'sensors_demo.sdf')
    rviz_config = os.path.join(pkg_this, 'rviz', 'rgbd.rviz')

    # ── Gazebo Sim ──────────────────────────────────────────────────────────
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}',  # -r: auto-run (play on start)
        }.items(),
    )

    # ── ROS-Gazebo Bridge ───────────────────────────────────────────────────
    # Format: <gz_topic>@<ros_type>@<gz_type>
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='rgbd_bridge',
        arguments=[
            # RGB image
            '/rgbd_camera/image'
            '@sensor_msgs/msg/Image'
            '@gz.msgs.Image',
            # CameraInfo (intrinsics — needed for projection)
            '/rgbd_camera/camera_info'
            '@sensor_msgs/msg/CameraInfo'
            '@gz.msgs.CameraInfo',
            # Depth image (float32, meters)
            '/rgbd_camera/depth_image'
            '@sensor_msgs/msg/Image'
            '@gz.msgs.Image',
            # PointCloud2 (XYZRGB — generated directly by Gazebo RGBD sensor)
            '/rgbd_camera/points'
            '@sensor_msgs/msg/PointCloud2'
            '@gz.msgs.PointCloudPacked',
        ],
        output='screen',
    )

    # ── RViz2 ───────────────────────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz2 for visualization',
        ),
        gz_sim,
        bridge,
        rviz,
    ])
