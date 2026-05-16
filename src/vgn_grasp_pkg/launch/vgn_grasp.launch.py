"""
vgn_grasp.launch.py

Launch vgn_grasp_node.

Prerequisites:
  1. git submodule add https://github.com/ethz-asl/vgn external/vgn
  2. source launch_env.bash  (sets GSAM_WS, activates gsam_ws_venv)
  3. pip install pytorch-ignite  (inside venv)
  4. Place VGN weights at: $GSAM_WS/models/vgn_conv.pth

Usage:
  ros2 launch vgn_grasp_pkg vgn_grasp.launch.py

Override example:
  ros2 launch vgn_grasp_pkg vgn_grasp.launch.py \
    vgn_model_path:=/abs/path/to/model.pt \
    min_quality:=0.4 \
    max_candidates:=3
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    args = [
        DeclareLaunchArgument('roi_size_m',             default_value='0.30'),
        DeclareLaunchArgument('tsdf_resolution',        default_value='40'),
        DeclareLaunchArgument('vgn_model_path',         default_value='models/vgn_conv.pth'),
        DeclareLaunchArgument('min_quality',            default_value='0.5'),
        DeclareLaunchArgument('max_candidates',         default_value='5'),
        DeclareLaunchArgument('min_point_count',        default_value='50'),
        DeclareLaunchArgument('semantic_filter_radius', default_value='0.05'),
        DeclareLaunchArgument('world_map_topic',        default_value='/world_map'),
        DeclareLaunchArgument('world_map_result_topic', default_value='/world_map_result'),
        DeclareLaunchArgument('grasp_candidates_topic', default_value='/grasp_candidates'),
        DeclareLaunchArgument('world_frame',            default_value='world'),
        DeclareLaunchArgument('robot_frame',            default_value='panda_link0'),
    ]

    node = Node(
        package    = 'vgn_grasp_pkg',
        executable = 'vgn_grasp_node',
        name       = 'vgn_grasp_node',
        output     = 'screen',
        parameters = [{
            'roi_size_m':             LaunchConfiguration('roi_size_m'),
            'tsdf_resolution':        LaunchConfiguration('tsdf_resolution'),
            'vgn_model_path':         LaunchConfiguration('vgn_model_path'),
            'min_quality':            LaunchConfiguration('min_quality'),
            'max_candidates':         LaunchConfiguration('max_candidates'),
            'min_point_count':        LaunchConfiguration('min_point_count'),
            'semantic_filter_radius': LaunchConfiguration('semantic_filter_radius'),
            'world_map_topic':        LaunchConfiguration('world_map_topic'),
            'world_map_result_topic': LaunchConfiguration('world_map_result_topic'),
            'grasp_candidates_topic': LaunchConfiguration('grasp_candidates_topic'),
            'world_frame':            LaunchConfiguration('world_frame'),
            'robot_frame':            LaunchConfiguration('robot_frame'),
        }],
    )

    return LaunchDescription(args + [node])
