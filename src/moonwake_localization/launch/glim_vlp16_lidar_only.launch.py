import os
import tempfile
import uuid

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_directory = PathJoinSubstitution([
        FindPackageShare('moonwake_localization'),
        'config', 'glim', 'vlp16_lidar_only',
    ])
    default_dump_path = os.path.join(
        tempfile.gettempdir(), 'moonwake_glim_vlp16_' + uuid.uuid4().hex)

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_path', default_value=config_directory,
            description='GLIM configuration directory containing config.json.'),
        DeclareLaunchArgument(
            'dump_path', default_value=default_dump_path,
            description='Unique output directory used when GLIM exits.'),
        DeclareLaunchArgument(
            'start_preflight', default_value='true',
            description='Start the point cloud diagnostics node.'),
        Node(
            package='glim_ros', executable='glim_rosnode', output='screen',
            parameters=[{
                'config_path': LaunchConfiguration('config_path'),
                'dump_path': LaunchConfiguration('dump_path'),
            }]),
        Node(
            package='moonwake_localization', executable='pointcloud_preflight_node',
            output='screen', condition=IfCondition(LaunchConfiguration('start_preflight'))),
    ])
