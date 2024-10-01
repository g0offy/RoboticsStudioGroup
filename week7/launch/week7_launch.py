import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Define the path to your world file
    world_file_name = 'libraryV2.world'
    world_path = os.path.join(get_package_share_directory('~/git/robotics_studio_classwork/world_maps'), 'worlds', world_file_name)

    return LaunchDescription([
        # Launch Gazebo with the specified world file
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_path],
            output='screen'),
    ])
