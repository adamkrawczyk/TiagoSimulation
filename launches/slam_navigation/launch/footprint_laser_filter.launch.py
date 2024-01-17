import pathlib

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            arguments=[
                '-d', str(pathlib.Path(__file__).parent.absolute().joinpath('config', 'footprint_filter_example.yaml'))
            ],
            # remappings=[("scan", "base_footprint/scan")]
        )
    ])