import os
from os import environ, pathsep

from ament_index_python import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import yaml
from launch_ros.substitutions import FindPackageShare


def get_params_from_yaml(path):
    with open(path, 'r') as file:
        params = yaml.safe_load(file)
    print('YAML params loaded successfully!')

    return params

def generate_launch_description():

    # READ AND PASS CONFIG FROM YAML FILE
    tiago_finder_pkg = get_package_share_directory('tiago_navigation')
    global_conf = get_params_from_yaml(
        os.path.join(tiago_finder_pkg,
                     'config', 'global_config.yaml')
    )

    use_moveit = global_conf['global_config']['ros__parameters']['use_moveit']
    print(f'Use moveit: {use_moveit}')

    map_name = global_conf['global_config']['ros__parameters']['map_name']
    print(f'Map: {map_name}')

    use_rviz = global_conf['global_config']['ros__parameters']['use_rviz']
    print(f'Use rviz: {use_rviz}')

    use_slam = global_conf['global_config']['ros__parameters']['use_slam']
    print(f'Use slam: {use_slam}')

    # DECLARE LAUNCH ARGUMENTS

    moveit_arg = DeclareLaunchArgument(
        'moveit', default_value=use_moveit,
        description='Specify whether to launch MoveIt2'
    )

    declare_rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value=use_rviz,
        description="Open RViz2 along with the Navigation",
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(tiago_finder_pkg, "maps", map_name),
        description="Full path to map yaml file to load",
    )

    declare_slam_cmd = DeclareLaunchArgument(
        "slam",
        default_value=use_slam,
        description="Whether to start the SLAM or the Map Localization",
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(tiago_finder_pkg, 'rviz',
                                   'rviz_tiago_finder.rviz'),
        description='Full path to the RVIZ config file to use')

    # LAUNCH BRINGUP

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tiago_navigation'),
                'launch',
                'nav_bringup.launch.py'
            ])
        ])
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tiago_moveit_config'),
                'launch',
                'move_group.launch.py'
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('moveit'))
    )

    # GAZEBO RESOURCES
    ld = LaunchDescription()

    ld.add_action(declare_rviz_arg)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)

    ld.add_action(navigation)

    ld.add_action(moveit_arg)
    ld.add_action(moveit)

    return ld
