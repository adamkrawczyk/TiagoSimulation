# Copyright (c) 2023 PAL Robotics S.L. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import SetRemap


def loc_and_nav(context, *args, **kwargs):
    map_yaml_file = LaunchConfiguration("map")

    nav2_bringup_pkg = os.path.join(
        get_package_share_directory("nav2_bringup"), "launch"
    )

    configured_params = os.path.join(
        get_package_share_directory("tiago_navigation"),
        "config", "tiago_nav2.yaml",
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [nav2_bringup_pkg, "/localization_launch.py"]
        ),
        launch_arguments={
            "params_file": configured_params,
            "map": map_yaml_file,
        }.items(),
        condition=UnlessCondition(LaunchConfiguration("slam")),
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_bringup_pkg, "/slam_launch.py"]),
        launch_arguments={"params_file": configured_params}.items(),
        condition=IfCondition(LaunchConfiguration("slam")),
    )

    cmd_vel_remap = SetRemap(src="base_footprint/cmd_vel", dst="nav_vel")

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [nav2_bringup_pkg, "/navigation_launch.py"]
        ),
        launch_arguments={"params_file": configured_params}.items(),
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_bringup_pkg, "/rviz_launch.py"]),
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return [
        localization_launch,
        slam_toolbox_launch,
        cmd_vel_remap,
        nav2_bringup_launch,
        rviz_launch,
    ]


def generate_launch_description():

    loc_and_nav_launch = OpaqueFunction(function=loc_and_nav)

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(loc_and_nav_launch)

    return ld
