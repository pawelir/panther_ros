#!/usr/bin/env python3

# Copyright 2024 Husarion sp. z o.o.
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        description="Add namespace to all launched nodes",
    )

    system_monitor_config_path = LaunchConfiguration("system_monitor_config_path")
    declare_system_monitor_config_path_arg = DeclareLaunchArgument(
        "system_monitor_config_path",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("panther_diagnostics"),
                "config",
                "system_monitor.yaml",
            ]
        ),
        description="Specify the path to the system monitor configuration file.",
    )

    system_monitor_node = Node(
        package="panther_diagnostics",
        executable="system_monitor_node",
        name="system_monitor",
        parameters=[system_monitor_config_path],
        namespace=namespace,
        remappings=[("/diagnostics", "diagnostics")],
        emulate_tty=True,
    )

    actions = [
        declare_namespace_arg,
        declare_system_monitor_config_path_arg,
        system_monitor_node,
    ]

    return LaunchDescription(actions)
