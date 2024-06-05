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
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.substitutions import FindPackageShare
from panther_utils.welcomeMsg import welcomeMsg


def generate_launch_description():
    disable_manager = LaunchConfiguration("disable_manager")
    declare_disable_manager_arg = DeclareLaunchArgument(
        "disable_manager",
        default_value="False",
        description="Enable or disable manager_bt_node.",
        choices=["True", "False"],
    )

    namespace = LaunchConfiguration("namespace")
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        description="Add namespace to all launched nodes.",
    )

    use_ekf = LaunchConfiguration("use_ekf")
    declare_use_ekf_arg = DeclareLaunchArgument(
        "use_ekf",
        default_value="True",
        description="Enable or disable EKF.",
        choices=["True", "False"],
    )

    serial_no = EnvironmentVariable(name="PANTHER_SERIAL_NO", default_value="----")
    panther_version = EnvironmentVariable(name="PANTHER_ROBOT_VERSION", default_value="1.0")
    welcome_msg = welcomeMsg(serial_no, panther_version)

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("panther_controller"), "launch", "controller.launch.py"]
            )
        ),
        launch_arguments={
            "namespace": namespace,
            "wheel_type": "WH05",
            "controller_config_path": PathJoinSubstitution(
                [
                    FindPackageShare("panther_controller"),
                    "config",
                    PythonExpression(["'", "WH05_controller.yaml'"]),
                ]
            ),
            "wheel_config_path": PathJoinSubstitution(
                [
                    FindPackageShare("panther_mini_description"),
                    "config",
                    PythonExpression(["'", "WH05.yaml'"]),
                ]
            ),
            "robot_description_path": PathJoinSubstitution(
                [
                    FindPackageShare("panther_mini_description"),
                    "urdf",
                    "panther_mini.urdf.xacro",
                ]
            ),
        }.items(),
    )

    system_status_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("panther_diagnostics"),
                    "launch",
                    "system_status.launch.py",
                ]
            ),
        ),
        launch_arguments={"namespace": namespace}.items(),
    )

    lights_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("panther_lights"), "launch", "lights.launch.py"]
            )
        ),
        launch_arguments={
            "namespace": namespace,
            "led_config_file": PathJoinSubstitution(
                [FindPackageShare("panther_lights"), "config", "mini_led_config.yaml"]
            ),
            "channel_1_num_led": "20",
            "channel_2_num_led": "28",
        }.items(),
    )

    battery_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("panther_battery"), "launch", "battery.launch.py"]
            ),
        ),
        launch_arguments={"namespace": namespace}.items(),
    )

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("panther_localization"), "launch", "ekf.launch.py"]
            )
        ),
        launch_arguments={"namespace": namespace}.items(),
        condition=IfCondition(use_ekf),
    )

    manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("panther_manager"), "launch", "manager_bt.launch.py"]
            )
        ),
        condition=UnlessCondition(disable_manager),
        launch_arguments={"namespace": namespace}.items(),
    )

    logo_animation_call = ExecuteProcess(
        cmd=[
            [
                FindExecutable(name="ros2"),
                " service call ",
                "/",
                namespace,
                "/lights/controller/set/animation  ",
                "panther_msgs/srv/SetLEDAnimation ",
                "'{animation: {id: 10}, repeating: true}'",
            ]
        ],
        shell=True,
    )

    delayed_action = TimerAction(
        period=10.0,
        actions=[
            battery_launch,
            lights_launch,
            manager_launch,
            ekf_launch,
            logo_animation_call,
        ],
    )

    actions = [
        declare_disable_manager_arg,
        declare_namespace_arg,
        declare_use_ekf_arg,
        welcome_msg,
        controller_launch,
        system_status_launch,
        delayed_action,
    ]

    return LaunchDescription(actions)
