# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
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
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
    GroupAction,
)

from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import ExecutableInPackage
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):

    mode = LaunchConfiguration("mode").perform(context)
    robot_urdf_description = LaunchConfiguration("robot_urdf_description").perform(context)

    joystick_configuration_file_path = (
        get_package_share_directory("romea_joystick_utils")
        + "/config/" + LaunchConfiguration("joystick_model").perform(context) + ".yaml"
    )

    robot = []

    if mode == "simulation":

        robot.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    get_package_share_directory("effibote3_bringup")
                    + "/launch/effibote3_gazebo.launch.py"
                ),
                launch_arguments={
                    "mode": mode,
                    "robot_urdf_description": robot_urdf_description,
                }.items(),
            )
        )

    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("effibote3_bringup") + "/launch/effibote3_base.launch.py"
        ),
        launch_arguments={
            "mode": mode,
            "tf_prefix": "effibote3_",
            "base_name": "base",
        }.items(),
    )

    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("effibote3_bringup") + "/launch/effibote3_teleop.launch.py"
        ),
        launch_arguments={
            "mode": mode,
            "joystick_configuration_file_path": joystick_configuration_file_path,
            "joystick_topic": "/effibote3/joystick/joy",
        }.items(),
    )

    robot.append(
        GroupAction(
            actions=[
                PushRosNamespace("effibote3"),
                PushRosNamespace("base"),
                base,
                teleop,
            ]
        )
    )

    robot.append(
        GroupAction(
            actions=[
                PushRosNamespace("effibote3"),
                PushRosNamespace("joystick"),
                Node(package="joy", executable="joy_node"),
            ]
        )
    )

    return robot


def generate_launch_description():

    urdf_description = Command(
        [
            ExecutableInPackage("generate_urdf_description.py", "effibote3_bringup"),
            " robot_namespace:effibote3",
            " base_name:base",
            " mode:",
            LaunchConfiguration("mode"),
        ],
        on_stderr="ignore",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="simulation"),
            DeclareLaunchArgument("robot_urdf_description", default_value=urdf_description),
            DeclareLaunchArgument("joystick_model", default_value="microsoft_xbox"),
            OpaqueFunction(function=launch_setup),
        ]
    )
