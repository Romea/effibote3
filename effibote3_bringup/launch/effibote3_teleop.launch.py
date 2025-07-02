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

from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetParameter

from ament_index_python.packages import get_package_share_directory
from effibote3_description import get_specifications_path_file


def launch_setup(context, *args, **kwargs):

    mode = LaunchConfiguration("mode").perform(context)
    joystick_topic = LaunchConfiguration("joystick_topic").perform(context)

    mobile_base_configuration_file_path = get_specifications_path_file()

    joystick_configuration_file_path = LaunchConfiguration(
        "joystick_configuration_file_path"
    ).perform(context)

    teleop_configuration_file_path = LaunchConfiguration(
        "teleop_configuration_file_path"
    ).perform(context)

    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("romea_mobile_base_teleop") + "/launch/teleop.launch.py"
        ),
        launch_arguments={
            "mobile_base_configuration_file_path": mobile_base_configuration_file_path,
            "joystick_configuration_file_path": joystick_configuration_file_path,
            "teleop_configuration_file_path": teleop_configuration_file_path,
            "joystick_topic": joystick_topic,
        }.items(),
    )

    return [
        GroupAction(
            actions=[
                SetParameter(name="use_sim_time", value=(mode != "live")),
                teleop,
            ]
        )
    ]


def generate_launch_description():

    default_teleop_configuration_file_path = (
        get_package_share_directory("effibote3_description") + "/config/teleop.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("mode"),
            DeclareLaunchArgument("joystick_configuration_file_path"),
            DeclareLaunchArgument("joystick_topic"),
            DeclareLaunchArgument(
                "teleop_configuration_file_path",
                default_value=default_teleop_configuration_file_path
            ),
            OpaqueFunction(function=launch_setup)
        ]
    )
