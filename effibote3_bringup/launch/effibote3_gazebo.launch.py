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
)

from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import ExecutableInPackage
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):

    mode = LaunchConfiguration("mode").perform(context)
    robot_urdf_description = LaunchConfiguration("robot_urdf_description").perform(context)

    robot = []

    if mode == "simulation":
        mode += "_gazebo_classic"

    if mode == "simulation_gazebo_classic":

        world = (
            get_package_share_directory("romea_simulation_gazebo_worlds")
            + "/worlds/friction_cone.world"
        )

        robot.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    get_package_share_directory("gazebo_ros") + "/launch/gzserver.launch.py"
                ),
                launch_arguments={"world": world, "verbose": "false"}.items(),
            )
        )

        robot.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    get_package_share_directory("gazebo_ros") + "/launch/gzclient.launch.py"
                )
            )
        )

        robot_description_file = "/tmp/effibote3_description.urdf"
        with open(robot_description_file, "w") as f:
            f.write(robot_urdf_description)

        robot.append(
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                exec_name="gazebo_spawn_entity",
                arguments=["-file", robot_description_file, "-entity", "effibote3"],
                output={"stdout": "log", "stderr": "log"},
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
            OpaqueFunction(function=launch_setup),
        ]
    )
