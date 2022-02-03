from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])]
        ),
        launch_arguments={"verbose": "false"}.items(),
    )

    urdf_file = get_package_share_directory("effibote3_description") + "/urdf/effibote3.urdf.xacro"
    controller_conf_yaml_file = get_package_share_directory("effibote3_bringup") + "/config/diff_drive_controller.yaml"

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            urdf_file,
#            PathJoinSubstitution(
#                [
#                    FindPackageShare("effibote3_description"),
#                    "urdf",
#                    "effibote3.urdf.xacro",
#                ]
#            ),
            " use_sim:=true ",
            "controller_conf_yaml_file:=",
            controller_conf_yaml_file,

        ]
    )
    robot_description = {"robot_description": robot_description_content}

    diff_drive_controller = PathJoinSubstitution(
        [
            FindPackageShare("effibote3_bringup"),
            "config",
            "diff_drive_controller.yaml",
        ]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "effibote3"],
        output="screen",
#        arguments=['--ros-args', '--log-level', 'debug'],
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    diff_drive_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["diff_drive_controller"],
        output="screen",
    )

    return LaunchDescription(
        [
            gazebo,
            robot_state_publisher,
            spawn_entity,
            joint_state_broadcaster,
            diff_drive_controller
#            controller_manager,
        ]
    )


    #    controller_manager = Node(
    #        package="controller_manager",
    #        executable="ros2_control_node",

    #        parameters=[robot_description],
    ##        parameters=[robot_description, diff_drive_controller],
    #        output={
    #            "stdout": "screen",
    #            "stderr": "screen",
    #        },
    ##        arguments=['--ros-args', '--log-level', 'debug']
    #    )
