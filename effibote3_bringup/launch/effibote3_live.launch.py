from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():


    info_file = get_package_share_directory("effibote3_description") + "/config/effibote3.yaml"
    urdf_file = get_package_share_directory("effibote3_description") + "/urdf/effibote3.urdf.xacro"
    controller_manager_yaml_file = get_package_share_directory("effibote3_bringup") + "/config/controller_manager.yaml"
    diff_drive_controller_yaml_file = get_package_share_directory("effibote3_bringup") + "/config/diff_drive_controller.yaml"
    mobile_base_controller_yaml_file = get_package_share_directory("effibote3_bringup") + "/config/mobile_base_controller.yaml"

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
            " mode:=live",
            " controller_conf_yaml_file:=",
            controller_manager_yaml_file,
        ]
    )

    robot_description = {"robot_description": robot_description_content}


    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )


    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description,],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
 #        arguments=['--ros-args', '--log-level', 'debug']
    )

    diff_drive_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller","--param-file",diff_drive_controller_yaml_file],
        output="screen",
    )

    mobile_base_controller = Node(
        package="controller_manager",
        executable="spawner",
        name="mobile_base_controller_spawner",
        arguments=["mobile_base_controller","--param-file",mobile_base_controller_yaml_file],
        output="screen",
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            joint_state_broadcaster,
            controller_manager,
#            diff_drive_controller,
#            mobile_base_controller,
#            ,
        ]
    )
