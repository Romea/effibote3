from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    romea_odo_description_package_prefix = get_package_share_directory('romea_odo_description')
    urdf_file = get_package_share_directory("effibote3_description") + "/urdf/effibote3.urdf.xacro"

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([romea_odo_description_package_prefix,'/launch/view_robot.launch.py']),
            launch_arguments = {'urdf_file': urdf_file}.items(),
        ),
    ])
