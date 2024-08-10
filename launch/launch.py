from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_spaceros_gz_sim = get_package_share_directory('spaceros_gz_sim')
    gz_launch_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
    moon_world_path = PathJoinSubstitution([pkg_spaceros_gz_sim, 'worlds', 'moon.sdf'])

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', PathJoinSubstitution([pkg_spaceros_gz_sim, 'models'])),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={'gz_args': [moon_world_path], 'on_exit_shutdown': 'True'}.items()),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[],
            remappings=[],
            output='screen'
        ),
    ])
