from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_spaceros_gz_demos = get_package_share_directory('spaceros_gz_demos')
    gz_launch_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
    gz_model_path = PathJoinSubstitution([pkg_spaceros_gz_demos, 'models'])

    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_model_path),
        DeclareLaunchArgument(
            'world_file',
            description='World file to load into Gazebo'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args': [PathJoinSubstitution([pkg_spaceros_gz_demos, 'worlds',
                                                  LaunchConfiguration('world_file')])],
                'on_exit_shutdown': 'True'
            }.items(),
        ),
    ])