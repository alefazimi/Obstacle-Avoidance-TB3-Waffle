from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
import os


def generate_launch_description():
    model = DeclareLaunchArgument('model', default_value='burger')
    world = DeclareLaunchArgument('world', default_value='')
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')

    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    gazebo_launch = os.path.join(pkg_tb3_gazebo, 'launch', 'turtlebot3_world.launch.py')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={
            'model': LaunchConfiguration('model'),
            'world': LaunchConfiguration('world')
        }.items()
    )

    params_file = os.path.join(get_package_share_directory('tb3_obstacle_avoidance'), 'config', 'obstacle_avoidance.yaml')
    avoidance = Node(
        package='tb3_obstacle_avoidance',
        executable='obstacle_avoidance_node',
        name='obstacle_avoidance_node',
        output='screen',
        parameters=[params_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        model,
        world,
        use_sim_time,
        gazebo,
        avoidance
    ])






