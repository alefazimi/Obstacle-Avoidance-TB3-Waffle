from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    model = DeclareLaunchArgument('model', default_value='burger')

    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    gazebo_world = os.path.join(get_package_share_directory('tb3_obstacle_avoidance'), 'worlds', 'home.world')
    gazebo_launch = os.path.join(pkg_tb3_gazebo, 'launch', 'turtlebot3_world.launch.py')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={
            'model': LaunchConfiguration('model'),
            'world': gazebo_world,
            'server_required': 'true',
            'gui_required': 'false'
        }.items()
    )

    slam_params = os.path.join(get_package_share_directory('slam_toolbox'), 'config', 'mapper_params_online_async.yaml')
    slam = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    params_file = os.path.join(get_package_share_directory('tb3_obstacle_avoidance'), 'config', 'obstacle_avoidance.yaml')
    avoidance = Node(
        package='tb3_obstacle_avoidance',
        executable='obstacle_avoidance_node',
        name='obstacle_avoidance_node',
        output='screen',
        parameters=[params_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    rviz_config = os.path.join(get_package_share_directory('tb3_obstacle_avoidance'), 'config', 'rviz', 'tb3_mapping.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        use_sim_time,
        model,
        gazebo,
        slam,
        avoidance,
        rviz
    ])






