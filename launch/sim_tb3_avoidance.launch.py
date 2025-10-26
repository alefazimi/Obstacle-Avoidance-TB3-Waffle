
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
import os


def generate_launch_description():
    model = DeclareLaunchArgument('model', default_value='waffle')
    world = DeclareLaunchArgument('world', default_value='')
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')

    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    gazebo_launch = os.path.join(pkg_tb3_gazebo, 'launch', 'turtlebot3_world.launch.py')

    # t=0s: Gazebo launching
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={
            'model': LaunchConfiguration('model'),
            'world': LaunchConfiguration('world')
        }.items()
    )

    gazebo_msg = LogInfo(
        msg=['\n', '='*60, '\n',
             '  TIMED SIMULATION STARTED\n',
             '  -----------------------\n',
             '  t=0s: gazebo launching...\n',
             '  t=5s: RViz2 will launch\n',
             '  t=15s: robot will start moving\n',
             '='*60, '\n']
    )

    # SLAM Toolbox
    slam_params = os.path.join(get_package_share_directory('slam_toolbox'), 'config', 'mapper_params_online_async.yaml')
    slam = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # t=5s: RViz2 launching
    rviz_config = os.path.join(get_package_share_directory('tb3_obstacle_avoidance'), 'config', 'rviz', 'tb3_mapping.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    rviz_delayed = TimerAction(
        period=5.0, 
        actions=[
            LogInfo(msg=['\n', '='*60, '\n',
                        '  RVIZ2 LAUNCHED (t=5s)\n',
                        '  Visualization started. Robot will move in 10 seconds...\n',
                        '='*60, '\n']),
            rviz
        ]
    )

    # t=15s: Obstacle Avoiding
    params_file = os.path.join(get_package_share_directory('tb3_obstacle_avoidance'), 'config', 'obstacle_avoidance.yaml')
    avoidance = Node(
        package='tb3_obstacle_avoidance',
        executable='obstacle_avoidance_node',
        name='obstacle_avoidance_node',
        output='screen',
        parameters=[params_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    avoidance_delayed = TimerAction(
        period=15.0, 
        actions=[
            LogInfo(msg=['\n', '='*60, '\n',
                        '  OBSTACLE AVOIDANCE STARTED (t=15s)\n',
                        '  ROBOT IS NOW MOVING! \n',
                        '='*60, '\n']),
            avoidance
        ]
    )

    
    clear_map = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/slam_toolbox/clear_queue', 'slam_toolbox/srv/ClearQueue'],
        output='screen'
    )

    clear_map_delayed = TimerAction(period=7.0, actions=[clear_map])

    return LaunchDescription([
        model,
        world,
        use_sim_time,
        gazebo_msg,
        gazebo,
        slam,
        rviz_delayed,
        clear_map_delayed,
        avoidance_delayed,
    ])





