import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('deepscan_gazebo')
    world_file = os.path.join(pkg_share, 'worlds', 'water.world')

    # Статический трансформ base_footprint -> base_link
    static_tf_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0.4',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'base_footprint',
                   '--child-frame-id', 'base_link'],
        output='screen'
    )

    # Статический трансформ world -> odom (world теперь фиксированный родитель)
    static_tf_world = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'world',
                   '--child-frame-id', 'odom'],
        output='screen'
    )

    # Запуск Gazebo с миром
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             world_file],
        output='screen'
    )

    # Спавн катера через 8 секунд
    spawn = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=['python3', os.path.join(pkg_share, 'scripts', 'spawn_cube.py')],
                output='screen'
            )
        ]
    )
    web_video_server = ExecuteProcess(
        cmd=['ros2', 'run', 'web_video_server', 'web_video_server'],
        output='screen'
    )

    return LaunchDescription([
	web_video_server,
        static_tf_base,
        static_tf_world,
        gazebo,
        spawn,
    ])
