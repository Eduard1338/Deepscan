import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('deepscan_gazebo')
    world_file = os.path.join(pkg_share, 'worlds', 'water.world')
    urdf_file  = os.path.join(pkg_share, 'models', 'deepscan_boat.urdf')

    # Запуск Gazebo с ROS-инициализацией И фабрикой
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             world_file],
        output='screen'
    )

    # Спавн лодки через 8 секунд
    spawn = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-file', urdf_file,
                    '-entity', 'deepscan_boat',
                    '-x', '0', '-y', '0', '-z', '2.0'
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        gazebo,
        spawn,
    ])
