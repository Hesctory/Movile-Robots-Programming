from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    robo_mapper = Node(
        package='robot_mapper',
        executable='robo_mapper',
        name='robo_mapper',
        output='screen',
    )

    controle = Node(
        package='robot_control',
        executable='controle_robo',
        name='controle_do_robo',
        output='screen',
    )

    return LaunchDescription([
        robo_mapper,
        controle,
    ])
