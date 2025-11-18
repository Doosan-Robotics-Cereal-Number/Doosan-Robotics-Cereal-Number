from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    cfg = PathJoinSubstitution([FindPackageShare('ai_ros'), 'rviz', 'cup_view.rviz'])
    return LaunchDescription([
        DeclareLaunchArgument('rviz_config', default_value=cfg),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            output='screen'
        )
    ])
