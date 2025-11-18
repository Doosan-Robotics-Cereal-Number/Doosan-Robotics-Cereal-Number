from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gwonho_orchestrator',
            executable='motion_control',
            name='motion_control',
            namespace='dsr01',
            output='screen',
        ),
        Node(
            package='gwonho_orchestrator',
            executable='network_manager',
            name='network_manager',
            namespace='dsr01',  # ← 이거 추가!
            output='screen',
        ),
    ])