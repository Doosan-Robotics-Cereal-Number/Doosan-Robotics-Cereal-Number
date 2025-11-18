from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Launch Arguments
    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        default_value='dsr01',
        description='Robot ID'
    )
    
    return LaunchDescription([
        robot_id_arg,
        
        # Motion Control Node (v2)
        Node(
            package='gwonho_orchestrator',
            executable='motion_control_v3',
            name='motion_control',
            namespace=LaunchConfiguration('robot_id'),
            output='screen',
            emulate_tty=True,
            parameters=[{
                'use_sim_time': False,
            }],
            arguments=['--ros-args', '--log-level', 'info'],
            prefix='bash -c "sleep 1; $0 $@"',  # 1초 대기 후 시작
        ),
        
        # Network Manager Node (v2)
        Node(
            package='gwonho_orchestrator',
            executable='network_manager_v3',
            name='network_manager',
            namespace=LaunchConfiguration('robot_id'),
            output='screen',
            emulate_tty=True,
            parameters=[{
                'robot_id': LaunchConfiguration('robot_id'),
                'use_sim_time': False,
            }],
            prefix='bash -c "sleep 2; $0 $@"',  # 2초 대기 후 시작
        ),
    ])