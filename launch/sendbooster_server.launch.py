from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # URDF 파일 경로 설정
    urdf_file = os.path.join(
        get_package_share_directory('sendbooster_server'),
        'urdf', 'amr.urdf'
    )

    # LaunchDescription 반환
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        
        # robot_state_publisher 노드 실행 (URDF 파일 경로 사용)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file, 'r').read()}]
        ),
        
        # sendbooster_odom 노드 실행
        Node(
            package='sendbooster_server',
            executable='sendbooster_odom',
            name='sendbooster_odom_node',
            output='screen',
            parameters=[{'some_param': 'value'}]  # 기존 파라미터 추가
        ),
        # sendbooster_odom 노드 실행
        Node(
            package='sendbooster_server',
            executable='sendbooster_JointStatePublisher',
            name='sendbooster_JointStatePublisher_node',
            output='screen',
            parameters=[{'some_param': 'value'}]  # 기존 파라미터 추가
        ),
    ])
