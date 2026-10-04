from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='edge_robot',
            executable='state_estimator',
            name='state_estimator_node',
            output='screen'
        ),
        Node(
            package='edge_robot',
            executable='pid',
            name='pid_node',
            output='screen'
        ),
        Node(
            package='edge_robot',
            executable='mock_imu',
            name='mock_imu_node',
            output='screen'
        )
    ])