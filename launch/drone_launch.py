from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='orb_slam2_ros',
            executable='orb_slam2_node',
            name='orb_slam2_node',
            output='screen'
        ),
        Node(
            package='yolo_ros',
            executable='yolo_node',
            name='yolo_node',
            output='screen'
        ),
        Node(
            package='pid_controller_ros',
            executable='pid_controller_node',
            name='pid_controller_node',
            output='screen'
        )
    ])
