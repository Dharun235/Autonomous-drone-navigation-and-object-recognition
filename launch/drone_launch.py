from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='worlds/orchard.world', description='Gazebo world file'),

        Node(
            package='gazebo_ros',
            executable='gazebo',
            name='gazebo',
            output='screen',
            arguments=['-s', 'libgazebo_ros_factory.so', LaunchConfiguration('world')]
        ),

        Node(
            package='orb_slam2_ros',
            executable='orb_slam2_node',
            name='orb_slam2_node',
            output='screen',
            parameters=[{
                'vocabulary_file': '/path/to/ORBvoc.txt',
                'settings_file': '/path/to/TUM1.yaml'
            }]
        ),

        Node(
            package='yolo_ros',
            executable='yolo_node',
            name='yolo_node',
            output='screen',
            parameters=[{
                'config_file': '/path/to/yolov3.cfg',
                'weights_file': '/path/to/yolov3.weights',
                'data_file': '/path/to/coco.data'
            }]
        ),

        Node(
            package='pid_controller_ros',
            executable='pid_controller_node',
            name='pid_controller_node',
            output='screen',
            parameters=[{
                'start_x': 0.0,
                'start_y': 0.0,
                'start_z': 1.0,
                'goal_x': 10.0,
                'goal_y': 10.0,
                'goal_z': 1.0
            }]
        )
    ])
