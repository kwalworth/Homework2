# File: my_launch_file.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Terminal 1
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera_node'
        ),
        
        # Terminal 2
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='rqt_image_view'
        ),
        
        # Terminal 3
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            output='screen',
            arguments=[
                '--ros-args',
                '-r image_rect:=/image_raw',
                '-r camera_info:=/camera_info',
                '--params-file',
                '/opt/ros/humble/share/apriltag_ros/cfg/tags_36h11.yaml'
            ]
        ),
        
        # Terminal 4
        Node(
            package='ros2cli',
            executable='ros2',
            name='ros2_topic_echo',
            arguments=['topic', 'echo', '/detections']
        ),
        
        # Terminal 5
        Node(
            package='turtlebot3_bringup',
            executable='robot',
            name='turtlebot3_bringup',
            output='screen',
            arguments=['launch', 'turtlebot3_bringup', 'robot.launch.py']
        ),
        
        # Terminal 6
        Node(
            package='my_package',
            executable='my_node',
            name='my_node',
            output='screen'
        )
    ])
