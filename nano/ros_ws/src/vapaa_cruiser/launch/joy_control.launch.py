from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            node_namespace='vapaa_cruiser',
            node_executable='joy_node',
            node_name='joy_node'
        ),
        Node(
            package='vapaa_cruiser',
            node_namespace='vapaa_cruiser',
            node_executable='joy_control',
            node_name='joy_control'
        ),
        Node(
            package='vapaa_cruiser',
            node_namespace='vapaa_cruiser',
            node_executable='serial_command',
            node_name='serial_command',
            output='screen',
        )
        
    ])