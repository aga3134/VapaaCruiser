from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vapaa_cruiser',
            node_namespace='vapaa_cruiser',
            node_executable='serial_command',
            node_name='serial_command',
            output='screen',
        )
        
    ])