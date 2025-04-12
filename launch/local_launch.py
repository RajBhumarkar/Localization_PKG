from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='local_pkg',
            namespace='rotor',
            executable='serial_node_rotor',
            name='rotor_node',
            output = 'screen'
        ),
        Node(
            package='local_pkg',
            namespace='imu',
            executable='serial_node_imu',
            name='imu_node',
            
        )
    ])