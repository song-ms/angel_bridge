from launch import LaunchDescription
from launch_ros.actions import Node

node_name = "eth_bridge"

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="wasp_engine_demo",
            executable=node_name,
            name=node_name,
            output="screen",
            emulate_tty=True,
            parameters=[
                {"cm_ip_addr": "192.168.0.181"},
                {"cm_port": 7},
            ]
        )
    ])