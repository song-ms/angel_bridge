from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="wasp_engine_demo",
            executable="cm_emulator",
            name="cm_emulator",
            output="screen",
            emulate_tty=True,
        ),

        Node(
            package="wasp_engine_demo",
            executable="eth_bridge",
            name="eth_bridge",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"cm_ip_addr": "127.0.0.1"},
                {"cm_port": 1818},
            ]
        )
    ])