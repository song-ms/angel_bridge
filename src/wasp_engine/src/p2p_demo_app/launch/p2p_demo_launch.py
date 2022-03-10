from launch import LaunchDescription
from launch_ros.actions import Node
node_name = "eth_bridge"

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="p2p_demo_app",
            executable="emr_pub",
            name="emr_button_bool_publisher",
            output="screen",
            emulate_tty=True,
        ),
        Node(
            package="p2p_demo_app",
            executable="emr_sub",
            name="emr_button_bool_sub",
            output="screen",
            emulate_tty=True,
        ),
        Node(
            package="p2p_demo_app",
            executable="backpack_pub",
            name="backpack_button_publisher",
            output="screen",
            emulate_tty=True,
        ),
        Node(
            package="p2p_demo_app",
            executable="backpack_sub",
            name="backpack_button_sub",
            output="screen",
            emulate_tty=True,
        ),
         Node(
            package="p2p_demo_app",
            executable="joystick_pub",
            name="agr_joystick_publisher",
            output="screen",
            emulate_tty=True,
        ),
        Node(
            package="p2p_demo_app",
            executable="joystick_sub",
            name="agr_joystick_sub",
            output="screen",
            emulate_tty=True,
        )
    ])