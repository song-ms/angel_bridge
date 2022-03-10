from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float32

node_name = "sine_pub"
wave_topic = "sine_wave"
freq_topic = "sine_freq"

joint_name = "test"

prerid = 0.01

class SinePub(Node):
    def __init__(self):
        super().__init__(node_name)
        self.tim_ = self.create_timer(prerid, self.TimerCallback)
        self.pub_ = self.create_publisher(JointTrajectory, wave_topic, 10)
        self.sub_ = self.create_subscription(Float32, freq_topic, self.FreqCallback, 10)
        self.t_ = 0
        self.hz_ = 0
        self.get_logger().info("Publish frequency!")
        self.get_logger().info(f"Sine wave frequency: {self.hz_}Hz")

    def FreqCallback(self, msg):
        self.hz_ = msg.data
        self.get_logger().info(f"Sine wave frequency: {self.hz_}Hz")

    def TimerCallback(self):
        self.t_ += prerid
        freq = 2*pi*self.hz_
        phase = freq*self.t_

        pos =           sin(phase)
        vel =      freq*cos(phase)
        acc = freq*freq*sin(phase)

        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names.append(joint_name)

        pt = JointTrajectoryPoint()
        pt.positions.append(pos)
        pt.velocities.append(vel)
        pt.accelerations.append(acc)
        msg.points.append(pt)

        self.pub_.publish(msg)


def main(args = None):
    rclpy.init(args=args)
    sine_pub = SinePub()
    rclpy.spin(sine_pub)
    sine_pub.destroy_node()
    rclpy.shutdown()
