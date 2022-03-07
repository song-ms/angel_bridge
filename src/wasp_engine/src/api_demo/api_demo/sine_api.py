import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Float32


wave_topic = "sine_wave"
freq_topic = "sine_freq"

class API:
    def __init__(self, app_name):
        rclpy.init()
        self.node_ = Node(app_name)
        self.pub_ = self.node_.create_publisher(Float32, freq_topic, 10)
        self.sub_ = self.node_.create_subscription(JointTrajectory, wave_topic, self.WaveCallback, 10)
        self.pos = 0
        self.node_.get_logger().info('Start!')

    def SetFreq(self, freq):
        msg = Float32()
        msg.data = float(freq)
        self.pub_.publish(msg)
    
    def ReadWave(self):
        rclpy.spin_once(self.node_)
        return self.pos

    def Terminate(self):
        self.node_.get_logger().info('Shutting down!')
        rclpy.shutdown()

    def WaveCallback(self, msg):
        self.pos = msg.points[0].positions[0]
        self.vel = msg.points[0].velocities[0]
        self.acc = msg.points[0].accelerations[0]

