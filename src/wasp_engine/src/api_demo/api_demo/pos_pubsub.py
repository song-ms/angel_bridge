import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

node_name = "sine_pub"
pub_topic_name = "/eth_bridge/joint_state_topic"
sub_topic_name = "/eth_bridge/joint_target_topic"

joint_names = ["l_hip", "l_knee", "r_hip", "r_knee"]

prerid = 0.01
gain = 0.01

class PosPubSub(Node):
    def __init__(self):
        super().__init__(node_name)
        self.tim_ = self.create_timer(prerid, self.TimerCallback)
        self.pub_ = self.create_publisher(JointTrajectory, pub_topic_name, 10)
        self.sub_ = self.create_subscription(JointTrajectory, sub_topic_name, self.SubsCallback, 10)
        self.t_ = 0
        
        self.pos_state_ = [0.0, 0.0, 0.0, 0.0]
        self.pos_command_ = [0.0, 0.0, 0.0, 0.0]

        self.get_logger().info("Start Position PubSub!")

    def SubsCallback(self, msg:JointTrajectory):
        self.pos_command_ = msg.points[0].effort

    def TimerCallback(self):
        self.t_ += prerid
        new_pos = []
        for i in range(4):
            new_pos.append((1-gain)*self.pos_state_[i] + gain*self.pos_command_[i])
        self.pos_state_ = new_pos
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = joint_names
        pt = JointTrajectoryPoint()
        pt.positions = self.pos_state_
        msg.points.append(pt)
        self.pub_.publish(msg)
        for j in range(4):
            self.get_logger().info(f"{joint_names[j]}:\tstate = {self.pos_state_[j]:.2f},\tcmd = {self.pos_command_[j]:.2f}")
        self.get_logger().info("---------------------------------------------------------------------")

def main(args = None):
    rclpy.init(args=args)
    pos_pubsub = PosPubSub()
    try:
        rclpy.spin(pos_pubsub)
    except:
        pos_pubsub.get_logger().info("Position PubSub Shutting Down!")
    pos_pubsub.destroy_node()
    rclpy.shutdown()
