
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Bool, '/topic/button/emr/bool', 10)
        self.publisher_liveness = self.create_publisher(String, '/controller_version', 10)
        timer_period = 5  # seconds
        timer_period2 = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.timer_liveness = self.create_timer(timer_period2, self.timer_callback_liveness)
        self.i = 0

    def timer_callback(self):

        msg = Bool()
        if self.i%2 == 0 :
            msg.data = True
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)
        else : 
            msg.data = False
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

    def timer_callback_liveness(self):
        msg = String()
        msg.data = "0.1.0"
        self.publisher_liveness.publish(msg)
        self.get_logger().info('version: "%s"' % msg.data)



def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()