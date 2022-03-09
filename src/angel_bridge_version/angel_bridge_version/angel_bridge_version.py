
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import String
import pymongo
import json
MONGO_HOSTNAME = '192.168.0.243'
MONGO_PORT = '27017'
MONGO_DB = 'wasp'

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('bridge_version_node')
        self.publisher_liveness = self.create_publisher(String, '/bridge_version', 10)
        timer_period2 = 2  # seconds
        self.client = pymongo.MongoClient('mongodb://'+MONGO_HOSTNAME+':'+MONGO_PORT)
        self.timer_liveness = self.create_timer(timer_period2, self.timer_callback_liveness)
        self.i = 0

    def timer_callback_liveness(self):
        msg = String()
        msg.data = "0.2"
        db = self.client.wasp.bridgever
        version_db = {"id" : "M30", "version" : msg.data}
        db.insert_one(version_db)
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