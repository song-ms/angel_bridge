
import rclpy
from rclpy.node import Node

import os, struct, array
from fcntl import ioctl

from sensor_msgs.msg import Joy
from std_msgs.msg import String
from std_msgs.msg import Header
from math import modf
import time
import platform

from queue import Queue

class AgrJoystickSubscriber(Node):

    def __init__(self, rclpy):

        super().__init__('agr_joystick_subscriber')
        print('Create agr_joystick_subscriber')
        self.prev_value_ = None
        self.btn_queue_ = Queue()
        self.rclpy_ = rclpy
        # self.grpc_channel = grpc.insecure_channel('localhost:50051')
        self.subscription = self.create_subscription(Joy, '/controller/joystic', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning


    def listener_callback(self, msg):
        print("[Call] listener_callback")
        # self.btn_queue_.put(msg.data)
        self.get_logger().info('I heard: "%s" "%s"' % (msg.buttons, msg.axes))

    def run(self):
        stub = helloworld_pb2_grpc.GreeterStub(self.grpc_channel)
        response = stub.SayHello(helloworld_pb2.HelloRequest(name='bpk_button_sub'))
        print("Greeter client received: " + response.message)
        try:
            while True:
                self.rclpy_.spin_once(self)
                if self.btn_queue_.empty():
                    print("버튼 큐 비어 있음")
                else:
                    v = self.btn_queue_.get()
                    print("---- 큐 받은값 ", v)
                    try:
                        response = stub.SayBackpackButtons(helloworld_pb2.HelloBackpackButtons(buttons = v))
                        print("Greeter client received: " + response.message)
                        # if v == True:
                            # self.client_socket.send("true".encode())  # 서버에 메시지 전송
                        # else:
                            # self.client_socket.send("false".encode())  # 서버에 메시지 전송
                       
                    except Exception as msg:
                        print("쓰기 에러 | Socket Error: %s" % msg)
                        pass
                
                time.sleep(0.01)
        finally:
          pass


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = AgrJoystickSubscriber(rclpy)
    # minimal_subscriber.run()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()