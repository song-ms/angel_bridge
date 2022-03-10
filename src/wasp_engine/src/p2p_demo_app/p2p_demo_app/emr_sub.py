import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool

import socket
import time

# from __future__ import print_function

import logging
import grpc
import p2p_demo_app.helloworld_pb2 as helloworld_pb2
import p2p_demo_app.helloworld_pb2_grpc as helloworld_pb2_grpc


# 접속 정보 설정
SERVER_IP = '127.0.0.1'
SERVER_PORT = 8003
SIZE = 1024
SERVER_ADDR = (SERVER_IP, SERVER_PORT)

class MinimalSubscriber(Node):

    def __init__(self, rclpy) :
        super().__init__('wifi_p2p')
        self.socket_queue = []
        self.subscription = self.create_subscription(
            Bool,
            '/topic/button/emr/bool',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.rclpy_ =  rclpy
        # self.run()
        self.channel = grpc.insecure_channel('localhost:50051')
        
        
        # 클라이언트 소켓 설정
       
        # with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
        #     client_socket.connect(SERVER_ADDR)  # 서버에 접속
           
        #     msg = client_socket.recv(SIZE)  # 서버로부터 응답받은 메시지 반환
        #     print("resp from server : {}".format(msg))  # 서버로부터 응답받은 메시지 출력

    def listener_callback(self, msg):
        print("[Call] listener_callback")
        self.socket_queue.append(msg.data)
        self.get_logger().info('I heard: "%s"' % msg.data)
    
    def run(self):
        try:
            # self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            # self.client_socket.connect(SERVER_ADDR) 
            # print("======== 호스트 서버와 연결 완료 ============")
            # self.client_socket.send('hi'.encode())  # 서버에 메시지 전송
            # socket_msg = self.client_socket.recv(SIZE)  # 서버로부터 응답받은 메시지 반환
            # print("resp from server : {}".format(socket_msg))  # 서버로부터 응답받은 메시지 출력
            # print("======== init 종료 ============")
            stub = helloworld_pb2_grpc.GreeterStub(self.channel)
            response = stub.SayHello(helloworld_pb2.HelloRequest(name='you'))
            print("Greeter client received: " + response.message)
            while True:
                self.rclpy_.spin_once(self)
                if not self.socket_queue:
                    print("값 없음")
                else:
                    v = self.socket_queue.pop(0)
                    print("---- 큐 받은값 ", v)
                    try:
                        response = stub.SayButton(helloworld_pb2.HelloButton(emrButton = v))
                        print("Greeter client received: " + response.message)
                        # if v == True:
                            # self.client_socket.send("true".encode())  # 서버에 메시지 전송
                        # else:
                            # self.client_socket.send("false".encode())  # 서버에 메시지 전송
                       
                    except Exception as msg:
                        print("쓰기 에러 | Socket Error: %s" % msg)
                        pass
                time.sleep(0.1)
        except socket.error as msg:
            print("Socket Error: %s" % msg)
        
            

    def closeSocket(self):
        self.client_socket.close()

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber(rclpy)
    # rclpy.spin(minimal_subscriber)
    minimal_subscriber.run()
    
    # minimal_subscriber.run()
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # minimal_subscriber.closeSocket()
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
