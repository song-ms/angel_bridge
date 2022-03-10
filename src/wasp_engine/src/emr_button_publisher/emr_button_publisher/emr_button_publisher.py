
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import UInt8MultiArray, MultiArrayDimension, Bool
import RPi.GPIO as GPIO
import time


class EmrButtonPublisher(Node):

    def __init__(self):

        super().__init__('emr_button_publisher')
        # self.publisher_ = self.create_publisher(String, '/topic', 10)
        self.publisher_button_ = self.create_publisher(UInt8MultiArray, '/topic/button/emr', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.but_pin = 15
        GPIO.setmode(GPIO.BOARD)  # BOARD pin-numbering scheme
        GPIO.setup(self.but_pin, GPIO.IN)  # button pin set as input
        GPIO.add_event_detect(self.but_pin, GPIO.BOTH, callback=self.blink, bouncetime=10)
        print("Starting demo now! Press CTRL+C to exit")
   
    def blink(self, channel):
        print("==== EMR Button Callback=====")
        print(GPIO.input(channel))
        btn_ = 0x01
        if GPIO.input(channel) == GPIO.HIGH:
            print("Released")
            btn_ = 0x01
        else:
            print("Pressed")
            btn_ = 0x00

        msg2 = UInt8MultiArray()
        msg2.layout.dim.append(MultiArrayDimension())
        msg2.layout.dim[0].label = "btn_emr"
        msg2.layout.dim[0].size = 1
        msg2.data = [btn_]
        print(msg2)
        self.publisher_button_.publish(msg2)
        self.get_logger().info('Publishing button: "%s"' % msg2.data)

    def run(self):
        try:
            while True:
                # blink LED 1 slowly
                time.sleep(0.01)
                pass
        finally:
            GPIO.cleanup()  # cleanup all GPIOs

    def timer_callback(self):
        # msg = String()
        # msg.data = 'Hello World: %d' % self.i
        # self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        
        msg2 = UInt8MultiArray()
        self.i += 1
        msg2.layout.dim.append(MultiArrayDimension())
        msg2.layout.dim[0].label = "btn_emr"
        msg2.layout.dim[0].size = 1

        msg2.data = [self.i]
        print(msg2)
        self.publisher_button_.publish(msg2)
        self.get_logger().info('Publishing button: "%s"' % msg2.data)


def main(args=None):
    rclpy.init(args=args)

    emr_button_publisher = EmrButtonPublisher()
    emr_button_publisher.run()

    # rclpy.spin(emr_button_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    emr_button_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()