
import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt8MultiArray, MultiArrayDimension
import RPi.GPIO as GPIO
import time


class BackpackButtonPublisher(Node):

    def __init__(self):

        super().__init__('backpack_button_publisher')
        print('Create backpack_button_publisher')
        self.prev_value_ = None
        self.prev_value_0_ = None
        self.prev_value_1_ = None
        self.prev_value_2_ = None
        self.publisher_button_ = self.create_publisher(UInt8MultiArray, '/topic/button/backpack', 10)

        timer_period = 0.5  # seconds
        self.msg2 = UInt8MultiArray()
        self.left_btn = 32
        self.left_btn_1 = 29
        self.right_btn_0 = 31
        self.right_btn_1 = 33

        GPIO.setmode(GPIO.BOARD)  # BOARD pin-numbering scheme
        GPIO.setup(self.left_btn, GPIO.IN)  # button pin set as input
        GPIO.setup(self.left_btn_1, GPIO.IN)  # button pin set as input
        GPIO.setup(self.right_btn_0, GPIO.IN)  # button pin set as input
        GPIO.setup(self.right_btn_1, GPIO.IN)  # button pin set as input
        # GPIO.add_event_detect(self.left_btn, GPIO.FALLING, callback=self.btn_pressed, bouncetime=10)
       
        self.init_value_ = GPIO.input(self.left_btn)
        self.init_value_1_ = GPIO.input(self.left_btn_1)
        self.init_value_2_ = GPIO.input(self.right_btn_0)
        self.init_value_3_ = GPIO.input(self.right_btn_1)
        
        if self.init_value_ == GPIO.HIGH:
            self.msg2.data = [0x01, 0x00, 0x00, 0x00]
        else:
            self.msg2.data = [0x00, 0x00, 0x00, 0x00]

        if self.init_value_1_ == GPIO.HIGH:
            self.msg2.data[1] = 0x01
        else:
            self.msg2.data[1] = 0x00
        
        if self.init_value_2_ == GPIO.HIGH:
            self.msg2.data[2] = 0x01
        else:
            self.msg2.data[2] = 0x00
        
        if self.init_value_3_ == GPIO.HIGH:
            self.msg2.data[3] = 0x01
        else:
            self.msg2.data[3] = 0x00
        
        
        self.msg2.layout.dim.append(MultiArrayDimension())
        self.msg2.layout.dim[0].label = "btn_lt_0"
        self.msg2.layout.dim[0].size = 1
        self.msg2.layout.dim.append(MultiArrayDimension())
        self.msg2.layout.dim[1].label = "btn_lt_1"
        self.msg2.layout.dim[1].size = 1
        self.msg2.layout.dim.append(MultiArrayDimension())
        self.msg2.layout.dim[2].label = "btn_rt_0"
        self.msg2.layout.dim[2].size = 1
        self.msg2.layout.dim.append(MultiArrayDimension())
        self.msg2.layout.dim[3].label = "btn_rt_1"
        self.msg2.layout.dim[3].size = 1
        self.publisher_button_.publish(self.msg2)
        # print("Value read from pin {} : {}".format(input_pin, value_str))

        print("Starting demo now! Press CTRL+C to exit")
   
    def btn_pressed(self, channel):
        print("==== Backpack Button | Left 0 | Callback =====")
        # print(GPIO.input(channel))
        btn_ = 0x01
        value = GPIO.input(channel)
        print(type(channel))
        print(channel)
        if value == GPIO.HIGH:
            print("OFF")
        else:
            print("Pushed Pressed")
            btn_ = 0x00
        # print(btn_)
        # self.msg2.data =  btn_
       
        self.msg2.data = [btn_]
        print(self.msg2)
        try:
            self.publisher_button_.publish(self.msg2)
        except:
            print("Error Publisher")
        self.get_logger().info('Publishing Backpack Button State: "%s"' % self.msg2.data)

    def run(self):
        try:
            while True:
                value0 = GPIO.input(self.left_btn)
                value1 = GPIO.input(self.left_btn_1)
                value2 = GPIO.input(self.right_btn_0)
                value3 = GPIO.input(self.right_btn_1)
                if value0 != self.prev_value_:
                    if value0 == GPIO.HIGH:
                        value_str = "HIGH"
                        # self.msg2.data.insert(0, 0x01)
                        self.msg2.data[0] = 0x01
                    else:
                        value_str = "LOW"
                        self.msg2.data[0] = 0x00
                        # self.msg2.data.insert(0, 0x00)
                    print("Value read from pin {} : {}".format(self.left_btn, value_str))
                    self.publisher_button_.publish(self.msg2)
                    self.prev_value_ = value0

                if value1 != self.prev_value_0_:
                    if value1 == GPIO.HIGH:
                        value_str = "HIGH"
                        # self.msg2.data.insert(0, 0x01)
                        self.msg2.data[1] = 0x01
                    else:
                        value_str = "LOW"
                        self.msg2.data[1] = 0x00
                        # self.msg2.data.insert(0, 0x00)
                    print("Value read from pin {} : {}".format(self.left_btn_1, value_str))
                    self.publisher_button_.publish(self.msg2)
                    self.prev_value_0_ = value1

                if value2 != self.prev_value_1_:
                    if value2 == GPIO.HIGH:
                        value_str = "HIGH"
                        # self.msg2.data.insert(0, 0x01)
                        self.msg2.data[2] = 0x01
                    else:
                        value_str = "LOW"
                        self.msg2.data[2] = 0x00
                        # self.msg2.data.insert(0, 0x00)
                    print("Value read from pin {} : {}".format(self.right_btn_0, value_str))
                    self.publisher_button_.publish(self.msg2)
                    self.prev_value_1_ = value2
                time.sleep(0.01)
        finally:
            GPIO.cleanup()


def main(args=None):
    rclpy.init(args=args)

    backpack_button_publisher = BackpackButtonPublisher()
    backpack_button_publisher.run()
    backpack_button_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()