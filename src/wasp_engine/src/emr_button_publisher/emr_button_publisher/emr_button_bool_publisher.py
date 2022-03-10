
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
import RPi.GPIO as GPIO
import time


class EmrButtonBoolPublisher(Node):

    def __init__(self):

        super().__init__('emr_button_bool_publisher')
        print('Create emr_button_bool_publisher')
        self.publisher_button_ = self.create_publisher(Bool, '/topic/button/emr/bool', 10)

        timer_period = 0.5  # seconds
        self.msg2 = Bool()
        self.but_pin = 15
        GPIO.setmode(GPIO.BOARD)  # BOARD pin-numbering scheme
        GPIO.setup(self.but_pin, GPIO.IN)  # button pin set as input
        # GPIO.add_event_detect(self.but_pin, GPIO.BOTH, callback=self.blink, bouncetime=10)
       
        value = GPIO.input(self.but_pin)
        
        if value == GPIO.HIGH:
            value_str = "HIGH"
            self.msg2.data = True
        else:
            value_str = "LOW"
            self.msg2.data = False
        
        self.publisher_button_.publish(self.msg2)
        # print("Value read from pin {} : {}".format(input_pin, value_str))

        print("Starting demo now! Press CTRL+C to exit")
   
    def blink(self, channel):
        print("==== EMR Button Callback =====")
        # print(GPIO.input(channel))
        btn_ = True
        if GPIO.input(channel) == GPIO.HIGH:
            print("Released")
        else:
            print("Pressed")
            btn_ = False
        print(btn_)
        self.msg2.data =  btn_
        # msg2.layout.dim.append(MultiArrayDimension())
        # msg2.layout.dim[0].label = "btn_emr"
        # msg2.layout.dim[0].size = 1
        # msg2.data = [btn_]
        print(self.msg2)
        try:
            self.publisher_button_.publish(self.msg2)
        except:
            print("Error Publisher")
        self.get_logger().info('Publishing EMR Button State: "%s"' % self.msg2.data)

    def run(self):
        prev_value_ = None
        try:
            while True:
                # blink LED 1 slowly
                # GPIO.wait_for_edge(self.but_pin, GPIO.FALLING)
                # print("Button Pressed!")
                value = GPIO.input(self.but_pin)
                if value != prev_value_:
                    if value == GPIO.HIGH:
                        value_str = "HIGH"
                        # self.msg2.data.insert(0, 0x01)
                        self.msg2.data = True
                    else:
                        value_str = "LOW"
                        self.msg2.data = False
                        # self.msg2.data.insert(0, 0x00)
                    print("Value read from pin {} : {}".format(self.but_pin, value_str))
                    self.publisher_button_.publish(self.msg2)
                    prev_value_ = value
                time.sleep(0.01)
                pass
        finally:
            GPIO.cleanup()  # cleanup all GPIOs


def main(args=None):
    rclpy.init(args=args)

    emr_button_publisher = EmrButtonBoolPublisher()
    emr_button_publisher.run()
    emr_button_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()