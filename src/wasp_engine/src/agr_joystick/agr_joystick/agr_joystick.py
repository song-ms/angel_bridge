
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


# Iterate over the joystick devices.
print('Available devices:')

for fn in os.listdir('/dev/input'):
    if fn.startswith('js'):
        print('/dev/input/%s' % (fn))

# We'll store the states here.
axis_states = {}
button_states = {}

# These constants were borrowed from linux/input.h
CODE_MAP = {
    'axis0' : 0,
    'axis1' : 1,
    'axis2' : 2,
    'axis3' : 3,
    'axis4' : 4,
    'axis5' : 5,
}

BUTTON_CODE_MAP = {
    'btn0' : 0,
    'btn1' : 1,
    'btn2' : 2,
    'btn3' : 3,
    'btn4' : 4,
    'btn5' : 5,
    'btn6' : 6,
    'btn7' : 7,
    'btn8' : 8,
    'btn9' : 9,
    'btn10' : 10,
    'btn11' : 11,
}

axis_names = {
    0x00 : 'axis0',
    0x01 : 'axis1',
    0x02 : 'axis2',
    0x03 : 'rx',
    0x04 : 'ry',
    0x05 : 'axis3',
    0x06 : 'throttle',
    0x07 : 'rudder',
    0x08 : 'wheel',
    0x09 : 'gas',
    0x0a : 'brake',
    0x10 : 'axis4',
    0x11 : 'axis5',
    0x12 : 'hat1x',
    0x13 : 'hat1y',
    0x14 : 'hat2x',
    0x15 : 'hat2y',
    0x16 : 'hat3x',
    0x17 : 'hat3y',
    0x18 : 'pressure',
    0x19 : 'distance',
    0x1a : 'tilt_x',
    0x1b : 'tilt_y',
    0x1c : 'tool_width',
    0x20 : 'volume',
    0x28 : 'misc',
}

button_names = {
    0x120 : 'btn0',
    0x121 : 'btn1',
    0x122 : 'btn2',
    0x123 : 'btn3',
    0x124 : 'btn4',
    0x125 : 'btn5',
    0x126 : 'btn6',
    0x127 : 'btn7',
    0x128 : 'btn8',
    0x129 : 'btn9',
    0x12a : 'btn10',
    0x12b : 'btn11',
    0x12f : 'dead',
    0x130 : 'a',
    0x131 : 'b',
    0x132 : 'c',
    0x133 : 'x',
    0x134 : 'y',
    0x135 : 'z',
    0x136 : 'tl',
    0x137 : 'tr',
    0x138 : 'tl2',
    0x139 : 'tr2',
    0x13a : 'select',
    0x13b : 'start',
    0x13c : 'mode',
    0x13d : 'thumbl',
    0x13e : 'thumbr',

    0x220 : 'dpad_up',
    0x221 : 'dpad_down',
    0x222 : 'dpad_left',
    0x223 : 'dpad_right',

    # XBox 360 controller uses these codes.
    0x2c0 : 'dpad_left',
    0x2c1 : 'dpad_right',
    0x2c2 : 'dpad_up',
    0x2c3 : 'dpad_down',
}

axis_map = []
button_map = []

# Open the joystick device.
fn = '/dev/input/js0'
print('Opening %s...' % fn)
jsdev = open(fn, 'rb')

# Get the device name.
#buf = bytearray(63)
buf = array.array('B', [0] * 64)
ioctl(jsdev, 0x80006a13 + (0x10000 * len(buf)), buf) # JSIOCGNAME(len)
js_name = buf.tobytes().rstrip(b'\x00').decode('utf-8')
print('Device name: %s' % js_name)

# Get number of axes and buttons.
buf = array.array('B', [0])
ioctl(jsdev, 0x80016a11, buf) # JSIOCGAXES
num_axes = buf[0]

buf = array.array('B', [0])
ioctl(jsdev, 0x80016a12, buf) # JSIOCGBUTTONS
num_buttons = buf[0]

# Get the axis map.
buf = array.array('B', [0] * 0x40)
ioctl(jsdev, 0x80406a32, buf) # JSIOCGAXMAP

for axis in buf[:num_axes]:
    print(axis)
    axis_name = axis_names.get(axis, 'unknown(0x%02x)' % axis)
    axis_map.append(axis_name)
    axis_states[axis_name] = 0.0

# Get the button map.
buf = array.array('H', [0] * 200)
ioctl(jsdev, 0x80406a34, buf) # JSIOCGBTNMAP

for btn in buf[:num_buttons]:
    btn_name = button_names.get(btn, 'unknown(0x%03x)' % btn)
    button_map.append(btn_name)
    button_states[btn_name] = 0

print('%d axes found: %s' % (num_axes, ', '.join(axis_map)))
print('%d buttons found: %s' % (num_buttons, ', '.join(button_map)))


class AgrJoystickPublisher(Node):

    def __init__(self):
        super().__init__('agr_joystick_publisher')
          # Joy message
        self.joy = Joy()
        self.joy.header = Header()
        self.joy.header.frame_id = ''
        self.joy.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joy.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        # Joy publisher
        self.publisher_ = self.create_publisher(Joy, '/controller/joystic', 10)

    def publish_joy(self):
        current_time = modf(time.time())
        self.joy.header.stamp.sec = int(current_time[1])
        self.joy.header.stamp.nanosec = int(current_time[0] * 1000000000) & 0xffffffff
        self.publisher_.publish(self.joy)
        self.last_publish_time = time.time()
       
    def run(self):
         while True:
            evbuf = jsdev.read(8)
            if evbuf:
                time, value, type, number = struct.unpack('IhBB', evbuf)

                if type & 0x80:
                    print("(initial)", end="")

                if type & 0x01:
                    button = button_map[number]
                    print(button)
                    if button in BUTTON_CODE_MAP:
                        if button:
                            button_states[button] = value
                            self.joy.buttons[BUTTON_CODE_MAP[button]] = value
                            self.publish_joy()
                            if value:
                                print("%s pressed" % (button))
                            else:
                                print("%s released" % (button))
                        

                if type & 0x02:
                    axis = axis_map[number]
                    print(axis)
                    if  axis in CODE_MAP:
                        print(CODE_MAP[axis])
                        if axis:
                            fvalue = value / 32767.0
                            axis_states[axis] = fvalue
                            # print(axis_states[axis].Key)
                            
                            self.joy.axes[CODE_MAP[axis]] = fvalue
                            self.publish_joy()
                            print("%s: %.3f" % (axis, fvalue))

def main(args=None):
    rclpy.init(args=args)

    agr_joystick_publisher = AgrJoystickPublisher()
    agr_joystick_publisher.run()
  
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
