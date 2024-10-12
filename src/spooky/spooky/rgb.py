import can
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class rgb(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.bus = can.interface.Bus(interface='socketcan', channel='vcan0', bitrate='500000')

    # method to convert to INT16 Big Endian (i hope)
    # also returns two variables for (AB)
    def conval(self, value):
        # just so everyone knows, this conversion was confusing as hell
        prog = int(1000 - (value * (1000 - 500) / 255))
        global x
        global y
        x = (prog >> 8) & 0xFF
        y = prog & 0xFF
        return x, y

    # callable method that allows for using classic rgb(255,255,255)
    # does this by assigning the converted hex values to an array
    def setrgb(self, r, g, b):
        global values
        values = [0] * 8
        self.conval(r)
        values[2] = x
        values[3] = y
        self.conval(g)
        values[4] = x
        values[5] = y
        self.conval(b)
        values[6] = x
        values[7] = y

    # can method to publish the can message, derrived from the topmost method
    # theoretically should reproduce the [0,0,0,0,0,0,0,0]
    def timer_callback(self):
        self.can_publish(30, values, True)

def main(args=None):
    rclpy.init(args=args)

    rgb_leds = rgb()

    rclpy.spin(rgb_leds)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rgb_leds.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()