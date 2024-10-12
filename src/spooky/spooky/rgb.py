import rclpy
from rclpy.node import Node
import can
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
    # does this by assigning the converted hex values to an array[12]
    def setrgb(self, r, g, b):
        global values
        values = [0,0]*6
        i = 1
        self.conval(r)
        values[i*0] = x
        values[i*1] = y
        self.conval(g)
        values[i*2] = x
        values[i*3] = y
        self.conval(b)
        values[i*4] = x
        values[i*5] = y

    # i couldn't get the two leading 0's without doing an array slice
    # theoretically should reproduce the [0,0,0,0,0,0,0,0]
    def timer_callback(self):
        self.can_publish(30, values[:8], True)

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