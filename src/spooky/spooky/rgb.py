import can
import rclpy
import time
from rclpy.node import Node
# from std_msgs.msg import String


class rgb(Node):

    def __init__(self):
        super().__init__('rgb_leds')
        timer_period = 4  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate='500000')
        self.values = [0] * 8

    def can_publish(self, arbitration_id, data, is_extended_id) -> None:
        can_msg = can.Message(
                arbitration_id = arbitration_id,
                data = data, 
                is_extended_id = is_extended_id
                ) 
        self.bus.send(can_msg)

    # method to convert to INT16 Big Endian (i hope)
    # also returns two variables for (AB)
    def conval(self, value):
        # just so everyone knows, this conversion was confusing as hell
        prog = int(1000 - (value * (1000 - 500) / 255))
        x = (prog >> 8) & 0xFF
        y = prog & 0xFF
        return x, y

    # callable method that allows for using classic rgb(255,255,255)
    # does this by assigning the converted hex values to an array
    def setRGB(self, r, g, b):
        self.values[2], self.values[3] = self.conval(r)
        self.values[4], self.values[5] = self.conval(g)
        self.values[6], self.values[7] = self.conval(b)
        self.can_publish(30, self.values, True)

    # can method to publish the can message, derrived from the topmost method
    # theoretically should reproduce the [0,0,0,0,0,0,0,0]
    def timer_callback(self):
        self.setRGB(255, 0, 0)
        time.sleep(1)
        self.setRGB(0, 255, 0)
        time.sleep(1)
        self.setRGB(0, 0, 255)
        time.sleep(1)

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
