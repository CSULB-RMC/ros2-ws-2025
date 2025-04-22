import rclpy
import time
from rclpy.node import Node, Publisher
from sensor_msgs.msg import Joy
from std_msgs.msg import UInt8


class TrapDoor(Node):
    def __init__(self):
        super().__init__("dig_publisher")

        self.subscription = self.create_subscription(
            Joy, "joy", self.listener_callback, 10
        )

        self.trap = self.create_publisher(UInt8, "trap", 10)

    def trapdoor(self, msg):
        uint8 = UInt8()
        # Trap Door
        if msg.buttons[0] == 1:
            uint8.data = 100
            self.trap.publish(uint8)
        elif msg.buttons[0] == 0:
            uint8.data = 0
            self.trap.publish(uint8)

    def listener_callback(self, msg: Joy):
        self.trapdoor(msg)


def main():
    print("Controller On")
    rclpy.init(args=None)

    node = TrapDoor()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
