import rclpy
import time
from rclpy.node import Node, Publisher
from sensor_msgs.msg import Joy
from std_msgs.msg import UInt8
from .controls_lib import negativeNumberHandler, pubDelay


class BucketController(Node):
    def __init__(self, bucketSpeedLimit):
        super().__init__("dig_publisher")
        self.declare_parameter("bucket_speed_limit", bucketSpeedLimit)
        self.dig_pub = self.create_publisher(UInt8, "dig_pub", 10)

        self.bucketSpeedLimit = self.get_parameter("bucket_speed_limit").value
        self.bucketSpeed = 0

    def dig(self, msg):
        uint8 = UInt8()

        # i don't know if this is correct?
        if msg.buttons[3] == 1:
            self.bucketSpeed = 10
            uint8.data = self.bucketSpeed
            self.dig_pub.publish(uint8)
            # pubDelay(self.dig_pub, uint8, 0.5)
        elif msg.buttons[3] == 0:
            self.bucketSpeed = 0
            uint8.data = self.bucketSpeed
            self.dig_pub.publish(uint8)
            # pubDelay(self.dig_pub, uint8, 0.5)

    def listener_callback(self, msg: Joy):
        self.dig(msg)


def main():
    print("Controller On")
    rclpy.init(args=None)

    node = BucketController()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
