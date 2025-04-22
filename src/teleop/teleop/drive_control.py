import rclpy
import time
from rclpy.node import Node, Publisher
from sensor_msgs.msg import Joy
from std_msgs.msg import UInt8
from can import Message, Bus
from rcl_interfaces.msg import ParameterDescriptor
from .controls_lib import negativeNumberHandler, pubDelay, joyStickToUInt8


class DriveController(Node):
    def __init__(self, deadband, speedLimit):
        super().__init__(node_name="drive_publisher")
        self.declare_parameter("deadband", deadband)
        self.declare_parameter("speed_limit", speedLimit)

        self.dt_l_pub = self.create_publisher(UInt8, "dt_l_pub", 10)
        self.dt_r_pub = self.create_publisher(UInt8, "dt_r_pub", 10)

        self.deadband = self.get_parameter("deadband").value
        self.speed_limit = self.get_parameter("speed_limit").value
        self.get_logger().info(
            f"Deadband: {self.deadband}, Speed Limit: {self.speed_limit}"
        )

    def driveTrain(self, msg: Joy):
        uint8 = UInt8()

        # Left Stick Maps - Left Drive Train
        # Don't Touch this, it works
        # 0-100 is forward, 100-200 is backward
        # if msg.axes[1] > self.deadband:
        #     uint8.data = int(msg.axes[1] * self.speed_limit)
        # elif msg.axes[1] < -self.deadband:
        #     uint8.data = int(abs(msg.axes[1]) * self.speed_limit + 100)
        #     uint8.data = 0 if uint8.data == 100 else uint8.data
        # else:
        #     uint8.data = 0  # deadband resets it to neutral
        self.dt_l_pub.publish(
            joyStickToUInt8(msg.axes[1], self.deadband, self.speed_limit)
        )  # subtract 1 to no include 100)

        # Right Stick Maps - Right Drive Train
        # Don't Touch this either, it works
        # 0-100 is forward, 100-200 is backward
        # if msg.axes[3] > self.deadband:
        #     uint8.data = int((msg.axes[3] * self.speed_limit))
        # elif msg.axes[3] < -self.deadband:
        #     uint8.data = int((abs(msg.axes[3]) * self.speed_limit) + 100)
        #     uint8.data = 0 if uint8.data == 100 else uint8.data
        # else:
        #     uint8.data = 0  # deadband resets it to neutral
        # self.dt_r_pub.publish(uint8)
        self.dt_r_pub.publish(
            joyStickToUInt8(msg.axes[3], self.deadband, self.speed_limit)
        )

    def listener_callback(self, msg: Joy):
        # self.driveTrain(msg)
        pass


def main():
    print("Controller On")
    rclpy.init(args=None)

    node = DriveController()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
