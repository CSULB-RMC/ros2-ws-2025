import rclpy
import time
from rclpy.node import Node, Publisher
from sensor_msgs.msg import Joy
from std_msgs.msg import UInt8
from can import Message, Bus
from rcl_interfaces.msg import ParameterDescriptor
from .controls_lib import negativeNumberHandler, pubDelay, joyStickToUInt8


class LadderController(Node):
    def __init__(self, deadband, liftSpeed, descendSpeed):
        super().__init__("ladder_publisher")
        self.declare_parameter("deadband", deadband)
        self.declare_parameter("lift_speed_limit", liftSpeed)
        self.declare_parameter("descend_speed_limit", descendSpeed)

        self.ex_1_pub = self.create_publisher(UInt8, "ex_1_pub", 10)
        self.ex_2_pub = self.create_publisher(UInt8, "ex_2_pub", 10)

        self.deadband = self.get_parameter("deadband").value
        self.liftSpeedLimit = self.get_parameter("lift_speed_limit").value
        self.descendSpeedLimit = self.get_parameter("descend_speed_limit").value
        self.get_logger().info(
            f"Deadband: {self.deadband}, Lift Speed Limit: {self.liftSpeedLimit}, Descend Speed Limit: {self.descendSpeedLimit}"
        )

    def ladder(self, msg: Joy):
        self.ex_1_pub.publish(
            joyStickToUInt8(msg.axes[1], self.deadband, self.liftSpeedLimit)
        )
        self.ex_2_pub.publish(
            joyStickToUInt8(msg.axes[3], self.deadband, self.descendSpeedLimit)
        )

    def listener_callback(self, msg: Joy):
        # self.ladder(msg)
        pass


def main():
    print("Controller On")
    rclpy.init(args=None)

    node = LadderController()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
