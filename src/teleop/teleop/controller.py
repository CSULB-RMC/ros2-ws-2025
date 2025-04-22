import rclpy
import time
from rclpy.node import Node, Publisher
from sensor_msgs.msg import Joy
from std_msgs.msg import UInt8
from can import Message, Bus
from rcl_interfaces.msg import ParameterDescriptor
from .controls_lib import negativeNumberHandler, pubDelay
from .drive_control import DriveController
from .ladder_control import LadderController
from .dig_control import BucketController
from .trapdoor import TrapDoor
from rclpy.parameter import Parameter


class JoyPub(Node):
    def __init__(self):
        super().__init__("controller")
        self.declare_parameters(
            namespace="",
            parameters=[
                (
                    "deadband",
                    0.05,
                    ParameterDescriptor(
                        description="Controls joystick senesitivity to start reading data"
                    ),
                ),
                (
                    "speed_limit",
                    0,
                    ParameterDescriptor(
                        description="Controls Drive Train max speed (Percentage)"
                    ),
                ),
                (
                    "bucket_speed_limit",
                    0,
                    ParameterDescriptor(
                        description="Controls Drive Train max speed (Percentage)"
                    ),
                ),
                (
                    "lift_speed_limit",
                    0,
                    ParameterDescriptor(
                        description="Controls Drive Train max speed (Percentage)"
                    ),
                ),
                (
                    "descend_speed_limit",
                    0,
                    ParameterDescriptor(
                        description="Controls Drive Train max speed (Percentage)"
                    ),
                ),
            ],
        )

        self.subscription = self.create_subscription(
            Joy, "joy", self.listener_callback, 10
        )

        self.off = False
        self.deadband = self.get_parameter("deadband").value
        self.speed_limit = self.get_parameter("speed_limit").value
        self.bucketSpeedLimit = self.get_parameter("bucket_speed_limit").value
        self.liftSpeedLimit = self.get_parameter("lift_speed_limit").value
        self.descendSpeedLimit = self.get_parameter("descend_speed_limit").value
        self.get_logger().info(
            f"{self.deadband}, {self.speed_limit}, {self.bucketSpeedLimit}, {self.liftSpeedLimit}, {self.descendSpeedLimit}"
        )
        self.drive_controller = DriveController(
            deadband=self.deadband, speedLimit=self.speed_limit
        )
        self.ladder_controller = LadderController(
            deadband=self.deadband,
            liftSpeed=self.liftSpeedLimit,
            descendSpeed=self.descendSpeedLimit,
        )
        self.dig_controller = BucketController(bucketSpeedLimit=self.bucketSpeedLimit)
        self.trapdoor = TrapDoor()

        self.stage = {0: "standby", 1: "drive", 2: "ladder", 3: "dig"}
        self.state = 0

    def listener_callback(self, msg: Joy):
        if self.off:
            return

        # the (back) button turns the whole machine off
        if msg.buttons[8] == 1:
            self.off = True
            msg.axes = [0] * 6
            msg.buttons = [0] * 12
            self.drive_controller.driveTrain(msg)
            self.ladder_controller.ladder(msg)
            self.dig_controller.dig(msg)
            self.trapdoor.trapdoor(msg)

        # Cycle States
        if msg.buttons[9] == 1:
            self.state = (self.state + 1) % 4
            self.get_logger().info(f"State: {self.stage[self.state]}")

        match self.stage[self.state]:
            case "standby":
                msg.axes = [0] * 6
                msg.buttons = [0] * 12
                self.drive_controller.driveTrain(msg)
                self.ladder_controller.ladder(msg)
                self.dig_controller.dig(msg)
                self.trapdoor.trapdoor(msg)
            case "drive":
                self.drive_controller.driveTrain(msg)
            case "ladder":
                self.ladder_controller.ladder(msg)
            case "dig":
                self.dig_controller.dig(msg)
                self.trapdoor.trapdoor(msg)


def main():
    print("Controller On")
    rclpy.init(args=None)

    node = JoyPub()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
