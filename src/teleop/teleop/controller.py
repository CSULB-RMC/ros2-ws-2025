import rclpy
import time
from rclpy.node import Node, Publisher
from sensor_msgs.msg import Joy
from std_msgs.msg import UInt8
from can import Message, Bus
from rcl_interfaces.msg import ParameterDescriptor


class JoyPub(Node):
    def __init__(self):
        super().__init__("controller_publisher")
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
                    1,
                    ParameterDescriptor(
                        description="Controls Drive Train max speed (Percentage)"
                    ),
                ),
            ],
        )

        self.dt_l_pub = self.create_publisher(UInt8, "dt_l_pub", 10)
        self.dt_r_pub = self.create_publisher(UInt8, "dt_r_pub", 10)
        self.trap = self.create_publisher(UInt8, "trap", 10)
        self.ex_2_pub = self.create_publisher(UInt8, "ex_2_pub", 10)
        self.ex_1_pub = self.create_publisher(UInt8, "ex_1_pub", 10)
        self.dig_pub = self.create_publisher(UInt8, "dig_pub", 10)
        self.subscription = self.create_subscription(
            Joy, "joy", self.listener_callback, 10
        )

        self.off = False
        self.deadband = self.get_parameter("deadband").value
        self.speed_limit = self.get_parameter("speed_limit").value
        self.get_logger().info(
            f"Deadband: {self.deadband}, Speed Limit: {self.speed_limit}"
        )
        self.bucketSpeed = 0
        self.liftspeed = 0
        self.descendSpeed = 0

    def pubDelay(self, publisher: Publisher, data, delay: float):
        publisher.publish(data)
        time.sleep(delay)

    def listener_callback(self, msg: Joy):
        if self.off:
            return
        uint8 = UInt8()
        # the (back) button turns the whole machine off
        if msg.buttons[8] == 1:
            self.off = True
            uint8.data = 0
            self.dig_pub.publish(uint8)
            self.ex_1_pub.publish(uint8)
            self.ex_2_pub.publish(uint8)
            self.dt_l_pub.publish(uint8)
            self.dt_r_pub.publish(uint8)

        # Left Stick Maps - Left Drive Train
        # Don't Touch this, it works
        # 0-100 is forward, 100-200 is backward
        if msg.axes[1] > self.deadband:
            uint8.data = int(msg.axes[1] * self.speed_limit)
        elif msg.axes[1] < -self.deadband:
            uint8.data = int(abs(msg.axes[1]) * self.speed_limit + 100)
            uint8.data = 0 if uint8.data == 100 else uint8.data
        else:
            uint8.data = 0  # deadband resets it to neutral
        self.dt_l_pub.publish(uint8)

        # Right Stick Maps - Right Drive Train
        # Don't Touch this either, it works
        # 0-100 is forward, 100-200 is backward
        if msg.axes[3] > self.deadband:
            uint8.data = int((msg.axes[3] * self.speed_limit))
        elif msg.axes[3] < -self.deadband:
            uint8.data = int((abs(msg.axes[3]) * self.speed_limit) + 100)
            uint8.data = 0 if uint8.data == 100 else uint8.data
        else:
            uint8.data = 0  # deadband resets it to neutral
        self.dt_r_pub.publish(uint8)

        if msg.buttons[0] == 1:
            uint8.data = 100
            self.trap.publish(uint8)
        elif msg.buttons[0] == 0:
            uint8.data = 0
            self.trap.publish(uint8)

        if msg.buttons[1] == 1:
            pass

        if msg.buttons[2] == 1:
            pass

        if msg.buttons[3] == 1:
            pass

        # i don't know if this is correct?
        if msg.buttons[3] == 1:  # digging High (Left bumper)
            if self.bucketSpeed >= 10:
                self.bucketSpeed -= 10
                uint8.data = self.bucketSpeed
                self.dig_pub.publish(uint8)
            time.sleep(0.5)

        # Stage 1 Speed Control
        if msg.buttons[5] == 1 and self.liftspeed >= 10:
            self.liftspeed -= 10
            uint8.data = self.liftspeed
            self.pubDelay(self.ex_1_pub, uint8, 0.5)

        if msg.buttons[7] == 1 and self.liftspeed <= 90:
            self.liftspeed += 10
            uint8.data = self.liftspeed
            self.pubDelay(self.ex_1_pub, uint8, 0.5)

        # Stage 2 Speed Control
        if msg.buttons[4] == 1 and self.descendSpeed >= 10:
            self.descendSpeed -= 10
            uint8.data = self.descendSpeed
            self.pubDelay(self.ex_2_pub, uint8, 0.5)

        if msg.buttons[6] == 1 and self.descendSpeed <= 90:
            self.descendSpeed += 10
            uint8.data = self.descendSpeed
            self.pubDelay(self.ex_2_pub, uint8, 0.5)


def main():
    print("Controller On")
    rclpy.init(args=None)

    node = JoyPub()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
