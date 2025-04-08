import rclpy
import time
from rclpy.node import Node
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
        self.bucketSpeed = 0
        self.liftspeed = 0
        self.descendSpeed = 0
        self.deadband = self.get_parameter("deadband").value
        self.speed_limit = self.get_parameter("speed_limit").value

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

            uint8.data = 100

            self.dt_l_pub.publish(uint8)
            self.dt_r_pub.publish(uint8)
        

        exec = False
        if msg.axes[1] > self.deadband:
            exec = True
            uint8.data = int(-msg.axes[1] * self.speed_limit + 100)
            self.dt_l_pub.publish(uint8)
        elif msg.axes[1] < -self.deadband:
            exec = True
            uint8.data = int(-msg.axes[1] * self.speed_limit + 100)
            self.dt_l_pub.publish(uint8)
        elif not exec:
            uint8.data = 100
            self.dt_l_pub.publish(uint8)
        exec = False
        if msg.axes[3] > self.deadband:
            exec = True
            uint8.data = int(msg.axes[3] * self.speed_limit + 100)
            self.dt_r_pub.publish(uint8)
        elif msg.axes[3] < -self.deadband:
            exec = True
            uint8.data = int(msg.axes[3] * self.speed_limit + 100)
            self.dt_r_pub.publish(uint8)
        elif not exec:
            uint8.data = 100
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
        if msg.buttons[4] == 1:  # digging High (Left bumper)
            if self.bucketSpeed >= 10:
                self.bucketSpeed -= 10
                uint8.data = self.bucketSpeed
                self.dig_pub.publish(uint8)
            time.sleep(0.5)

        # Button 5 (right bumper)
        if msg.buttons[5] == 1:
            self.liftspeed += 10
            uint8.data = self.liftspeed
            self.ex_1_pub.publish(uint8)
            time.sleep(1)

        if msg.buttons[6] == 1:  # diggin low  (Left trigger)
            self.bucketSpeed += 10
            uint8.data = self.bucketSpeed
            self.dig_pub.publish(uint8)
            time.sleep(0.8)

        # Excavation Stage 2 Button (right trigger)
        if msg.buttons[7] == 1:
            self.descendSpeed = 30  # placeholder
            uint8.data = self.descendSpeed
            self.ex_2_pub.publish(uint8)


def main():
    print("Controller On")
    rclpy.init(args=None)

    node = JoyPub()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
