import rclpy
from rclpy.node import Node, Publisher
from std_msgs.msg import UInt8
import time


def negativeNumberHandler(x: int):
    if x < 0:
        x = abs(x) + 100
    return x


def pubDelay(publisher: Publisher, data, delay: float):
    publisher.publish(data)
    time.sleep(delay)


def joyStickToUInt8(axes: float, deadband: float, speedLimit: int):
    uint8 = UInt8()
    if axes > deadband:
        uint8.data = int(axes * speedLimit)
    elif axes < -deadband:
        uint8.data = int(abs(axes) * speedLimit + 100)
        uint8.data = 0 if uint8.data == 100 else uint8.data
    else:
        uint8.data = 0  # deadband resets it to neutral
    return uint8
