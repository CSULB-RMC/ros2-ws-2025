import can
import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String, UInt8
from .vesc import Vesc

class motor_controller(Node):
    # remember bit conversions are INT16 - Big Endian (AB)
    def __init__ (self):
        super().__init__('motor_controller')
        # can bus yipie (switch channel name to 'vcan0' for virtual can testing)
        self.bus = can.interface.Bus(interface='socketcan', channel='vcan0', bitrate='500000')
        topic_list = {'dt_l_pub', 'dt_r_pub', 'dig_pub'}
        for topic in topic_list:
             self.create_subscription(
                  UInt8, 
                  topic,
                  lambda msg,
                  topic = topic:
                  self.listener_callback(msg, topic),
                  10)
             
    def id_conversion(self, device_id: int, command_id: int)->int:
        return (command_id << 8) | device_id

    def can_publish(self, arbitration_id, data, is_extended_id) -> None:
        can_msg = can.Message(
                arbitration_id = arbitration_id,
                data = data, 
                is_extended_id = is_extended_id
                ) 
        self.bus.send(can_msg)    

    def listener_callback(self, msg: UInt8, topic: String):
        print(topic, msg.data)

        # VESCs
        if topic == 'dt_l_pub':
            self.can_publish(self.id_conversion(15, 3), Vesc.signal_conversion(msg.data, 4, 1), True)
            self.can_publish(self.id_conversion(16, 3), Vesc.signal_conversion(msg.data, 4, 1), True)

        elif topic == 'dt_r_pub':
            self.can_publish(self.id_conversion(17, 3), Vesc.signal_conversion(msg.data, 4, 1), True)
            self.can_publish(self.id_conversion(18, 3), Vesc.signal_conversion(msg.data, 4, 1), True)

        # STMs
        elif topic == 'dig_pub':
            self.can_publish(self.id_conversion(30, 0), Vesc.signal_conversion(msg.data, 4, 1), True)

def main(args=None):
        rclpy.init(args=args)
        node = motor_controller()
        rclpy.spin(node)

        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()