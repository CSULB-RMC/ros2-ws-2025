import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import Joy 
from std_msgs.msg import UInt8
from can import Message, Bus

class JoyPub(Node):
    def __init__(self):
        super().__init__('controller_publisher')
        self.dt_l_pub = self.create_publisher(UInt8, 'dt_l_pub', 10)
        self.dt_r_pub = self.create_publisher(UInt8, 'dt_r_pub', 10)
        self.subscription = self.create_subscription(Joy, 'joy', self.listener_callback, 10)
        self.declare_parameters(
            namespace='',
            parameters=[
                ('deadzone', None),
                ('speed_limit', None)
            ]
        )
        # 100 = no speed
    def listener_callback(self, msg:Joy):
        uint8 = UInt8()
        if msg.axes[1] > self.deadband:
            uint8.data = int(-msg.axes[1]*self.speed_limit + 100)
            self.dt_l_pub.publish(uint8)
        elif msg.axes[1] < -self.deadband:
            uint8.data = int(-msg.axes[1]*self.speed_limit + 100)
            self.dt_l_pub.publish(uint8)
        else:
            uint8.data = 100
            self.dt_l_pub.publish(uint8)

        if msg.axes[3] > self.deadband:
            uint8.data = int(msg.axes[3]*self.speed_limit + 100)
            self.dt_r_pub.publish(uint8)
        elif msg.axes[3] < -self.deadband:
            uint8.data = int(msg.axes[3]*self.speed_limit+ 100)
            self.dt_r_pub.publish(uint8)
        else:
            uint8.data = 100
            self.dt_r_pub.publish(uint8)

def main():
    print("Controller On")
    rclpy.init(args=None)

    node = JoyPub()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
