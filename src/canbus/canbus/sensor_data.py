import can
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, UInt8
from Phidget22.Phidget import*
from Phidget22.Devices.CurrentInput import*

class sensor_data(Node):
    def __init__(self):
        super().__init__('sensor_data')
        self.bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate='500000')
        self.timer = self.create_timer(1, self.currentMod)
        # for current sensor
        self.publisher = self.create_publisher(
			Float32,
            'ex_2_pub',
            10)

    #def data_conversion(self, data: Float32) -> UInt8

    def can_publish(self, data) -> None:
        can_msg = can.Message(
                data = data
                ) 
        self.bus.send(can_msg)

    def currentMod(self):
        # create current phidget channel
        currentInput0 = CurrentInput()
         
        # send stop
        if currentInput0.getCurrent() > 1: # value after method is the max current
            rosMsg = UInt8()
            rosMsg.data = 0
            self.publisher.publish(rosMsg.data)

def main(args=None):
        rclpy.init(args=args)
        node = sensor_data()
        rclpy.spin(node)

        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()