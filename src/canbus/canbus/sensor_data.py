import can
import rclpy
import serial
import struct
from rclpy.node import Node
from std_msgs.msg import Float32, UInt8, String
from sensor_msgs.msg import BatteryState
from Phidget22.Phidget import*
from Phidget22.Devices.CurrentInput import*

class sensor_data(Node):
    def __init__(self):
        super().__init__('sensor_data')
        self.bus = can.interface.Bus(interface='socketcan', channel='vcan0', bitrate='500000')
        self.timer = self.create_timer(1, self.currentMod)
        self.timer = self.create_timer(1, self.bmsMod)
        # for current sensor
        self.publisher = self.create_publisher(
			UInt8,
            'ex_2_pub',
            10)
        # for Daly BMS
        self.publisher = self.create_publisher(
            BatteryState,
            'batt',
            10)
        # Phidget Connections
        self.currentInput0 = CurrentInput()
        self.currentInput0.openWaitForAttachment(5000)
        # BMS Connection
        self.serial_port = serial.Serial('/dev/serial0', baudrate=5000, timeout=1)

    #def data_conversion(self, data: Float32) -> UInt8

    def can_publish(self, data) -> None:
        can_msg = can.Message(
                data = data
                ) 
        self.bus.send(can_msg)

    def currentMod(self):
        currentValue = self.currentInput0.getCurrent()
        # create current phidget channel
        # self.get_logger().info(f'{currentValue}')
         
        # send stop
        if currentValue > 0.67: # value after method is the max current
            rosMsg = UInt8()
            rosMsg.data = 0
            self.publisher.publish(rosMsg.data)

    # main func of BMS to get data from BMS (every 1 sec) and pass it as a ROS msg
    def bmsMod(self):
        # according to protocol requests voltage, current, & batt percentage data
        frame = bytearray([0xA5, 0x03, 0x00])
        # arranges the data into 8 bits
        checksum = sum(frame) & 0xFF
        frame.append(checksum)
        # sends the frame over UART
        self.serial_port.write(frame)
        data = self.serial_port.read(13)

        # if statement to unpack data 
        if len(data) >= 13 and data[0] == 0xA5:
             voltage = struct.unpack('>H', data[4:6])[0] / 10
             current = struct.unpack('>h', data[6:8])[0] / 10
             battper = data[8]

            # arrange data into BatteryState message
             msg = BatteryState()
             msg.voltage = voltage
             msg.current = current
             msg.percentage = battper / 100 # for percentage sake?

             self.publisher.publish(msg)
        else:
             print('no BMS response lol')

def main(args=None):
        rclpy.init(args=args)
        node = sensor_data()
        rclpy.spin(node)

        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()