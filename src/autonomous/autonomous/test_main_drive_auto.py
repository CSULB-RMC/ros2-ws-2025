import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import UInt8, String
import time
from src.test_package.test_package.vesc import Vesc as vesc
import can

class MainDriveAuto(Node):
    
    def __init__(self):
        super().__init__('main_drive_auto')

        #joy listener
        self.subscription = self.create_subscription(
			Joy,
			'joy',
			self.listener_callback,
			1)

        # create can bus link, right now is linked to virtual vcan 0, most likely
        # will be can0 when on the bot
        self.bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate='500000')

    def can_publish(self, arbitration_id, data, is_extended_id) -> None:
        can_msg = can.Message(
                arbitration_id = arbitration_id,
                data = data, 
                is_extended_id = is_extended_id
                ) 
        self.bus.send(can_msg)

    #Duty 0 is stop, 1-100 is forward, 101-200 is backwards. Time is in ms
    def drive_for_time(self, left_duty, right_duty, time_elapse):
        left_data = vesc.signal_conversion(left_duty, 4, 50)
        right_data = vesc.signal_conversion(right_duty, 4, 50)
        start_time = time.time()
        while((time.time() - start_time) < (time_elapse / 1000)):
            self.can_publish(vesc.id_conversion(15, 3), left_data, True)
            self.can_publish(vesc.id_conversion(16, 3), left_data, True)
            self.can_publish(vesc.id_conversion(17, 3), right_data, True)
            self.can_publish(vesc.id_conversion(18, 3), right_data, True)
            time.sleep(0.1)

            # hardstop at 3 seconds for safety since we are testing
            if((time.time() - start_time) > 3000):
                break
        # publish duty cycle of 0 to stop the motors
        stop = self.signal_conversion(0, 4, 1000)
        self.can_publish(15, stop, True)
        self.can_publish(16, stop, True)
        self.can_publish(17, stop, True)
        self.can_publish(18, stop, True)
        

    def listener_callback(self, msg):
        # X Button
        if msg.buttons[3]:
            # we are philosphers eating rice and the robot is the chopstick
            self.drive_for_time(10, 10, 2000)
            self.drive_for_time(0, 10, 1900)

def main(args=None):
    print("Main Drive Auto Mode")
    rclpy.init(args=args)
    node = MainDriveAuto()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
