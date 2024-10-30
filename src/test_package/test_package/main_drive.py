import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import UInt8, String
#ignore can import error if it's there, it works if you installed python-can
import can
from .vesc import Vesc as vesc


class DrivetrainExcavator(Node):
    
    def __init__(self):
        super().__init__('drivetrain_ex')
        self.activity_publisher_ = self.create_publisher(String, 'ex_active', 10) 
        self.status_timer = self.create_timer(0.1, self.timer_callback)

        # create subscribers to listen for teleop computer commands
        self.ex_dt_left_sub = self.create_subscription(UInt8, 'ex_dt_left', self.ex_dt_left_update, 10)
        self.ex_dt_right_sub = self.create_subscription(UInt8, 'ex_dt_right', self.ex_dt_right_update, 10)
        self.ex_conveyer_sub = self.create_subscription(UInt8, 'ex_conveyer', self.ex_conveyer_update, 10)
        self.ex_arm_sub = self.create_subscription(UInt8, 'ex_arm', self.ex_arm_update, 10)
        self.ex_digger_sub = self.create_subscription(UInt8, 'ex_digger', self.ex_digger_update, 10)
        self.ex_servo_sub = self.create_subscription(UInt8, 'ex_servo', self.ex_servo_update, 10)

        # create state variables, these keep track of what motors
        # should be running and how fast at the current moment
        self.ex_dt_left_speed = 0
        self.ex_dt_right_speed = 0
        self.ex_conveyer_speed = 0
        self.ex_arm_speed = 0
        self.ex_digger_speed = 0
        self.ex_servo_speed = 0

        # create can bus link, right now is linked to virtual vcan 0, most likely
        # will be can0 when on the bot
        self.bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate='500000')

        self.i = 0
    
    def timer_callback(self):
        s = String()
        s.data = "Alive"
        self.activity_publisher_.publish(s)

    def can_publish(self, arbitration_id, data, is_extended_id) -> None:
        can_msg = can.Message(
                arbitration_id = arbitration_id,
                data = data, 
                is_extended_id = is_extended_id
                ) 
        self.bus.send(can_msg)

    #updates the states of the left drivetrain motors
    def ex_dt_left_update(self, msg):
        # checks if speed is different then previous message published
        if self.ex_dt_left_speed == msg.data:     
            return None
        self.ex_dt_left_speed = msg.data


    #updates the states of the right drivetrains motors
    def ex_dt_right_update(self, msg):
        # checks if speed is different then previous message published
        if self.ex_dt_right_speed == msg.data:
            return None
        self.ex_dt_right_speed = msg.data
    
    # Conveyer Belt
    def ex_conveyer_update(self, msg):
        if self.ex_conveyer_speed == msg.data:
            return None
        self.ex_conveyer_speed = msg.data

        temp_data = vesc.signal_conversion(msg.data, 8, 10)

        self.can_publish(19, temp_data, True) 

    # Arm
    def ex_arm_update(self, msg):
        if self.ex_arm_speed == msg.data:
            return None
        self.ex_arm_speed = msg.data 
        temp_data = vesc.signal_conversion(msg.data, 8, 10)

        self.can_publish(21, temp_data, True)
    
    def ex_digger_update(self, msg):
        if self.ex_digger_speed == msg.data:
            return None
        self.ex_digger_speed = msg.data 
        temp_data = vesc.signal_conversion(msg.data, 8, 10)

        self.can_publish(23, temp_data, True)

    # Servo
    def ex_servo_update(self, msg):
        if self.ex_servo_speed == msg.data:
            return None
        self.ex_servo_speed = msg.data

        temp_data = vesc.signal_conversion(msg.data, 8, 100)

        self.can_publish(25, temp_data, True) 

    def timer_callback(self):
        temp_data = vesc.signal_conversion(self.ex_dt_left_speed, 4, 1000)  # Has to be 4 to work on vesc
        # can message for right and left motor
        self.can_publish(vesc.id_conversion(15, 3), temp_data, True)
        self.can_publish(vesc.id_conversion(16, 3), temp_data, True) 

        # converts controller signal to bytes array
        temp_data = vesc.signal_conversion(self.ex_dt_right_speed, 4, 1000)  # Has to be 4 to work on vesc
        self.can_publish(vesc.id_conversion(17, 3), temp_data, True)
        self.can_publish(vesc.id_conversion(18, 3), temp_data, True)

def main(args=None):
    print("Bus Publisher Active New")
    rclpy.init(args=args)
    node = DrivetrainExcavator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
