import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String, UInt8, Bool, Int32

# This program handles translating the controller inputs from the driver station to be published to the onboard computer on the robot.
class MiniJoy(Node):
    def __init__(self):
        super().__init__('mini_joy')

        # Publishers for the left and right drivetrain motors
        self.dt_l_publisher_ = self.create_publisher(UInt8, 'dt_left', 10)
        self.dt_r_publisher_ = self.create_publisher(UInt8, 'dt_right', 10)
        self.bmo_face_ = self.create_publisher(Int32, 'bmo_animation', 10)


        # Intakes the controller inputs from the ros joy package
        self.subscription = self.create_subscription(
			Joy,
			'joy',
			self.listener_callback,
			10)

        # Constants
        self.speed_limit_left = 7.9 * 2 # Scalar value for keeping the bot in the right speed range
        self.speed_limit_right = 10 * 2
        self.DEADBAND = 0.05 #the size of the deadband for the controller to ignore inputs

    def listener_callback(self, msg: Joy):
        uint8 = UInt8()

        if msg.axes[1] > self.DEADBAND: # L Stick Up 
            uint8.data = 50 + int(msg.axes[1] * self.speed_limit_left) # add 100 to indicate forward motion and not include 100
            self.dt_l_publisher_.publish(uint8)
        elif msg.axes[1] < -self.DEADBAND: # L Stick Down 
            uint8.data = 50 - int(abs(msg.axes[1]) * self.speed_limit_left) # subtract 1 to no include 100 
            self.dt_l_publisher_.publish(uint8)
        else:
            uint8.data = 50 # deadband resets it to neutral
            self.dt_l_publisher_.publish(uint8)

        # Right Stick Maps - Right Drive Train
        if msg.axes[3] > self.DEADBAND: # R Stick Up
            uint8.data = 50 - int((msg.axes[3] * self.speed_limit_right)) 
            self.dt_r_publisher_.publish(uint8)
        elif msg.axes[3] < -self.DEADBAND: # R Stick Down
            uint8.data = 50 + int((abs(msg.axes[3]) * self.speed_limit_right)) 
            self.dt_r_publisher_.publish(uint8)
        else:
            uint8.data = 50 # deadband resets it to neutral
            self.dt_r_publisher_.publish(uint8)    

        if msg.buttons[0]:
            self.bmo_face_.publish(0)
        elif msg.buttons[1]:
            self.bmo_face_.publish(1)
        elif msg.buttons[2]:
            self.bmo_face_.publish(2)
        elif msg.buttons[3]:
            self.bmo_face_.publish(3)

def main(args=None):
    print("Joystick Live")
    rclpy.init(args=args)

    mini_joy = MiniJoy()

    rclpy.spin(mini_joy)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mini_joy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
