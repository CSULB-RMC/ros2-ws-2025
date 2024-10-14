import rclpy
from rclpy.node import Node
import can


class CanTest(Node):

    def __init__(self):
        super().__init__('can_test')
        self.status_timer = self.create_timer(0.1, self.timer_callback)

        self.bus = can.interface.Bus(interface='socketcan', channel='vcan0', bitrate='500000')
    
    def can_publish(self, arbitration_id, data, is_extended_id) -> None:
        can_msg = can.Message(
                arbitration_id = arbitration_id,
                data = data, 
                is_extended_id = is_extended_id
                ) 
        self.bus.send(can_msg)

    def timer_callback(self):
        bytes_range = 8
        frequency_floor = 10
        data: int = 65
        temp_data: list[int] = []

        #make sure the direction is correct
        if data > 100:
            #increment for 2's comp
            c = 1
            # Forward msg correction:
            data -= 100
            # covert controller signal to proper range (1000-100000)
            data *= frequency_floor

            #convert to byte array but also 2's compliment to reverse motor
            for i in range(bytes_range - 1, -1, -1):
                temp_data.append(255 - ((data >> (8*i)) & 0xff))

            for i in range(len(temp_data) - 1, - 1, -1):
                temp_data[i] += c 
                if temp_data[i] > 255:
                    temp_data[i] = 0
                else:
                    c = 0
                    break
        else:
            # covert controller signal to proper range (1000-100000)
            data *= frequency_floor
    
            # convert signal to byte array
            for i in range(bytes_range - 1, -1, -1):
                temp_data.append((data >> (8*i)) & 0xff)

        self.can_publish(19, temp_data, True)

def main(args=None):
    rclpy.init(args=args)

    can_test = CanTest()

    rclpy.spin(can_test)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    can_test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()