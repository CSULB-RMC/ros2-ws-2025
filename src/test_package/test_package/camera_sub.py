import rclpy
from rclpy.node import Node
import cv2
import os
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from datetime import datetime

class ImageSaver(Node):
    def __init__(self):
        super().__init__("camera_sub")

        self.subscription = self.create_subscription(Image, "image", self.image_callback, 10)

        self.bridge = CvBridge()

        self.image_save_dir = os.path.expanduser("ros2/img")
        os.makedirs(self.image_save_dir, exist_ok=True)


    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            image_filename = os.path.join(self.image_save_dir, f"frame_{timestamp}.jpg")

            cv2.imwrite(image_filename, cv_image)

            print('wrote this file')
        except:
            print('failed to read img')
def main(args=None):
    rclpy.init(args=args)
    node = ImageSaver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
