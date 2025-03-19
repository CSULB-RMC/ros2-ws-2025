import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class CameraPublisher(Node):
    def __init__(self):
        super().__init__("camera_publisher")
        
        self.publisher_ = self.create_publisher(Image, "image", 10)
        
        self.cap = cv2.VideoCapture(0)  
        self.bridge = CvBridge()
        
        self.timer = self.create_timer(0.1, self.publish_frame)  
        self.get_logger().info("Camera Publisher Node Started")

    def publish_frame(self):
        ret, frame = self.cap.read()
        if ret:
            ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.publisher_.publish(ros_image)
        else:
            print('failed')
    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
