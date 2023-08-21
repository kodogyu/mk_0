import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

from cv_bridge import CvBridge
import cv2
import numpy as np
from roboflow import Roboflow

class ImageProcessor(Node):

    def __init__(self):
        super().__init__('image_processor_mk_0')
        self.subscription = self.create_subscription(Image, "/camera/image_raw", self.image_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Roboflow initialization
        rf = Roboflow(api_key="zPJlUOT7OWO79KWnxFwc")
        project = rf.workspace("khu-id7cs").project("cone-9mpfj")
        self.model = project.version(1).model
        
    # Image callback function
    def image_callback(self, msg):
        # ROS Image message to OpenCV format
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Convert to grayscale
        cv_image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Perform object detection
        result = self.model.predict(cv_image_gray, confidence=40, overlap=30)
        if result:
            print("result found!")
            # Stop the robot by sending a stop command to /cmd_vel topic
            stop_cmd = Twist()
            stop_cmd.linear.x = 0.0
            stop_cmd.angular.z = 0.0
            self.publisher.publish(stop_cmd)
        
            # Draw bounding boxes and labels on the image
            for prediction in result:
                x, y, width, height = int(prediction["x"]), int(prediction["y"]), int(prediction["width"]), int(prediction["height"])
                class_name = prediction["class"]
                confidence = prediction["confidence"]
                print(f"x:{x}, y:{y}, w:{width}, h:{height}")
                # Draw bounding box
                cv2.rectangle(image, (x-110, y-205), (x + width-110, y + height-205), (0, 255, 0), 2)
                
                # Add class name and confidence as text
                label = f"{class_name}: {confidence:.2f}"
                cv2.putText(image, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Display the image with bounding boxes
        cv2.namedWindow("ImageWindow2", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("ImageWindow2", 1000, 800)
        cv2.imshow("ImageWindow2", image)
        
        cv2.waitKey(1)
        # Draw bounding boxes and labels on the image

def main(args=None):
    # Initialize ROS node
    rclpy.init(args=args)

    image_processor = ImageProcessor()

    rclpy.spin(image_processor)

    image_processor.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()