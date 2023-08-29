"""

    카메라 이미지에서 물체를 검출하는 노드.

    1. 노드가 실행될 때 detection 모델을 로드.
    2. Object detection을 계속 수행.
    3. 찾는 물체가 발견되면 로봇 정지.
    4. 물체의 중심좌표를 publish.

"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point

from cv_bridge import CvBridge
import cv2
import numpy as np
from roboflow import Roboflow

class ImageProcessor(Node):

    def __init__(self):
        super().__init__('image_processor_mk_0')
        self.subscription = self.create_subscription(Image, "/camera/image_raw", self.image_callback, 10)
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)   # for stop movement
        self.center_publisehr = self.create_publisher(Point, '/center_point', 10)   # for position_controller node

        # Roboflow initialization
        # rf = Roboflow(api_key="zPJlUOT7OWO79KWnxFwc") # 콘 인식
        # project = rf.workspace().project("cone-9mpfj")
        # self.model = project.version(1).model

        rf = Roboflow(api_key="gKERgYX4nFZ0IZAQJt2c")   # 소화기 인식
        project = rf.workspace("project-rbclj").project("fire-t4aw2")
        # dataset = project.version(3).download("yolov5")
        self.model = project.version(3).model
        
        
    # Image callback function
    def image_callback(self, msg):
        # ROS Image message to OpenCV format
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Convert to grayscale
        cv_image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Perform object detection
        self.get_logger().info('Got an Image. Processing...')
        result = self.model.predict(cv_image_gray, confidence=40, overlap=30)
        if result:
            self.get_logger().info('Object Detected!')

            # Stop the robot by sending a stop command to /cmd_vel topic
            self.vel_publisher.publish(Twist())
            self.get_logger().info('Stopping.')
        
            # Draw bounding boxes and labels on the image
            for prediction in result:
                x, y, width, height = int(prediction["x"]), int(prediction["y"]), int(prediction["width"]), int(prediction["height"])
                class_name = prediction["class"]
                confidence = prediction["confidence"]
                print(f"x:{x}, y:{y}, w:{width}, h:{height}")
                # Draw bounding box
                cv2.rectangle(image, (x-110, y-205), (x + width-110, y + height-205), (0, 255, 0), 2)
                # Draw a circle
                cv2.circle(image, (x, y), 4, color=(0,0,255), thickness=2)
                
                # Add class name and confidence as text
                label = f"{class_name}: {confidence:.2f}"
                cv2.putText(image, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Publish a center point topic
            center_point = Point()
            center_point.x = float(x)
            center_point.y = float(y)
            self.center_publisehr.publish(center_point)
            self.get_logger().info(f'Sent a center point: ({center_point.x}, {center_point.y})')
        
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