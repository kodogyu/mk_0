"""

    로봇의 자세를 제어하는 노드.
    image_processor 노드에서 물체를 검출하고 신호를 주면 이 노드가 실행.

    1. 검출된 물체의 중심 좌표와 카메라의 FOV간의 관계를 통해 물체의 방향 계산.
    2. 물체를 정면으로 바라보도록 회전. (일부분 가려져 있는 물체인 경우 회전했을 때 완전히 가려질 수도?)
    3. 물체와 일정거리가 되도록 직선 이동.
    4. 목적지에 도달하면 노드를 대기 상태로 전환.

"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

import math

class PositionController(Node):

    def __init__(self):
        super().__init__('position_control_mk_0')
        self.group = MutuallyExclusiveCallbackGroup()

        self.center_point_sub = self.create_subscription(Point, "/center_point", self.center_point_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, "/scan", self.lidar_callback, 10)
        self.vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        self.pixel_x = 0
        self.direction = 0
        self.distance = 0
        self.get_logger().info('Initializing done.')

    # Not used.
    # 라이다, 카메라 위치를 평행하게 맞추고 난 후에 사용 가능
    def calc_direction(self, x):
        # 방향 계산
        image_width = 1920  # waffle: 1920x1080 (turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_waffle/model.sdf:376)
        fov = 58.99    # 1.02974 rad = 58.99976 deg
        self.direction = (x - image_width/2)*fov / image_width # 좌측(- deg), 중앙(0 deg), 우측(+ deg)
        self.get_logger().info(f'Direction: {self.direction} (deg)')
        
    def center_point_callback(self, msg):
        self.get_logger().info(f'Got a center point: ({msg.x}, {msg.y})')
        
        # 물체의 이미지 x 좌표
        self.pixel_x = msg.x

        # 물체가 일정 범위안에 없을 경우 이동
        twist_msg = Twist()
        if not (-10 <= self.pixel_x - 960 <= 10) :
            delta = self.pixel_x - 960
            twist_msg.angular.z = float((-delta / math.sqrt(abs(delta))) / 960)
            
            self.get_logger().info(f"delta: {self.pixel_x - 960}, angular velocity: {twist_msg.angular.z}")
            self.vel_publisher.publish(twist_msg)
        else :
            self.get_logger().info(f"The Object is Centered.")

    def lidar_callback(self, msg):
        # direction = 0 -> msg.ranges[0]
        # direction = -10 -> msg.ranges[10]
        # direction = +10 -> msg.ranges[350]
        range_idx = int((360 - self.direction) % 360)
        self.distance = msg.ranges[range_idx]

        
def main(args=None):
    rclpy.init(args=args)
    
    position_controller = PositionController()

    rclpy.spin(position_controller)

    position_controller.destroy_node()
    rclpy.shutdown()   

if __name__ == "__main__":
    main()