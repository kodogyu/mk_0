"""

    로봇의 자세를 제어하는 노드.
    image_processor 노드에서 물체를 검출하고 물체의 중심 좌표(center_point)를 줄 때마다 이 노드가 1번씩 실행.

    1. 검출된 물체의 중심 좌표와 카메라의 FOV간의 관계를 통해 물체의 방향 계산.
    2. 물체를 정면으로 바라보도록 회전. (일부분 가려져 있는 물체인 경우 회전했을 때 완전히 가려질 수도?)
    3. 물체와 일정거리가 되도록 직선 이동.
    4. 목적지에 도달해 있으면 정지.

"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

import math

class PositionController(Node):

    def __init__(self):
        super().__init__('position_control_mk_0')
        self.group = MutuallyExclusiveCallbackGroup()

        self.center_point_sub = self.create_subscription(Point, "/center_point", self.center_point_callback, 10)    # 물체 중심좌표
        self.lidar_sub = self.create_subscription(LaserScan, "/scan", self.lidar_callback, 10)  # 라이다 센서 값
        self.vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)   # 로봇 이동 속도

        self.direction = 0
        self.distance = 0
        self.min_distance = 0
        self.get_logger().info('Initializing done.')

    def calc_direction(self, x):
        # 방향 계산
        image_width = 1920  # waffle: 1920x1080 (turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_waffle/model.sdf:376)
        fov = 58.99    # 1.02974 rad = 58.99976 deg
        self.direction = (x - image_width/2)*fov / image_width # 좌측(- deg), 중앙(0 deg), 우측(+ deg)
        self.get_logger().info(f'Direction: {self.direction} (deg)')
        self.get_logger().info(f"Interest distance: {self.distance} (m)")
        
    def center_point_callback(self, msg):
        """
            center_point 토픽을 받을 때마다 수행.
        """
        self.get_logger().info(f'Got a center point: ({msg.x}, {msg.y})')
        
        # 물체의 방향 계산
        # self.direction에 반영
        self.calc_direction(msg.x)

        # 물체가 일정 범위안에 없을 경우 이동
        twist_msg = Twist()
        delta = msg.x - 960 # 물체의 중심이 정중앙에서 벗어나있는 정도 (픽셀). 960 = 이미지 넓이의 절반 (1920/2)
        direc_thresh = 10   # 방향 허용 범위 (pxl). 정중앙에서 +-10 픽셀까지는 중앙으로 봄.
        distance_thresh = 0.5   # 물체와의 목표 거리 (m).

        # 회전 운동
        if not (-direc_thresh <= delta <= direc_thresh) :
            """
                물체를 중앙에 위치시키려면 angular velocity는 delta와 부호가 반대여야 한다.
                ex) 물체가 좌측(delta < 0)에 있으면 반시계방향(ang_vel > 0)으로 이동해야 함.

                회전이 너무 빠르면 image_processor가 이미지를 처리하기 전에 화면에서 물체가 벗어난다.
                그래도 바깥 영역에 있을수록 빨리 중앙에 도달할 수 있도록 ang_vel을 정했다.
                ang_vel = -(delta / |delta|^0.5) * (1 / 960)
                960 = 이미지 넓이의 절반 (1920/2)
            """
            twist_msg.angular.z = float((-delta / math.sqrt(abs(delta))) / 960) # 회전 속도 (rad/s).
            self.get_logger().info(f"delta: {delta}, angular velocity: {twist_msg.angular.z}")
        else :
            self.get_logger().info("The Object is Centered.")

        # 직선 운동
        if self.min_distance > distance_thresh:  
            self.get_logger().info(f"Minimum distance: {self.min_distance} (m)")
            twist_msg.linear.x = 0.15    # 직선 이동 속도 (m/s)
        else:
            self.get_logger().info(f"The Object is close enough. ({self.min_distance} (m))")

        # Publish
        self.vel_publisher.publish(twist_msg)        

    def lidar_callback(self, msg):
        """
            direction = 0 -> msg.ranges[0]
            direction = -10 -> msg.ranges[10]
            direction = +10 -> msg.ranges[350]
        """
        range_idx = int((360 - self.direction) % 360)   # 관심 있는 방향. (deg)
        self.distance = msg.ranges[range_idx]   # 관심 있는 방향과의 거리.
        self.min_distance = min(msg.ranges) # 가장 가까운 물체와의 거리.

        # stop if something is too close (10cm)
        if self.min_distance < 0.01:
            self.get_logger().info(f"Something is Too Close!!!. ({self.min_distance} (m))")
            self.vel_publisher.publish(Twist())

        
def main(args=None):
    rclpy.init(args=args)
    
    position_controller = PositionController()

    rclpy.spin(position_controller)

    position_controller.destroy_node()
    rclpy.shutdown()   

if __name__ == "__main__":
    main()