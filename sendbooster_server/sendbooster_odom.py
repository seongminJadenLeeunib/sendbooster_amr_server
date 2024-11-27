import rclpy
from rclpy.node import Node
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros

class RobotOdometry:
    def __init__(self, wheel_base, wheel_radius, encoder_resolution, motor_offset):
        self.x = 0
        self.y = 0
        self.theta = 0
        self.v = 0  # 선속도 초기화
        self.omega = 0  # 각속도 초기화
        self.wheel_base = wheel_base  # 휠 간 거리
        self.wheel_radius = wheel_radius  # 휠 반지름
        self.encoder_resolution = encoder_resolution  # 엔코더 해상도
        self.motor_offset = motor_offset / 100.0  # 모터 오프셋 (cm -> m)
        self.prev_left_encoder = 0
        self.prev_right_encoder = 0

    def update(self, left_encoder, right_encoder, delta_time):
        # 엔코더값의 차이 계산 (이전 엔코더 값과 현재 엔코더 값의 차이)
        delta_left = left_encoder - self.prev_left_encoder
        delta_right = right_encoder - self.prev_right_encoder

        # 왼쪽, 오른쪽 휠에서 이동한 거리 계산
        d_left = 2 * math.pi * self.wheel_radius * delta_left / self.encoder_resolution
        d_right = 2 * math.pi * self.wheel_radius * delta_right / self.encoder_resolution

        # 선속도 (Linear Velocity)와 각속도 (Angular Velocity) 계산
        self.v = (d_left + d_right) / 2  # 평균 이동 거리 => 선속도
        self.omega = (d_right - d_left) / self.wheel_base  # 휠 간 차이 => 각속도

        # 위치 갱신 (모터 오프셋 보정 포함)
        if self.omega != 0:
            # 회전 중심 기준 보정 (로봇 중심으로 변환)
            radius = self.v / self.omega  # 회전 반지름
            icc_x = self.x - radius * math.sin(self.theta)  # 회전 중심(ICC)의 x
            icc_y = self.y + radius * math.cos(self.theta)  # 회전 중심(ICC)의 y

            # 새 좌표 계산 (회전 중심 기준으로)
            self.theta += self.omega * delta_time
            self.x = icc_x + radius * math.sin(self.theta)
            self.y = icc_y - radius * math.cos(self.theta)
        else:
            # 직선 이동의 경우
            self.x += self.v * math.cos(self.theta) * delta_time
            self.y += self.v * math.sin(self.theta) * delta_time

        # 이전 엔코더값을 갱신
        self.prev_left_encoder = left_encoder
        self.prev_right_encoder = right_encoder

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        # 파라미터 설정
        self.wheel_base = 60  # 휠 간 거리 (cm)
        self.wheel_radius = 6  # 휠 반지름 (cm)
        self.encoder_resolution = 300  # 엔코더 해상도 (회전당 센서 카운트)
        self.motor_offset = 30  # 모터 축이 로봇 중심에서 x축으로부터 뒤로 떨어진 거리 (cm)

        # Odometry 객체 생성
        self.odometry = RobotOdometry(wheel_base=self.wheel_base, wheel_radius=self.wheel_radius, 
                                      encoder_resolution=self.encoder_resolution, motor_offset=self.motor_offset)

        # Odometry 메시지를 발행할 퍼블리셔
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # TF 발행을 위한 broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 타이머를 이용해 주기적으로 odometry 업데이트 및 발행
        self.timer = self.create_timer(0.1, self.timer_callback)

        # 초기 엔코더 값 (시작 시)
        self.left_encoder = 0
        self.right_encoder = 0

        # 이전 시간 기록
        self.last_time = self.get_clock().now()

    def timer_callback(self):
        # 현재 시간 계산
        current_time = self.get_clock().now()
        delta_time = (current_time - self.last_time).nanoseconds / 1e9  # 나노초를 초로 변환

        if delta_time > 0:
            # 엔코더 값 주기적으로 증가 (테스트용으로)
            # 실제 데이터를 받아 해야 함, /motor_status data[2](motor1번엔코더),data[5](motor2번엔코더)
            self.left_encoder += 30
            self.right_encoder += 30

            # 오도메트리 업데이트
            self.odometry.update(self.left_encoder, self.right_encoder, delta_time)

            # Odometry 메시지 생성
            odom_msg = Odometry()
            odom_msg.header.stamp = current_time.to_msg()
            odom_msg.header.frame_id = 'odom'
            odom_msg.pose.pose.position.x = self.odometry.x / 100  # cm -> m
            odom_msg.pose.pose.position.y = self.odometry.y / 100  # cm -> m
            odom_msg.pose.pose.orientation.z = math.sin(self.odometry.theta / 2)
            odom_msg.pose.pose.orientation.w = math.cos(self.odometry.theta / 2)

            # 속도 정보 추가
            odom_msg.child_frame_id = 'base_footprint'  # 속도는 로봇의 기본 프레임 기준
            odom_msg.twist.twist.linear.x = self.odometry.v  # 선속도 (m/s)
            odom_msg.twist.twist.angular.z = self.odometry.omega  # 각속도 (rad/s)

            # Odometry 메시지 발행
            self.odom_pub.publish(odom_msg)

            # TF 변환 발행
            transform = TransformStamped()
            transform.header.stamp = current_time.to_msg()
            transform.header.frame_id = 'odom'
            transform.child_frame_id = 'base_footprint'

            transform.transform.translation.x = self.odometry.x / 100  # cm -> m
            transform.transform.translation.y = self.odometry.y / 100  # cm -> m
            transform.transform.translation.z = 0.0
            transform.transform.rotation.z = math.sin(self.odometry.theta / 2)
            transform.transform.rotation.w = math.cos(self.odometry.theta / 2)

            self.tf_broadcaster.sendTransform(transform)

            # 마지막 시간 업데이트
            self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
