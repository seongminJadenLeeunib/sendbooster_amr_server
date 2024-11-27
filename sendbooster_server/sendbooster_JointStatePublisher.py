import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray  # motor_status 메시지 타입 가정
import math

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_states)  # 0.1초마다 퍼블리시

        self.wheel_radius = 0.1  # 바퀴 반지름 (미터)
        self.gear_ratio = 10  # 기어비 (10:1)
        self.rpm_to_radps = 2 * math.pi / 60 / self.gear_ratio  # RPM -> 라디안/초 변환 계수
        self.left_angle = 0.0  # 왼쪽 바퀴 각도
        self.right_angle = 0.0  # 오른쪽 바퀴 각도
        self.left_rpm = 10 # 왼쪽 모터 RPM
        self.right_rpm = 10  # 오른쪽 모터 RPM

    def motor_status_callback(self):
        # motor_status 메시지의 인덱스 0과 3이 각각 모터 1과 2의 RPM이라고 가정
        # 실제 데이터를 받아 해야 함, /motor_status data[0](motor1번속도),data[3](motor2번속도)
        self.left_rpm = 10
        self.right_rpm = 10

    def publish_joint_states(self):
        left_radps = self.left_rpm * self.rpm_to_radps  # 왼쪽 모터 각속도 (라디안/초)
        right_radps = self.right_rpm * self.rpm_to_radps  # 오른쪽 모터 각속도 (라디안/초)

        # 각속도 적분 -> 각도
        self.left_angle += left_radps * 0.1  # 0.1초 간격 타이머
        self.right_angle += right_radps * 0.1

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['base_left_wheel_joint', 'base_right_wheel_joint']
        msg.position = [self.left_angle, self.right_angle]
        msg.velocity = [left_radps, right_radps]
        msg.effort = []  # 토크 정보 필요 없다면 생략
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
