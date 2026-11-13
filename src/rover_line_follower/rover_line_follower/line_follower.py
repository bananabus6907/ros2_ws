#!/usr/bin/env python3
"""
간단한 라인 팔로워
- 버드아이 뷰
- 간단한 라인 검출
- P 제어
- 후진 복구 모드
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from rover_line_follower import image_processing

# Helper functions wrapping image_processing module
def get_bird_eye_view(frame):
    """Apply bird's eye view transformation"""
    h, w = frame.shape[:2]
    M, _ = image_processing.get_bird_eye_view_transform(w, h)
    return image_processing.apply_bird_eye_view(frame, M)

def get_roi(warped):
    """Extract ROI from warped image"""
    roi, _, _ = image_processing.get_roi(warped)
    return roi

def detect_line(roi):
    """Detect line in ROI and return center point"""
    roi_h, roi_w = roi.shape[:2]
    _, s, v = image_processing.get_hsvroi(roi)
    binary = image_processing.get_binary(v=v, roi_h=roi_h)
    binary = image_processing.get_blur_dilate_binary(binary)

    line_candidates = image_processing.get_line_candidate(roi_h, binary)
    slice_centers = image_processing.isconnect(line_candidates, roi_w)

    if len(slice_centers) > 0:
        line_center = slice_centers[0][0]  # Use bottom slice center
        return line_center, binary
    else:
        return None, binary

def draw_debug(roi, line_center, status):
    """Draw debug visualization on ROI"""
    roi_debug = roi.copy()
    roi_h, roi_w = roi.shape[:2]

    # Draw center line
    cv2.line(roi_debug, (roi_w // 2, 0), (roi_w // 2, roi_h), (0, 255, 0), 1)

    # Draw detected line center
    if line_center is not None:
        cv2.circle(roi_debug, (int(line_center), roi_h - 10), 8, (0, 255, 255), -1)
        cv2.line(roi_debug, (int(line_center), 0), (int(line_center), roi_h), (255, 0, 255), 2)

    # Draw status text
    color = (255, 255, 0) if line_center is not None else (0, 165, 255)
    cv2.putText(roi_debug, status, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

    return roi_debug

class SimpleLineFollower(Node):
    def __init__(self):
        super().__init__('simple_line_follower')

        self._init_parameters()
        self._init_ros_communication()
        self._init_control_state()

        self.get_logger().info("Simple Line Follower Started")

    def _init_parameters(self):
        """파라미터 초기화"""
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('linear_speed', 1.2)
        self.declare_parameter('curve_speed', 0.4)
        self.declare_parameter('kp', 0.020)
        self.declare_parameter('debug_view', True)

        self.image_topic = self.get_parameter('image_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.curve_speed = self.get_parameter('curve_speed').value
        self.kp = self.get_parameter('kp').value
        self.debug_view = self.get_parameter('debug_view').value

    def _init_ros_communication(self):
        """ROS2 통신 초기화"""
        self.bridge = CvBridge()
        self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_vel_topic, 10)
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, 10
        )

    def _init_control_state(self):
        """제어 상태 초기화"""
        self.lost_count = 0
        self.max_lost_before_reverse = 10
        self.last_angular = 0.0
        self.recovery_mode = None  # None, 'reverse', 'rotate'
        self.rotation_direction = 1.0
        self.rotation_angle = 0.0
        self.max_rotation = np.pi/2

    def _process_image(self, frame):
        """
        이미지 처리 파이프라인

        Args:
            frame: 입력 이미지

        Returns:
            roi: ROI 이미지
            line_center: 라인 중심 x 좌표
            binary: 이진화 이미지
            warped: 버드아이 뷰 이미지
        """
        frame = cv2.flip(frame, 0)
        warped = get_bird_eye_view(frame)
        roi = get_roi(warped)
        line_center, binary = detect_line(roi)

        return roi, line_center, binary, warped

    def _compute_tracking_control(self, line_center, roi_width):
        """
        정상 추적 모드 제어 계산

        Args:
            line_center: 라인 중심 x 좌표
            roi_width: ROI 너비

        Returns:
            speed: 선속도
            angular_z: 각속도
            status: 상태 문자열
        """
        roi_center = roi_width // 2

        # 복구 상태 리셋
        self.lost_count = 0
        self.recovery_mode = None
        self.rotation_angle = 0.0

        # P 제어
        error = line_center - roi_center
        angular_z = -self.kp * error

        # 속도 계산 (커브에서 감속)
        if abs(angular_z) < 0.3:
            speed = self.linear_speed
        else:
            speed = self.curve_speed
            # 커브 방향 기억 (복구 시 사용)
            self.rotation_direction = 1.0 if angular_z < 0 else -1.0

        self.last_angular = angular_z

        status = f"TRACKING | Error: {error:.0f} | Angular: {angular_z:.2f}"

        return speed, angular_z, status

    def _compute_recovery_control(self):
        """
        복구 모드 제어 계산

        Returns:
            speed: 선속도
            angular_z: 각속도
            status: 상태 문자열
        """
        self.lost_count += 1

        # 1단계: 마지막 조향각 유지하며 전진
        if self.lost_count <= self.max_lost_before_reverse:
            speed = self.curve_speed * 0.5
            angular_z = self.last_angular
            status = f"SEARCHING ({self.lost_count}) | Keeping last turn"

        # 2단계: 일자로 후진
        elif self.recovery_mode != 'rotate':
            if self.recovery_mode != 'reverse':
                self.recovery_mode = 'reverse'
                self.get_logger().info("Starting REVERSE mode")

            speed = -self.curve_speed
            angular_z = 0.0
            status = f"REVERSE ({self.lost_count})"

            # 후진 충분히 했으면 회전 모드로 전환
            if self.lost_count > self.max_lost_before_reverse + 15:
                self.recovery_mode = 'rotate'
                self.rotation_angle = 0.0
                self.get_logger().info(f"Starting ROTATE mode (direction: {self.rotation_direction})")

        # 3단계: 커브 방향으로 제자리 회전
        else:
            speed = 0.0
            angular_z = self.rotation_direction * 0.4

            # 회전 각도 누적
            self.rotation_angle += abs(angular_z) * 0.1

            if self.rotation_angle >= self.max_rotation:
                # 180도 회전 후 반대 방향으로
                self.rotation_direction *= -1
                self.rotation_angle = 0.0
                self.get_logger().warn("180° rotation completed, reversing direction")

            status = f"ROTATE ({self.rotation_angle * 180 / np.pi:.0f}°) | Dir: {'RIGHT' if self.rotation_direction > 0 else 'LEFT'}"

        return speed, angular_z, status

    def _publish_control(self, speed, angular_z):
        twiststamped = TwistStamped()
        twiststamped.twist.linear.x = speed
        twiststamped.twist.angular.z = -angular_z  # 좌우 반전
        self.cmd_pub.publish(twiststamped)

    def _show_debug(self, roi, line_center, status, binary, warped):
        """
        디버그 화면 표시

        Args:
            roi: ROI 이미지
            line_center: 라인 중심
            status: 상태 문자열
            binary: 이진화 이미지
            warped: 버드아이 뷰 이미지
        """
        if self.debug_view:
            roi_debug = draw_debug(roi, line_center, status)
            cv2.imshow("ROI", roi_debug)
            cv2.imshow("Binary", binary)
            cv2.imshow("Bird Eye View", warped)
            cv2.waitKey(1)

    def image_callback(self, msg):
        """이미지 콜백 - 메인 루프"""
        # 이미지 변환
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # 이미지 처리
        roi, line_center, binary, warped = self._process_image(frame)

        # 제어 계산
        _, w = roi.shape[:2]

        if line_center is not None:
            # 정상 추적 모드
            speed, angular_z, status = self._compute_tracking_control(line_center, w)
        else:
            # 복구 모드
            speed, angular_z, status = self._compute_recovery_control()

        # 제어 명령 발행
        self._publish_control(speed, angular_z)

        # 디버그 화면
        self._show_debug(roi, line_center, status, binary, warped)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleLineFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()