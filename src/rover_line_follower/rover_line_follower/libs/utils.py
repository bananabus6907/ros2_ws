import rclpy
from rclpy.node import Node
# ↑ ROS2 파이썬 클라이언트 라이브러리와 Node 클래스

from sensor_msgs.msg import Image
# ↑ 카메라 영상이 전달되는 메시지 타입

from geometry_msgs.msg import Twist
# ↑ 로봇의 속도 명령 메시지 타입 (선속도 + 각속도)

from cv_bridge import CvBridge
# ↑ ROS Image 메시지를 OpenCV 이미지로 변환하기 위한 도구

import multiprocessing as mp
import cv2
import numpy as np
import time
import math

CAMERA_OFFSET_X = 2.25   # meter (앞쪽)
CAMERA_OFFSET_Y = 0.00   # meter (좌/우)

__all__ = [
"draw_robot_arrow"          ,     
"draw_candidate_points_fan" ,      

"threshold_base_map"        ,     
"resize_to_screen"          ,     

"update_base_map_with_pose" ,         
"warpAffine_no_crop"        ,     

"open_cap"                  ,     
"safe_put"                  ,     

"visualize"                 ,        

"bird_eye_convert"          ,     

"RobotState"                , 

"pure_pursuit"              , 
"evaluate_candidate"        , 
"path_planning"             , 

]

def draw_robot_arrow(img, angle, length=50):
    """
    img   : BGR 이미지
    angle : degree (로봇 heading)
    """
    h, w = img.shape[:2]
    cx, cy = w // 2, h // 2

    # angle → rad
    rad = math.radians(-angle)

    # 화살표 끝점 (y축 반전 주의)
    ex = int(cx + length * math.cos(rad))
    ey = int(cy - length * math.sin(rad))

    cv2.arrowedLine(
        img,
        (cx, cy),
        (ex, ey),
        (0, 0, 255),   # 빨간색 (BGR)
        3,
        tipLength=0.3
    )

def draw_candidate_points_fan(
    img,
    angle,
    angle_offsets=(-30, -15, 0, 15, 30),
    distances=(120, 180),
    color=(0, 0, 255),
    radius=4
):
    """
    img            : BGR 이미지
    angle          : 로봇 heading (degree)
    angle_offsets  : heading 기준 좌우 각도 후보들
    distances      : 로봇으로부터의 거리 후보들
    """

    h, w = img.shape[:2]
    cx, cy = w // 2, h // 2

    for d_angle in angle_offsets:
        rad = math.radians(-(angle + d_angle))

        for d in distances:
            x = int(cx + d * math.cos(rad))
            y = int(cy - d * math.sin(rad))

            cv2.circle(
                img,
                (x, y),
                radius,
                color,
                -1
            )

def threshold_base_map(base_map, thresh=90):
    """
    어두운 영역 → 0 (검정)
    밝은 영역 → 255 (흰색)
    """
    gray = cv2.cvtColor(base_map, cv2.COLOR_BGR2GRAY)

    _, binary = cv2.threshold(
        gray,
        thresh,
        255,
        cv2.THRESH_BINARY
    )

    return binary

def resize_to_screen(img, max_w=1280, max_h=720):
    h, w = img.shape[:2]
    scale = min(max_w / w, max_h / h)
    if scale >= 1.0:
        return img
    return cv2.resize(
        img,
        (int(w * scale), int(h * scale)),
        interpolation=cv2.INTER_AREA
    )
    
##############################################################

#def recenter_base_map(base_map, pose, scale):
#    """
#    pose = (x, y, angle)
#    """
#    if pose is None:
#        return base_map
#
#    x, y, _ = pose
#    H, W = base_map.shape[:2]
#
#    # 로봇 위치 (픽셀 단위)
#    px = int(x * scale)
#    py = int(y * scale)
#
#    # 이동 행렬 (로봇을 중앙으로)
#    M = np.float32([
#        [1, 0, -px],
#        [0, 1, -py],
#    ])
#
#    shifted = cv2.warpAffine(
#        base_map,
#        M,
#        (W, H),
#        borderValue=(255, 255, 255)
#    )
#
#    return shifted
#

def update_base_map_with_pose(
    base_map,
    current_image,
    current_pose,
    scale=300,
    bg_color=(255, 255, 255),
):
    """
    base_map       : 누적 지도 (로봇이 항상 화면 중앙)
    current_image  : 현재 카메라 프레임
    current_pose   : (x, y, angle) - 월드 좌표
    """

    H, W = base_map.shape[:2]

    # --------------------------------------------------
    # 0. 이전 pose 저장
    # --------------------------------------------------
    if not hasattr(update_base_map_with_pose, "prev_pose"):
        update_base_map_with_pose.prev_pose = current_pose

    x_p, y_p, _ = update_base_map_with_pose.prev_pose
    x_c, y_c, angle_c = current_pose

    # --------------------------------------------------
    # 1. 로봇 이동량만큼 base_map 이동
    #    (로봇이 항상 중앙에 있도록)
    # --------------------------------------------------
    dx = (x_c - x_p) * scale
    dy = (y_c - y_p) * scale

    M_shift = np.float32([
        [1, 0, -dx],
        [0, 1, -dy],
    ])

    base_map = cv2.warpAffine(
        base_map,
        M_shift,
        (W, H),
        borderValue=bg_color
    )

    # --------------------------------------------------
    # 2. 카메라 위치 계산 (로봇 좌표 → 월드 좌표)
    # --------------------------------------------------
    rad = math.radians(angle_c)

    cam_x = (
        x_c
        + CAMERA_OFFSET_X * math.cos(rad)
        - CAMERA_OFFSET_Y * math.sin(rad)
    )
    cam_y = (
        y_c
        + CAMERA_OFFSET_X * math.sin(rad)
        + CAMERA_OFFSET_Y * math.cos(rad)
    )

    # --------------------------------------------------
    # 3. 카메라 위치를 화면 좌표로 변환
    #    (로봇 중심 = 화면 중앙)
    # --------------------------------------------------
    px = int((cam_x - x_c) * scale + W // 2)
    py = int((cam_y - y_c) * scale + H // 2)

    # --------------------------------------------------
    # 4. 카메라 이미지 회전
    # --------------------------------------------------
    rotated = warpAffine_no_crop(current_image, -angle_c - 90)
    rh, rw = rotated.shape[:2]

    x1 = px - rw // 2
    y1 = py - rh // 2
    x2 = x1 + rw
    y2 = y1 + rh

    # --------------------------------------------------
    # 5. base_map에 합성
    # --------------------------------------------------
    if 0 <= x1 and 0 <= y1 and x2 <= W and y2 <= H:
        roi = base_map[y1:y2, x1:x2]
        mask = np.any(rotated != 0, axis=2)
        roi[mask] = rotated[mask]

    # --------------------------------------------------
    # 6. prev_pose 갱신
    # --------------------------------------------------
    update_base_map_with_pose.prev_pose = current_pose

    return base_map


#############################################################

def warpAffine_no_crop(img, angle):
    h, w = img.shape[:2]
    cx, cy = w / 2, h / 2

    rad = np.deg2rad(angle)
    cos = abs(np.cos(rad))
    sin = abs(np.sin(rad))

    new_w = int(h * sin + w * cos)
    new_h = int(h * cos + w * sin)

    M = cv2.getRotationMatrix2D((cx, cy), angle, 1.0)
    M[0, 2] += (new_w / 2) - cx
    M[1, 2] += (new_h / 2) - cy

    return cv2.warpAffine(img, M, (new_w, new_h))

########################################################################

def open_cap(WIDTH=1280, HEIGHT=720):
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise RuntimeError("카메라를 열 수 없습니다.")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    return cap

#############################################################

def safe_put(q, item):
    while not q.empty():
        try:
            q.get_nowait()
        except:
            break
    q.put(item)

def visualize(base_map, last_pose, last_frame=None):
    if base_map is None:
        return

    # 1. base_map → binary (이진화)
    binary = threshold_base_map(base_map)

    # 2. binary → BGR (화살표 그리기 위해)
    binary_bgr = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)

    # 3. 화면 크기에 맞게 resize
    vis = resize_to_screen(
        binary_bgr,
        max_w=1280,
        max_h=720
    )

    base_map_vis = resize_to_screen(
        base_map,
        max_w=1280,
        max_h=720
    )


    # 4. 로봇 방향 표시 (중앙 기준)
    if last_pose is not None:
        draw_robot_arrow(vis, last_pose[2])
        
    draw_candidate_points_fan(
        vis,
        angle=last_pose[2],
        angle_offsets=(-30, -15, 0, 15, 30),
        distances=(120, 180),
        radius=4
    )

    # 5. 출력
    cv2.imshow("base_map", base_map_vis)
    cv2.imshow("base_map_binary", vis)

    if last_frame is not None:
        cv2.imshow("camera", last_frame)


################################################


def bird_eye_convert(img):
    h, w = img.shape[:2]

    # ===============================
    # 1. 원본 이미지에서 사다리꼴 영역 (직접 튜닝)
    # ===============================
    src = np.float32([
        [w * 0.40, h * 0.75],  # 좌상
        [w * 0.60, h * 0.75],  # 우상
        [w * 0.99, h * 1.00],  # 우하
        [w * 0.01, h * 1.00],  # 좌하
    ])

    # ===============================
    # 2. 변환 후 직사각형
    # ===============================
    dst = np.float32([
        [0, 0],
        [w, 0],
        [w, 5*h],
        [0, 5*h],
    ])

    # ===============================
    # 3. Perspective Transform
    # ===============================
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (w, h))
    
    return warped

class RobotState:
    def __init__(self):
        self.x     = 0.0
        self.y     = 0.0
        self.angle = 0.0  # degree
        self.v     = 0.0  # current speed















def pure_pursuit(robot_x, robot_y, robot_angle, next_x, next_y):
    
    MAX_LINEAR_VELOCITY = 0.5
    GOAL_THRESHOLD = 0.1
    MIN_LOOKAHEAD_DISTANCE = 0.5
    
    dx = next_x - robot_x
    dy = next_y - robot_y
    
    target_distance_global = math.sqrt(dx**2 + dy**2)
    
    if target_distance_global < GOAL_THRESHOLD:
        linear = 0.0
        angular = 0.0
        return linear, angular
    
    cos_theta = math.cos(robot_angle)
    sin_theta = math.sin(robot_angle)
    
    x_robot = dx * cos_theta + dy * sin_theta
    y_robot = -dx * sin_theta + dy * cos_theta
    
    Ld = max(target_distance_global, MIN_LOOKAHEAD_DISTANCE)
    
    if Ld < 0.01:
        curvature = 0.0
    else:
        curvature = (2.0 * y_robot) / (Ld**2)

    linear = MAX_LINEAR_VELOCITY
    
    angular = linear * curvature
    
    return linear, angular




##########################################################
###########################################################
#def get_pos():
#    x     = 0
#    y     = 0 
#    angle = 0
#    return x, y, angle




def evaluate_candidate(binary_map, candidate, radius=15):
    """
    후보 점 주변의 '검정(0)' 픽셀 개수로 점수 계산
    binary_map: threshold된 1채널 이미지 (0/255)
    candidate : (x, y) in pixel
    """
    x, y = candidate
    h, w = binary_map.shape[:2]

    x1 = max(0, x - radius)
    x2 = min(w, x + radius)
    y1 = max(0, y - radius)
    y2 = min(h, y + radius)

    roi = binary_map[y1:y2, x1:x2]
    black_pixels = np.sum(roi == 0)
    return int(black_pixels)



def path_planning(base_map, current_pose):
    _, _, angle = current_pose

    h, w = base_map.shape[:2]
    cx, cy = w // 2, h // 2

    angle_offsets = (-30, -15, 0, 15, 30)
    distances = (120, 180)

    # 1. 후보 생성
    candidates = []

    for d_angle in angle_offsets:
        rad = math.radians(-(angle + d_angle))

        for d in distances:
            px = int(cx + d * math.cos(rad))
            py = int(cy - d * math.sin(rad))

            if 0 <= px < w and 0 <= py < h:
                candidates.append((px, py))

    # 2. 후보 평가
    best_score = -1
    next_point = (cx, cy)

    for candidate in candidates:
        score = evaluate_candidate(base_map, candidate)

        if score > best_score:
            best_score = score
            next_point = candidate

    return next_point



def process2(in_q, out_q, stop_event):
    base_map = np.full((3000, 3000, 3), 255, dtype=np.uint8)
    while not stop_event.is_set():
        try:
            pose, frame = in_q.get(timeout=0.1)
        except:
            continue
        
        frame = bird_eye_convert(frame) 
        
        base_map = update_base_map_with_pose(
            base_map      = base_map,
            current_image = frame,
            current_pose  = pose,
            scale         = 300
        )
        
        #visualize(base_map, pose, frame)
        
        #current_x, current_y, current_angle = pose
        #next_x, next_y = path_planning(base_map, pose)
        #linear, angular = pure_pursuit(
        #    current_x,
        #    current_y,
        #    current_angle,
        #    next_x,
        #    next_y
        #    )
        linear  = 2.0
        angular = 1.0
        safe_put(out_q, (linear, angular))