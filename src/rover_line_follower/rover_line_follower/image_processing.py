import cv2
import numpy as np

def get_bird_eye_view_transform(img_width=640, img_height=480):
    """
    버드아이 뷰를 위한 원근 변환 매트릭스 생성

    Args:
        img_width: 이미지 가로 크기
        img_height: 이미지 세로 크기

    Returns:
        M: 원근 변환 매트릭스
        M_inv: 역변환 매트릭스
    """
    # 원본 이미지의 사각형 영역 (사다리꼴 형태)
    # 하단은 넓고, 상단은 좁게 설정하여 원근감 보정
    src_points = np.float32([
        [img_width * 0.15, img_height],           # 왼쪽 하단
        [img_width * 0.85, img_height],           # 오른쪽 하단
        [img_width * 0.40, img_height * 0.6],     # 왼쪽 상단
        [img_width * 0.60, img_height * 0.6]      # 오른쪽 상단
    ])

    # 변환 후 이미지의 사각형 영역 (직사각형)
    dst_points = np.float32([
        [img_width * 0.2, img_height],            # 왼쪽 하단
        [img_width * 0.8, img_height],            # 오른쪽 하단
        [img_width * 0.2, 0],                     # 왼쪽 상단
        [img_width * 0.8, 0]                      # 오른쪽 상단
    ])

    M = cv2.getPerspectiveTransform(src_points, dst_points)
    M_inv = cv2.getPerspectiveTransform(dst_points, src_points)

    return M, M_inv

def apply_bird_eye_view(frame, M):
    """
    버드아이 뷰 적용

    Args:
        frame: 입력 이미지
        M: 원근 변환 매트릭스

    Returns:
        warped: 변환된 이미지
    """
    h, w = frame.shape[:2]
    warped = cv2.warpPerspective(frame, M, (w, h), flags=cv2.INTER_LINEAR)
    return warped

def get_roi(frame=None, h=None, roi_start_ratio=0.5):
    """
    ROI(관심 영역) 추출

    Args:
        frame: 입력 프레임 (선택적)
        h: 프레임 높이 (선택적, frame이 없을 때 사용)
        roi_start_ratio: ROI 시작 비율 (기본값: 0.5 = 화면 하단 50%)

    Returns:
        roi: ROI 영역 (frame이 제공된 경우)
        roi_h: ROI 높이
        roi_w: ROI 너비
    """
    if frame is not None:
        h, w = frame.shape[:2]
        roi_start = int(h * roi_start_ratio)
        roi = frame[roi_start:h, :]
        roi_h, roi_w = roi.shape[:2]
        return roi, roi_h, roi_w
    elif h is not None:
        # 높이만 제공된 경우 (파라미터만 반환)
        roi_h = int(h * (1.0 - roi_start_ratio))
        # 너비는 640으로 가정 (표준 해상도)
        roi_w = 640
        return None, roi_h, roi_w
    else:
        raise ValueError("frame 또는 h 중 하나는 반드시 제공되어야 합니다")

def get_hsvroi(roi):
    """
    HSV 색공간으로 변환하고 채널 분리

    Args:
        roi: RGB/BGR 이미지

    Returns:
        hsv_roi: HSV 이미지
        s: 채도 채널
        v: 명도 채널
    """
    hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv_roi)

    return hsv_roi, s, v

def get_binary(hsv_v=None, roi_h=None, s=None, v=None):
    """
    적응형 이진화 수행

    Args:
        hsv_v: V 채널 이미지 (또는 v 파라미터 사용)
        roi_h: ROI 높이
        s: 채도 채널 (선택적)
        v: 명도 채널 (선택적, hsv_v 대신 사용 가능)

    Returns:
        binary: 이진화된 이미지
    """
    # v 파라미터가 제공되면 hsv_v 대신 사용
    if v is not None:
        hsv_v = v

    if hsv_v is None:
        raise ValueError("hsv_v 또는 v 파라미터가 필요합니다")

    if roi_h is None:
        roi_h = hsv_v.shape[0]

    binary = np.zeros_like(hsv_v)

    for y in range(roi_h):
        # 거리 비율: 하단=0, 상단=1
        distance_ratio = 1.0 - (y / roi_h)

        # 거리에 따라 상한값 조정 (가까울수록 115, 멀수록 135)
        upper_threshold = int(115 + distance_ratio * 20)

        # 해당 행에 대해 이진화
        row_binary = cv2.inRange(hsv_v[y:y+1, :], 60, upper_threshold)
        binary[y:y+1, :] = row_binary

    return binary

def get_blur_dilate_binary(binary):
    """
    형태학적 연산으로 노이즈 제거 및 선 강조

    Args:
        binary: 이진화된 이미지

    Returns:
        binary: 처리된 이진화 이미지
    """
    # 작은 노이즈 제거 (Opening)
    kernel_small = np.ones((3, 3), np.uint8)
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel_small)

    # 선을 두껍게 만들어 연속성 향상 (Dilation)
    kernel_dilate = np.ones((3, 3), np.uint8)
    binary = cv2.dilate(binary, kernel_dilate, iterations=1)

    return binary

def get_line_candidate(roi_h, binary, num_slices=15):
    """
    여러 수평 슬라이스에서 라인 후보 추출

    Args:
        roi_h: ROI 높이
        binary: 이진화된 이미지
        num_slices: 슬라이스 개수

    Returns:
        line_candidates: 각 슬라이스에서의 라인 후보 리스트
    """
    # 슬라이스 위치 비율 (하단에서 상단으로)
    slice_ratios = [0.98 - i * 0.06 for i in range(num_slices)]

    line_candidates = []

    for ratio in slice_ratios:
        y = int(roi_h * ratio)
        if y >= binary.shape[0]:
            y = binary.shape[0] - 1

        row = binary[y, :]
        white_pixels = np.where(row > 0)[0]

        if len(white_pixels) < 2:
            line_candidates.append([])
            continue

        # 연속된 픽셀을 세그먼트로 그룹화
        segments = []
        current_segment = [white_pixels[0]]

        for i in range(1, len(white_pixels)):
            if white_pixels[i] - white_pixels[i-1] <= 10:  # 10픽셀 이내면 같은 세그먼트
                current_segment.append(white_pixels[i])
            else:
                if len(current_segment) >= 2:
                    segments.append(current_segment)
                current_segment = [white_pixels[i]]

        if len(current_segment) >= 2:
            segments.append(current_segment)

        # 큰 영역(바닥/배경) 배제
        candidates_at_slice = []
        for seg in segments:
            seg_width = seg[-1] - seg[0] + 1
            seg_pixel_count = len(seg)

            # 배경 판단 기준: 폭이 100픽셀 이상이거나 픽셀 밀도가 너무 높음
            if seg_width > 100 or seg_pixel_count > 70:
                continue

            # 너무 작은 노이즈도 제외 (최소 5픽셀 이상)
            if seg_pixel_count < 5:
                continue

            # 선 후보로 저장
            seg_cx = int(np.mean(seg))
            seg_left = seg[0]
            seg_right = seg[-1]
            candidates_at_slice.append({
                'cx': seg_cx,
                'left': seg_left,
                'right': seg_right,
                'width': seg_width,
                'pixel_count': seg_pixel_count,
                'y': y
            })

        line_candidates.append(candidates_at_slice)

    return line_candidates

def isconnect(line_candidates, roi_w):
    """
    기하 연결성 검증 (바닥부터 이어진 선만 유효)

    Args:
        line_candidates: 라인 후보 리스트
        roi_w: ROI 너비

    Returns:
        slice_centers: 연결된 슬라이스의 중심점 리스트
    """
    slice_centers = []

    # 최하단(바닥)에서 시작
    if len(line_candidates) == 0 or len(line_candidates[0]) == 0:
        return slice_centers

    # 바닥에 여러 후보가 있으면 중앙에 가장 가까운 것 선택
    bottom_candidates = line_candidates[0]
    best_bottom = min(bottom_candidates, key=lambda c: abs(c['cx'] - roi_w / 2))

    slice_centers.append((best_bottom['cx'], best_bottom['y'],
                         best_bottom['left'], best_bottom['right']))

    prev_cx = best_bottom['cx']
    prev_width = best_bottom['width']

    # 상위 슬라이스로 연결성 검증
    max_lateral_jump = 80
    max_width_change = 50

    for i in range(1, len(line_candidates)):
        candidates = line_candidates[i]

        if len(candidates) == 0:
            continue

        # 거리 비율 (멀수록 완화)
        distance_ratio = 1.0 - (i / len(line_candidates))
        adjusted_jump = max_lateral_jump + int(distance_ratio * 40)
        adjusted_width_change = max_width_change + int(distance_ratio * 30)

        # 이전 중심점과 가장 가까운 후보 찾기
        best_candidate = None
        min_distance = float('inf')

        for cand in candidates:
            cx_dist = abs(cand['cx'] - prev_cx)
            width_diff = abs(cand['width'] - prev_width)

            if cx_dist < adjusted_jump and width_diff < adjusted_width_change:
                if cx_dist < min_distance:
                    min_distance = cx_dist
                    best_candidate = cand

        if best_candidate is not None:
            slice_centers.append((best_candidate['cx'], best_candidate['y'],
                                 best_candidate['left'], best_candidate['right']))
            prev_cx = best_candidate['cx']
            prev_width = best_candidate['width']

    return slice_centers

def hysteresis(slice_centers, prev_line_valid, prev_slice_centers):
    """
    히스테리시스 (이전 프레임과 비교하여 안정성 검증)

    Args:
        slice_centers: 현재 슬라이스 중심점
        prev_line_valid: 이전 프레임 라인 유효 여부
        prev_slice_centers: 이전 프레임 슬라이스 중심점

    Returns:
        line_quality_score: 라인 품질 점수
        stable_frames: 안정 프레임 카운터
    """
    line_quality_score = 0
    stable_frames = 0

    if len(slice_centers) >= 3:
        # 기본 점수: 슬라이스 개수
        line_quality_score = len(slice_centers)

        # 보너스: 이전 프레임과 유사하면 가산점
        if prev_line_valid and len(prev_slice_centers) > 0:
            # 최하단 위치 비교
            prev_bottom_cx = prev_slice_centers[0][0]
            curr_bottom_cx = slice_centers[0][0]
            position_diff = abs(curr_bottom_cx - prev_bottom_cx)

            if position_diff < 30:  # 30픽셀 이내면 안정적
                line_quality_score += 5
                stable_frames = 1

    return line_quality_score, stable_frames

def hysteresis_thres(line_quality_score, slice_centers, prev_line_valid,
                     prev_slice_centers, stable_frames):
    """
    히스테리시스 임계값 적용

    Args:
        line_quality_score: 라인 품질 점수
        slice_centers: 현재 슬라이스 중심점
        prev_line_valid: 이전 프레임 라인 유효 여부
        prev_slice_centers: 이전 프레임 슬라이스 중심점
        stable_frames: 안정 프레임 카운터

    Returns:
        line_quality_score: 업데이트된 라인 품질 점수
        prev_line_valid: 업데이트된 라인 유효 여부
        prev_slice_centers: 업데이트된 슬라이스 중심점
        slice_centers: 최종 슬라이스 중심점
    """
    if prev_line_valid:
        # 이전에 선이 있었으면 낮은 임계값 (3개 이상)
        min_quality = 3
    else:
        # 이전에 선이 없었으면 높은 임계값 (5개 이상)
        min_quality = 5

    if line_quality_score >= min_quality:
        # 선 채택
        prev_line_valid = True
        prev_slice_centers = slice_centers.copy()
    else:
        # 선 거부 - 이전 데이터 유지 또는 초기화
        if stable_frames > 5:
            # 안정적이었으면 이전 데이터 일부 사용
            slice_centers = prev_slice_centers.copy()
        else:
            # 불안정하면 초기화
            slice_centers = []
            prev_line_valid = False

    return line_quality_score, prev_line_valid, prev_slice_centers, slice_centers

def draw_debug_info(roi, slice_centers, roi_h, roi_w, line_detected,
                    angular_z, speed, status, frames_without_line=0):
    """
    디버그 정보 시각화

    Args:
        roi: ROI 이미지
        slice_centers: 슬라이스 중심점 리스트
        roi_h: ROI 높이
        roi_w: ROI 너비
        line_detected: 라인 검출 여부
        angular_z: 각속도
        speed: 선속도
        status: 상태 문자열
        frames_without_line: 라인 없는 프레임 수

    Returns:
        roi: 디버그 정보가 그려진 ROI 이미지
    """
    # 검출된 모든 슬라이스의 중심점 표시
    for i, (cx, y_s, left, right) in enumerate(slice_centers):
        color = (0, 255, 255) if i == 0 else (255, 128, 0)
        cv2.circle(roi, (int(cx), int(y_s)), 5, color, -1)

    # 최하단 슬라이스의 좌우 끝점 표시
    if len(slice_centers) > 0:
        cx, y_s, left, right = slice_centers[0]
        cv2.circle(roi, (int(left), int(y_s)), 3, (255, 0, 0), -1)
        cv2.circle(roi, (int(right), int(y_s)), 3, (0, 0, 255), -1)
        cv2.line(roi, (int(left), int(y_s)), (int(right), int(y_s)), (255, 255, 0), 2)

    # ROI 중심선 표시
    cv2.line(roi, (roi_w // 2, 0), (roi_w // 2, roi_h), (0, 255, 0), 1)

    # 검출된 중심점들을 선으로 연결
    if len(slice_centers) >= 2:
        line_points = [(int(cx), int(y_s)) for cx, y_s, _, _ in slice_centers]

        for i in range(len(line_points) - 1):
            cv2.line(roi, line_points[i], line_points[i+1], (0, 255, 0), 3)

        # 룩어헤드 포인트 표시
        lookahead_idx = min(len(slice_centers) - 1, int(len(slice_centers) * 0.5))
        lookahead_cx, lookahead_y = slice_centers[lookahead_idx][0], slice_centers[lookahead_idx][1]
        cv2.circle(roi, (int(lookahead_cx), int(lookahead_y)), 10, (255, 0, 255), 2)

    # 상태 정보 표시
    color = (255, 255, 0) if line_detected else (0, 165, 255)
    cv2.putText(roi, f"{status} | Ang:{angular_z:.2f} Spd:{speed:.2f}",
                (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

    # 검출된 슬라이스 개수 표시
    cv2.putText(roi, f"Slices: {len(slice_centers)}",
                (10, roi_h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

    return roi
