"""
공용 설정. 각자 자기 구역만 수정할 것.
당일: 장치 이름, 시작 위치, 대상 색, 대상 수, 제한 시간부터 채운다.
"""
import os

# ============================ [D] 미션 / 장치 ============================
LEFT_MOTOR, RIGHT_MOTOR = "left wheel motor", "right wheel motor"
LEFT_ENC, RIGHT_ENC = "left wheel sensor", "right wheel sensor"
LIDAR_NAME, CAMERA_NAME = "LDS-01", "camera"
GYRO_NAME, COMPASS_NAME = "gyro", "compass"

START_X, START_Y, START_YAW = 0.0, 0.0, 0.0   # 시작점 기준 좌표. 보통 (0, 0, 0); 월드 translation 복사 불필요
NUM_TARGETS = 1                  # 찾아야 할 대상 수 (연습 월드: 초록 사과 1개)
TIME_LIMIT_S = 600.0             # 제한 시간 [s] (시뮬레이션 시간)
RETURN_MARGIN_S = 120.0          # 제한 시간 이만큼 전에는 무조건 복귀 시작
WARMUP_S = 1.0                   # 시작 후 이 시간 동안은 지도만 그림
EXPLORE_FAIL_LIMIT = 10          # frontier를 연속 이만큼 못 찾으면 탐색 종료
GOTO_GIVEUP_S = 10.0             # 경로 없음/접근 진행 없음 허용 시간
APPROACH_DIST = 0.5              # 대상 앞 몇 m 지점까지 갈지
GOAL_TOL = 0.2                   # 도착 판정 거리

SHOW_WINDOWS = os.environ.get("RESCUE_SHOW", "1") == "1"   # 지도/카메라 OpenCV 창
USE_GT_DEBUG = False             # True면 Supervisor로 실제 위치와 오차 출력 (월드에 supervisor TRUE 필요, 제출 시 False)
TELEOP_SPEED = 3.0               # M키로 수동 모드 전환 후 WASD 바퀴 속도 [rad/s]

# D 통합 설정
REQUIRE_CAMERA = True           # 지도/수동 테스트만 할 때 False
PLAN_INTERVAL_S = 1.5           # 주행 중 전역 경로 재계획 간격
EMPTY_REPLAN_S = 0.3            # 경로가 없을 때 재계획 간격
GOTO_MAX_S = 45.0               # 한 대상 접근 시도당 최대 시간
TARGET_RETRY_S = 20.0           # 실패한 대상 재시도 전 대기
TARGET_MAX_ATTEMPTS = 2         # 같은 대상 최대 접근 횟수
STATUS_EVERY_S = 5.0            # 콘솔 상태 출력 주기

# ============================ [A] 위치 / 지도 ============================
WHEEL_RADIUS = 0.033             # 바퀴 반지름 [m]
AXLE_LENGTH = 0.160              # 두 바퀴 사이 거리 [m]
LIDAR_ANGLE_SIGN = 1             # 왼쪽 장애물이 지도 오른쪽에 찍히면 -1
GYRO_AXIS = 2                    # 자이로 yaw축 인덱스
USE_GYRO = True
USE_COMPASS = False              # 켜기 전에 USE_GT_DEBUG로 heading이 좋아지는지 확인
COMPASS_SIGN = -1                # compass heading 부호 (실험으로 확인)
COMPASS_GAIN = 0.02              # 매 step compass 쪽으로 당기는 비율
MAP_SIZE_M = 20.0                # 지도 한 변 길이 [m]
RES = 0.05                       # 지도 한 칸 크기 [m]

# ============================ [B] 주행 / 안전 ============================
MAX_WHEEL_SPEED = 6.0            # 모터 최대 각속도 [rad/s] (한계 6.67)
ROBOT_RADIUS = 0.12              # 로봇 반경 [m] (실제 0.105 + 여유)
INFLATE_M = ROBOT_RADIUS + 0.10  # 전역 경로용 안전거리
INFLATE_MIN_M = ROBOT_RADIUS + 0.03   # 경로가 없을 때 쓰는 최소 안전거리 (좁은 문)
V_MAX, W_MAX = 0.18, 1.2         # 최대 선속도 [m/s], 각속도 [rad/s]
SAFE_DIST = ROBOT_RADIUS + 0.12  # DWA 최소 허용 거리
LOOKAHEAD = 0.5                  # look-ahead 거리 [m]
STUCK_S = 8.0                    # 이 시간 동안 0.1m도 못 가면 목표 포기
SEARCH_SPIN_W = 0.5              # 갈 곳이 없을 때 제자리 회전 속도

# ============================ [C] 대상 검출 ============================
DETECTOR = "color"               # "color" 또는 "yolo"
# 대상 이름 → (색공간, [(하한, 상한), ...]). LAB 값은 tb3_segmentation의 초록 공 기준
TARGET_COLORS = {
    "green": ("lab", [((30, 60, 90), (230, 115, 180))]),
    # "red": ("hsv", [((0, 150, 80), (8, 255, 255)), ((172, 150, 80), (180, 255, 255))]),
}
YOLO_MODEL = "../../models/YOLO/yolo11n.pt"
YOLO_CLASSES = {47: "apple"}     # COCO id → 대상 이름 (32 공, 47 사과, 49 오렌지)
YOLO_CONF = 0.25
TARGET_MIN_AREA = 300            # 이 픽셀 수 이상 보여야 대상으로 인정
CONFIRM_N = 3                    # 같은 자리에서 이만큼 보여야 대상으로 확정
MERGE_R = 0.8                    # 이 거리 안의 같은 종류 관측은 같은 대상 [m]
CAM_HEIGHT = 0.073               # 카메라 높이 [m] (extensionSlot 0.153 + 카메라 -0.08)
DETECT_EVERY = 5                 # 몇 step마다 카메라 처리
