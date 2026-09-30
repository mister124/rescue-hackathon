"""
공용 설정. 각자 자기 구역만 수정할 것.
당일: 장치 이름, 시작 위치, 대상 색, 대상 수, 제한 시간부터 채운다.
"""

import math
import os
from pathlib import Path

# ============================ [D] 미션 / 장치 ============================
LEFT_MOTOR, RIGHT_MOTOR = "left wheel motor", "right wheel motor"
LEFT_ENC, RIGHT_ENC = "left wheel sensor", "right wheel sensor"
LIDAR_NAME, CAMERA_NAME = "LDS-01", "camera"
GYRO_NAME, COMPASS_NAME = "gyro", "compass"
GLOBAL_MAP_DISPLAY = "Global Map"
CAMERA_DISPLAY = "Camera YOLO"
LIDAR_DISPLAY = "Lidar Point Cloud"

START_X, START_Y, START_YAW = (
    0.0,
    0.0,
    0.0,
)  # 시작점 기준 좌표. 보통 (0, 0, 0); 월드 translation 복사 불필요
NUM_TARGETS = 2  # 찾아야 할 빨간 사과 수
TIME_LIMIT_S = 1000.0  # 제한 시간 [s] (시뮬레이션 시간)
RETURN_MARGIN_S = 100.0  # 제한 시간 이만큼 전에는 무조건 복귀 시작
WARMUP_S = 1.0  # 시작 후 이 시간 동안은 지도만 그림
EXPLORE_FAIL_LIMIT = 10  # frontier를 연속 이만큼 못 찾으면 탐색 종료
GOTO_GIVEUP_S = 15.0  # 경로 없음/접근 진행 없음 허용 시간
APPROACH_DIST = 0.5  # 대상 앞 몇 m 지점까지 갈지
GOAL_TOL = 0.2  # 도착 판정 거리

SHOW_WINDOWS = os.environ.get("RESCUE_SHOW", "0") == "1"  # 기본 출력은 월드 내부 Display
USE_GT_DEBUG = False  # True면 Supervisor로 실제 위치와 오차 출력 (월드에 supervisor TRUE 필요, 제출 시 False)
TELEOP_SPEED = 3.0  # M키로 수동 모드 전환 후 WASD 바퀴 속도 [rad/s]

# D 통합 설정
REQUIRE_CAMERA = True  # 지도/수동 테스트만 할 때 False
PLAN_INTERVAL_S = 1.5  # 주행 중 전역 경로 재계획 간격
EMPTY_REPLAN_S = 0.3  # 경로가 없을 때 재계획 간격
GOTO_MAX_S = 45.0  # 한 대상 접근 시도당 최대 시간
TARGET_RETRY_S = 20.0  # 실패한 대상 재시도 전 대기
TARGET_MAX_ATTEMPTS = 2  # 같은 대상 최대 접근 횟수
STATUS_EVERY_S = 5.0  # 콘솔 상태 출력 주기

# ============================ [A] 위치 / 지도 ============================
WHEEL_RADIUS = 0.033  # 바퀴 반지름 [m]
AXLE_LENGTH = 0.160  # 두 바퀴 사이 거리 [m]
LIDAR_ANGLE_SIGN = 1  # 왼쪽 장애물이 지도 오른쪽에 찍히면 -1
GYRO_AXIS = 2  # 자이로 yaw축 인덱스
USE_GYRO = True
USE_COMPASS = False  # 켜기 전에 USE_GT_DEBUG로 heading이 좋아지는지 확인
COMPASS_SIGN = -1  # compass heading 부호 (실험으로 확인)
COMPASS_GAIN = 0.02  # 매 step compass 쪽으로 당기는 비율
COMPASS_GATE_DEG = 30.0  # 순간 compass 교란을 무시하는 오차 한계
LIDAR_OFFSET_X = 0.0  # 로봇 중심에서 LiDAR 원점까지 전방 오프셋 [m]; 월드 모델 확인
LIDAR_HALF_BEAM = True  # 360도 LiDAR 빔 중심각 반 칸 보정
SCAN_SKIP_S = 1.0  # 시작 직후 센서 안정화 대기 [s]
L_FREE, L_OCC = -0.4, 0.9  # 점유 격자 log-odds 증분
L_MIN, L_MAX = -4.0, 4.0  # log-odds 포화 범위
FREE_INF_RANGE = 3.5  # inf 빔에서 빈 공간으로 기록할 최대 거리 [m]
MAP_INSERT_DIST = 0.08  # 이 거리 이상 이동하면 새 스캔을 지도에 추가 [m]
MAP_INSERT_ANG_DEG = 5.0  # 이 각도 이상 회전하면 새 스캔 추가 [deg]
MAP_INSERT_S = 1.0  # 이동이 적어도 이 주기로 지도 갱신 [s]
USE_SCAN_MATCH = True  # OpenCV가 있으면 scan-to-map 위치 보정
SCAN_MATCH_HZ = 2.0  # scan matching 주기 [Hz]
SM_WIN = 0.15  # 위치 매칭 검색 반경 [m]
SM_YAW_WIN_DEG = 4.0  # compass가 불안정할 때 heading 검색 반경 [deg]
SM_MIN_SCORE = 0.25  # 이보다 낮은 매칭 결과는 버림
SM_GAIN = 0.5  # 매칭 위치 보정을 적용하는 비율
SM_DEG_STD = 0.08  # 이보다 불확실한 축 방향 보정은 제외 [m]
MAP_SIZE_M = 20.0  # 지도 한 변 길이 [m]
RES = 0.05  # 지도 한 칸 크기 [m]

# ============================ [B] 주행 / 안전 ============================
MAX_WHEEL_SPEED = 6.0  # 모터 최대 각속도 [rad/s] (한계 6.67)
ROBOT_RADIUS = 0.12  # 로봇 반경 [m] (실제 0.105 + 여유)
INFLATE_M = ROBOT_RADIUS + 0.10  # 전역 경로용 안전거리
INFLATE_MIN_M = ROBOT_RADIUS + 0.03  # 경로가 없을 때 쓰는 최소 안전거리 (좁은 문)
V_MAX, W_MAX = 0.18, 1.2  # 최대 선속도 [m/s], 각속도 [rad/s]
SAFE_DIST = INFLATE_MIN_M  # DWA 최소 허용 거리 (경로 계획의 최소 안전거리와 같게: 경로는 통과 가능한데 DWA가 거부하는 일 방지)
LOOKAHEAD = 0.5  # look-ahead 거리 [m]
STUCK_S = 8.0  # 이 시간 동안 0.1m도 못 가면 목표 포기
SEARCH_SPIN_W = 0.5  # 갈 곳이 없을 때 제자리 회전 속도

# Frontier 선택 / 재계획
MIN_FRONTIER_CELLS = 5  # 이보다 작은 frontier 덩어리는 잡음으로 무시
COMMIT_R = 0.8  # 기존 frontier 목표를 유지할 반경 [m]
TURN_W = 0.5  # 새 목표 선택 시 회전량 비용 [m/rad]
ARRIVE_R = 0.6  # frontier 도착 처리 거리 [m]
NO_GO_R = 0.25  # 막힌 위치 주변 진입 금지 반경 [m] (옆 통로 우회 허용)
NO_GO_TTL = 60.0  # 진입 금지 유지 시간 [s]

# 전역 경로 / DWA 비용 가중치
CENTER_M = 0.3  # 벽 근접 비용을 적용할 거리 [m]
CENTER_W = 4.0  # 벽 바로 옆 경로의 추가 비용 가중치
W_HEAD = 1.0  # DWA 목표 방향 점수
W_CLEAR = 0.5  # DWA 장애물 여유 점수
W_VEL = 0.3  # DWA 전진 속도 점수
W_PROG = 0.0  # DWA 목표 접근 점수

# 막힘 복구
BACKUP_V, BACKUP_S = -0.08, 3.0  # 후진 속도 [m/s], 지속 시간 [s]
SPIN_MOVE = 0.3  # 제자리 회전으로 판단할 이동 반경 [m]
SPIN_TURN = 2 * math.pi  # 막힘으로 판단할 누적 회전량 [rad]

# ============================ [C] 대상 검출 ============================
DETECTOR = "yolo"  # "color" 또는 "yolo"
# 대상 이름 → (색공간, [(하한, 상한), ...]). LAB 값은 tb3_segmentation의 초록 공 기준
TARGET_COLORS = {
    #"green": ("lab", [((30, 60, 90), (230, 115, 180))]),
    "red": ("hsv", [((0, 150, 80), (8, 255, 255)), ((172, 150, 80), (180, 255, 255))]),
}
YOLO_MODEL = str(Path(__file__).resolve().parents[2] / "models" / "YOLO" / "yolo11n.pt")
YOLO_CLASSES = {0: "human", 47: "apple"}
YOLO_CONF = 0.25
TARGET_MIN_AREA = 300  # 이 픽셀 수 이상 보여야 대상으로 인정
CONFIRM_N = 3  # 같은 자리에서 이만큼 보여야 대상으로 확정
MERGE_R = 0.8  # 이 거리 안의 같은 종류 관측은 같은 대상 [m]
DETECT_EVERY = 5  # 몇 step마다 카메라 처리
