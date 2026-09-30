"""[D] 통합·미션. A/B/C의 공개 함수는 유지한다.
M 자동/수동, WASD 이동, Space 정지 잠금, P 재개, R 복귀. OpenCV s 프레임 저장.
"""
import json
import math
import os
from pathlib import Path
import numpy as np
from config import *
from slam import Slam, wrap
import nav

cv2 = None  # D의 영상 처리는 main에서 초기화; B nav는 OpenCV를 직접 import
TERMINAL = {"DONE", "TIMEOUT", "ERROR"}


def validate_config():
    positive = (TIME_LIMIT_S, GOAL_TOL, APPROACH_DIST, PLAN_INTERVAL_S,
                EMPTY_REPLAN_S, GOTO_GIVEUP_S, GOTO_MAX_S, TARGET_RETRY_S,
                WHEEL_RADIUS, AXLE_LENGTH, MAX_WHEEL_SPEED, STATUS_EVERY_S)
    if not all(math.isfinite(v) and v > 0 for v in positive):
        raise ValueError("시간/거리/바퀴 설정은 유한한 양수 필요")
    if not 0 <= RETURN_MARGIN_S < TIME_LIMIT_S:
        raise ValueError("0 <= RETURN_MARGIN_S < TIME_LIMIT_S 필요")
    if not 0 <= WARMUP_S < TIME_LIMIT_S - RETURN_MARGIN_S:
        raise ValueError("WARMUP_S는 복귀 시작 시점보다 작아야 함")
    if any(not isinstance(v, int) or v < 1 for v in
           (NUM_TARGETS, DETECT_EVERY, EXPLORE_FAIL_LIMIT, TARGET_MAX_ATTEMPTS)):
        raise ValueError("대상 수/주기/재시도 횟수는 1 이상의 정수 필요")
    if not all(math.isfinite(v) for v in (START_X, START_Y, START_YAW)):
        raise ValueError("시작 좌표는 유한한 값 필요")
    if not math.isfinite(TELEOP_SPEED) or TELEOP_SPEED < 0:
        raise ValueError("TELEOP_SPEED는 0 이상의 유한한 값 필요")


def lidar_angles(n, fov):
    if n < 1 or not math.isfinite(fov) or not 0 < fov <= 2 * math.pi + 1e-3:
        raise ValueError("LiDAR 해상도/FOV 확인 필요")
    if n == 1:
        return np.zeros(1)
    step = fov / n if fov > 2 * math.pi - 1e-3 else fov / (n - 1)
    return LIDAR_ANGLE_SIGN * (fov / 2 - np.arange(n) * step)


def clean_ranges(raw, n):
    values = np.asarray(raw, dtype=float).copy()
    if values.shape != (n,):
        raise RuntimeError(f"LiDAR 크기 불일치: 기대 {n}, 수신 {values.shape}")
    values[values <= 0] = np.nan  # +inf는 범위 안에 반사점 없음: 유지
    if np.isnan(values).all():
        raise RuntimeError("LiDAR 유효 빔 없음: 장치/활성화/월드 확인 필요")
    return values


def limited_wheels(left, right, left_limit, right_limit):
    if not all(math.isfinite(v) for v in (left, right, left_limit, right_limit)):
        raise ValueError("유효하지 않은 모터 속도")
    if min(left_limit, right_limit) <= 0:
        raise ValueError("모터 최대 속도는 양수 필요")
    scale = max(1.0, abs(left) / left_limit, abs(right) / right_limit)
    return left / scale, right / scale

def guard_velocity(v, w, ranges, angles):
    """진행 영역에 가까운 장애물이 있으면 양쪽 모터 정지."""
    if abs(v) < 1e-6 and abs(w) < 1e-6:
        return 0.0, 0.0

    if ranges is None or angles is None:
        return 0.0, 0.0

    ranges = np.asarray(ranges, dtype=float)
    angles = np.asarray(angles, dtype=float)

    if (
        ranges.ndim != 1
        or ranges.shape != angles.shape
        or ranges.size == 0
        or not np.isfinite(angles).all()
    ):
        return 0.0, 0.0

    # 진행 방향의 센서 상태 확인.
    if v > 1e-6:
        sector = np.abs(angles) <= math.radians(60)
    elif v < -1e-6:
        sector = np.abs(angles) >= math.radians(120)
    else:
        sector = np.ones(angles.shape, dtype=bool)

    if not np.any(sector):
        return 0.0, 0.0

    invalid = np.isnan(ranges) | (ranges <= 0)
    if np.any(invalid & sector):
        return 0.0, 0.0

    # +inf는 반사점 없음으로 유지. 유한한 측정점만 좌표 변환.
    valid = np.isfinite(ranges) & (ranges > 0)
    x = (
        ranges[valid] * np.cos(angles[valid])
        + LIDAR_OFFSET_X
    )
    y = ranges[valid] * np.sin(angles[valid])
    distance = np.hypot(x, y)

    # 몸체 바로 주변의 장애물 검사.
    if np.any(distance <= ROBOT_RADIUS + 0.01):
        return 0.0, 0.0

    half_width = ROBOT_RADIUS + D_SIDE_MARGIN

    if v > 1e-6:
        danger = (
            (x >= 0)
            & (x <= D_STOP_DIST)
            & (np.abs(y) <= half_width)
        )
    elif v < -1e-6:
        danger = (
            (x <= 0)
            & (x >= -D_STOP_DIST)
            & (np.abs(y) <= half_width)
        )
    else:
        danger = np.zeros(x.shape, dtype=bool)

    if abs(w) > 1e-6:
        danger |= distance <= (
            ROBOT_RADIUS + D_ROTATE_MARGIN
        )

    if np.any(danger):
        return 0.0, 0.0

    return v, w
class Mission:
    """elapsed: 컨트롤러 시작 이후 경과한 시뮬레이션 초. 실제 시간과 구별."""
    def __init__(self, grid, book):
        self.grid, self.book = grid, book
        self.state, self.reason = "WARMUP", "센서 초기화"
        self.path, self.goal, self.target = [], None, None
        self.blacklist, self.attempts, self.retry_at = [], {}, {}
        self.last_plan = -math.inf
        self.explore_fail = 0
        self.follower = nav.Follower()
        self.goto_start = self.progress_at = self.last_path_ok = 0.0
        self.best_distance = math.inf
        self.finished_at = None

    def visited_count(self):
        return sum(bool(t["visited"]) for t in self.book.confirmed())

    def invalidate_path(self):
        self.path, self.goal = [], None
        self.last_plan = -math.inf
        self.follower = nav.Follower()

    def transition(self, state, reason, elapsed):
        if self.state == state:
            return
        self.state, self.reason = state, reason
        self.invalidate_path()
        print(f"[STATE] {state} | {reason} | {elapsed:.1f}s")
        if state in TERMINAL:
            self.finished_at = elapsed

    def request_return(self, reason, elapsed):
        if self.state not in TERMINAL and self.state != "RETURN":
            self.target = None
            self.transition("RETURN", reason, elapsed)

    def check_clock(self, elapsed):
        if self.state in TERMINAL:
            return
        if elapsed >= TIME_LIMIT_S:
            self.transition("TIMEOUT", "제한 시간 도달: 정지", elapsed)
        elif elapsed >= TIME_LIMIT_S - RETURN_MARGIN_S:
            self.request_return("제한 시간 복귀", elapsed)
        elif self.state == "WARMUP" and elapsed >= WARMUP_S:
            self.transition("EXPLORE", "지도 초기화 완료", elapsed)

    def resume(self, elapsed):
        self.invalidate_path()
        if self.state == "GOTO":
            self.goto_start = self.progress_at = self.last_path_ok = elapsed
            self.best_distance = math.inf

    def give_up(self, elapsed, reason):
        target = self.target
        self.retry_at[id(target)] = elapsed + TARGET_RETRY_S
        print(f"[GIVEUP] {target['cls']} | {reason} | "
              f"시도 {self.attempts[id(target)]}/{TARGET_MAX_ATTEMPTS}")
        # 실패한 대상은 visited=False 유지. 실제 접근 성공만 방문으로 계산.
        self.target = None
        self.transition("EXPLORE", "다른 대상/공간 탐색", elapsed)

    def step(self, elapsed, pose, ranges, angles, autonomous=True):
        self.check_clock(elapsed)
        if self.state in TERMINAL or self.state == "WARMUP" or not autonomous:
            return 0.0, 0.0
        if self.state == "RETURN" and math.hypot(pose[0] - START_X, pose[1] - START_Y) <= GOAL_TOL:
            self.transition("DONE", "시작 지점 복귀 완료", elapsed)
            outcome = "SUCCESS" if self.visited_count() >= NUM_TARGETS else "INCOMPLETE"
            print(f"[RESULT] {outcome} | 방문 {self.visited_count()}/{NUM_TARGETS}")
            return 0.0, 0.0
        if self.state == "EXPLORE":
            if self.visited_count() >= NUM_TARGETS:
                self.request_return("필요 대상 방문 완료", elapsed)
                return 0.0, 0.0
            candidates = [t for t in self.book.confirmed() if not t["visited"]
                          and self.attempts.get(id(t), 0) < TARGET_MAX_ATTEMPTS
                          and elapsed >= self.retry_at.get(id(t), -math.inf)]
            if candidates:
                self.target = min(candidates, key=lambda t: math.hypot(t["x"] - pose[0], t["y"] - pose[1]))
                self.attempts[id(self.target)] = self.attempts.get(id(self.target), 0) + 1
                self.goto_start = self.progress_at = self.last_path_ok = elapsed
                self.best_distance = math.inf
                self.transition("GOTO", f"{self.target['cls']} 접근", elapsed)
        if self.state == "GOTO":
            target = self.target
            distance = math.hypot(target["x"] - pose[0], target["y"] - pose[1])
            if distance <= APPROACH_DIST + GOAL_TOL:
                target["visited"] = True
                print(f"[VISIT] {target['cls']} ({target['x']:.2f}, {target['y']:.2f})")
                self.target = None
                if self.visited_count() >= NUM_TARGETS:
                    self.request_return("필요 대상 방문 완료", elapsed)
                else:
                    self.transition("EXPLORE", "다음 대상 탐색", elapsed)
                return 0.0, 0.0
            if distance < self.best_distance - 0.05:
                self.best_distance, self.progress_at = distance, elapsed
            if elapsed - self.goto_start >= GOTO_MAX_S or elapsed - self.progress_at >= GOTO_GIVEUP_S:
                self.give_up(elapsed, "접근 시간/진행 한도 초과")
                return 0.0, 0.0
        interval = PLAN_INTERVAL_S if self.path else EMPTY_REPLAN_S
        if self.follower.backup_until is None and elapsed - self.last_plan >= interval:
            self.last_plan = elapsed
            occ = self.grid.occupancy()
            blocks = nav.make_blocks(occ)
            if self.state == "EXPLORE":
                # frontier를 유지하면서 현재 지도로 재계획.
                if self.goal and math.hypot(self.goal[0] - pose[0], self.goal[1] - pose[1]) > nav.ARRIVE_R:
                    self.path = nav.plan(self.grid, blocks, pose, self.goal)
                    if not self.path:
                        self.blacklist.append(self.goal)
                        self.goal = None
                else:
                    self.path, self.goal = [], None
                if self.goal is None:
                    self.goal, self.path = nav.pick_frontier(self.grid, occ, blocks, pose, self.blacklist)
                self.explore_fail = 0 if self.path else self.explore_fail + 1
                if self.explore_fail >= EXPLORE_FAIL_LIMIT:
                    self.request_return("탐색 가능한 경로 없음", elapsed)
                    return 0.0, 0.0
            elif self.state == "GOTO":
                target = self.target
                dx, dy = target["x"] - pose[0], target["y"] - pose[1]
                distance = math.hypot(dx, dy)
                self.goal = (target["x"] - dx / distance * APPROACH_DIST,
                             target["y"] - dy / distance * APPROACH_DIST)
                self.path = nav.plan(self.grid, blocks, pose, self.goal)
                if self.path:
                    self.last_path_ok = elapsed
                elif elapsed - self.last_path_ok >= GOTO_GIVEUP_S:
                    self.give_up(elapsed, "경로 없음")
                    return 0.0, 0.0
            elif self.state == "RETURN":
                self.goal = (START_X, START_Y)
                self.path = nav.plan(self.grid, blocks, pose, self.goal)
        if not self.path and self.follower.backup_until is None:
            return (0.0, SEARCH_SPIN_W) if self.state == "EXPLORE" else (0.0, 0.0)
        v, w, stuck = self.follower.step(self.path, pose, ranges, angles, elapsed)
        if stuck:
            print(f"[STUCK] {self.state}: 정지 후 재계획")
            if self.state == "EXPLORE" and self.goal:
                self.blacklist.append(self.goal)
            self.invalidate_path()
            return 0.0, 0.0
        return v, w

    def result(self, elapsed, pose):
        targets = [{"cls": t["cls"], "x": float(t["x"]), "y": float(t["y"]),
                    "visited": bool(t["visited"]), "attempts": self.attempts.get(id(t), 0)}
                   for t in self.book.confirmed()]
        return {"state": self.state, "reason": self.reason,
                "success": self.state == "DONE" and self.visited_count() >= NUM_TARGETS,
                "elapsed_s": round(self.finished_at if self.finished_at is not None else elapsed, 3),
                "required": NUM_TARGETS, "visited": self.visited_count(),
                "returned": self.state == "DONE", "pose": list(map(float, pose)), "targets": targets}


def list_devices(robot):
    devices = {}
    print("===== 로봇 장치 목록 =====")
    for i in range(robot.getNumberOfDevices()):
        dev = robot.getDeviceByIndex(i)
        devices[dev.getName()] = dev
        print(f"  {dev.getName()} ({type(dev).__name__})")
    return devices


def require(devices, name, what):
    dev = devices.get(name)
    if dev is None:
        raise RuntimeError(f"{what} '{name}' 없음: 장치 목록과 config.py [D] 이름 비교 필요")
    return dev


def draw_map(grid, occ, pose, path, targets, goal, status):
    img = np.full(occ.shape + (3,), 128, np.uint8)
    img[occ == 0], img[occ == 100] = (255, 255, 255), (0, 0, 0)
    for p in path:
        r, c = grid.w2c(*p)
        if grid.inside(r, c):
            img[r, c] = (255, 0, 0)
    points = [((START_X, START_Y), (255, 180, 0), 4), (pose[:2], (0, 128, 255), 3)]
    if goal is not None:
        points.append((goal, (255, 0, 255), 3))
    for t in targets:
        points.append(((t["x"], t["y"]), (0, 200, 0) if t["visited"] else (0, 0, 255), 4))
    for xy, color, radius in points:
        r, c = grid.w2c(*xy)
        if grid.inside(r, c):
            cv2.circle(img, (c, r), radius, color, -1)
    img = cv2.resize(cv2.flip(img, 0), (600, 600), interpolation=cv2.INTER_NEAREST)
    cv2.rectangle(img, (0, 0), (600, 28), (30, 30, 30), -1)
    cv2.putText(img, status, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    cv2.imshow("map", img)


def save_result(mission, elapsed, pose):
    destination = Path(__file__).resolve().parent / "mission_result.json"
    temporary = destination.with_suffix(".tmp")
    try:
        temporary.write_text(json.dumps(mission.result(elapsed, pose), ensure_ascii=False, indent=2), encoding="utf-8")
        temporary.replace(destination)
        print(f"[SAVE] {destination.name}")
    except OSError as exc:
        print(f"[WARN] 결과 파일 저장 실패: {exc}")


def main():
    global cv2
    validate_config()
    try:
        import cv2 as opencv
        from perception import Perception, localize, TargetBook
        from controller import Robot, Supervisor
    except ImportError as exc:
        raise RuntimeError("Webots에서 실행하고, 사용 중인 Python에 requirements.txt의 패키지를 설치하세요") from exc
    cv2 = opencv
    robot = Supervisor() if USE_GT_DEBUG else Robot()
    ts = int(robot.getBasicTimeStep())
    if ts <= 0:
        raise ValueError("basicTimeStep은 양수 필요")
    dt, started = ts / 1000.0, robot.getTime()
    motors, mission = [], None
    elapsed, pose = 0.0, (START_X, START_Y, START_YAW)
    windows = SHOW_WINDOWS
    if os.name != "nt" and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        windows = False
    try:
        devices = list_devices(robot)
        for name in (LEFT_MOTOR, RIGHT_MOTOR):
            motor = require(devices, name, "모터")
            motors.append(motor)
            motor.setPosition(float("inf"))
            motor.setVelocity(0.0)
        limits = [min(MAX_WHEEL_SPEED, m.getMaxVelocity()) for m in motors]
        limited_wheels(0, 0, *limits)
        ranges = None
        angles = None

        def set_raw(left, right):
            left, right = limited_wheels(left, right, *limits)

            # 바퀴 각속도 → 로봇 선속도·각속도
            v = WHEEL_RADIUS * (left + right) / 2
            w = WHEEL_RADIUS * (right - left) / AXLE_LENGTH

            safe_v, safe_w = guard_velocity(
                v, w, ranges, angles
            )

            # 실제 이동 명령이 차단될 때 진단 출력.
            if (
                abs(v) + abs(w) > 1e-6
                and abs(safe_v) + abs(safe_w) < 1e-6
            ):
                nav.log("[GUARD] 장애물 근접 또는 센서 무효: 정지")

            # 검사 결과 → 바퀴 각속도
            left = (
                safe_v - safe_w * AXLE_LENGTH / 2
            ) / WHEEL_RADIUS
            right = (
                safe_v + safe_w * AXLE_LENGTH / 2
            ) / WHEEL_RADIUS

            left, right = limited_wheels(left, right, *limits)
            motors[0].setVelocity(left)
            motors[1].setVelocity(right)
        def set_wheels(v, w):
            set_raw((v - w * AXLE_LENGTH / 2) / WHEEL_RADIUS,
                    (v + w * AXLE_LENGTH / 2) / WHEEL_RADIUS)

        encoders = []
        for name, motor in zip((LEFT_ENC, RIGHT_ENC), motors):
            encoder = devices.get(name)
            if encoder is None:
                encoder = motor.getPositionSensor()
                if encoder is None:
                    raise RuntimeError(f"엔코더 '{name}' 및 모터 연결 PositionSensor 없음")
                print(f"[INFO] 연결된 엔코더 사용: {encoder.getName()}")
            encoder.enable(ts)
            encoders.append(encoder)
        lidar = require(devices, LIDAR_NAME, "LiDAR")
        lidar.enable(ts)
        camera = devices.get(CAMERA_NAME)
        if camera is None and REQUIRE_CAMERA:
            raise RuntimeError(f"카메라 '{CAMERA_NAME}' 없음: 미션에는 카메라 필요")
        if camera:
            camera.enable(ts)
        else:
            print("[WARN] 카메라 없이 지도/주행 테스트: 검출 불가")
        gyro = devices.get(GYRO_NAME) if USE_GYRO else None
        compass = devices.get(COMPASS_NAME) if USE_COMPASS else None
        for device, requested, label in ((gyro, USE_GYRO, "gyro"), (compass, USE_COMPASS, "compass")):
            if device:
                device.enable(ts)
            elif requested:
                print(f"[WARN] {label} 없음: encoder 기반 위치 추정 사용")
        keyboard = robot.getKeyboard()
        keyboard.enable(ts)
        n = lidar.getHorizontalResolution()
        angles = lidar_angles(n, lidar.getFov())
        max_range = lidar.getMaxRange()
        if not math.isfinite(max_range) or max_range <= 0:
            raise ValueError("LiDAR maxRange는 유한한 양수 필요")
        slam = Slam(*pose)
        book = TargetBook()
        mission = Mission(slam.grid, book)
        try:
            percep = Perception(camera) if camera else None
        except ImportError as exc:
            raise RuntimeError("C 검출기 의존성 설치 필요: python -m pip install -r requirements.txt") from exc
        if camera:
            print(f"[DETECTOR] C YOLO / 빨간 사과 / CPU / 모델: {YOLO_MODEL}")
        gt_node = robot.getSelf() if USE_GT_DEBUG else None
        if USE_GT_DEBUG and gt_node is None:
            raise RuntimeError("USE_GT_DEBUG=True에는 월드의 supervisor TRUE 필요")
        gt0 = None
        manual, paused, previous_keys = False, False, set()
        k, frame, dets = 0, None, []
        last_status = last_draw = last_gt = -math.inf
        saved_terminal = False
        print(f"[START] 제한 {TIME_LIMIT_S:.0f}s / 강제 복귀 {TIME_LIMIT_S - RETURN_MARGIN_S:.0f}s")
        print("[KEY] M 자동/수동 | WASD | Space 정지 잠금 | P 재개 | R 복귀")
        while robot.step(ts) != -1:
            k += 1
            elapsed = robot.getTime() - started
            mission.check_clock(elapsed)
            if mission.state in TERMINAL:
                set_wheels(0, 0)
                if not saved_terminal:
                    save_result(mission, elapsed, pose)
                    saved_terminal = True
                if windows:
                    try:
                        cv2.waitKey(1)
                    except cv2.error:
                        windows = False
                continue
            keys, key = set(), keyboard.getKey()
            while key != -1:
                keys.add(key & 0xFFFF)
                key = keyboard.getKey()
            pressed, previous_keys = keys - previous_keys, keys
            if ord(" ") in pressed:
                paused = True
                set_wheels(0, 0)
                print("[PAUSE] P로 재개. 제한 시간은 계속 흐름")
            elif ord("P") in pressed:
                paused = False
                mission.resume(elapsed)
            if ord("R") in pressed:
                mission.request_return("R키 복귀 요청", elapsed)
            if ord("M") in pressed and mission.state != "RETURN":
                manual = not manual
                mission.resume(elapsed)
                print("[MODE]", "MANUAL" if manual else "AUTO")
            if mission.state == "RETURN":
                manual = False
            raw = lidar.getLayerRangeImage(0) if lidar.getNumberOfLayers() > 1 else lidar.getRangeImage()
            ranges = clean_ranges(raw, n)
            enc = [e.getValue() for e in encoders]
            if not all(math.isfinite(v) for v in enc):
                raise RuntimeError("엔코더 측정값이 NaN/inf")
            gyro_z = gyro.getValues()[GYRO_AXIS] if gyro else None
            if gyro_z is not None and not math.isfinite(gyro_z):
                gyro_z = None
            compass_values = compass.getValues() if compass else None
            if compass_values is not None and not all(math.isfinite(v) for v in compass_values):
                compass_values = None
            slam.update(*enc, gyro_z, compass_values, ranges, angles, max_range, dt)
            pose = slam.pose
            if not all(math.isfinite(v) for v in pose) or not slam.grid.inside(*slam.grid.w2c(*pose[:2])):
                raise RuntimeError("위치 추정이 유효하지 않거나 지도 범위 밖")
            if gt_node is not None:
                # 현재 A와 같은 z-up/XY 월드. GT는 진단 출력에만 사용.
                p, rotation = gt_node.getPosition(), gt_node.getOrientation()
                yaw = math.atan2(rotation[3], rotation[0])
                if gt0 is None:
                    gt0 = (p[0], p[1], yaw)
                angle = START_YAW - gt0[2]
                dx, dy = p[0] - gt0[0], p[1] - gt0[1]
                gx = START_X + dx * math.cos(angle) - dy * math.sin(angle)
                gy = START_Y + dx * math.sin(angle) + dy * math.cos(angle)
                if elapsed - last_gt >= 5:
                    last_gt = elapsed
                    print(f"[GT] 위치 {math.hypot(pose[0]-gx, pose[1]-gy):.3f}m / "
                          f"heading {math.degrees(wrap(pose[2]-wrap(yaw+angle))):.1f}deg")
            if percep and k % DETECT_EVERY == 0:
                frame = percep.grab()
                if frame is not None and frame.size:
                    dets = percep.detect(frame)
                    for det in dets:
                        xy = localize(det, ranges, angles, pose, max_range)
                        if xy is not None and all(math.isfinite(v) for v in xy):
                            book.add(*xy, det["cls"])
            v, w = mission.step(elapsed, pose, ranges, angles, autonomous=not (manual or paused))
            if paused or mission.state in TERMINAL:
                set_wheels(0, 0)
            elif manual:
                forward = int(ord("W") in keys) - int(ord("S") in keys)
                turn = int(ord("A") in keys) - int(ord("D") in keys)
                set_raw(TELEOP_SPEED * (forward - turn), TELEOP_SPEED * (forward + turn))
            else:
                set_wheels(v, w)
            mode = "PAUSED" if paused else "MANUAL" if manual else "AUTO"
            status = f"{mode} {mission.state} {elapsed:.0f}/{TIME_LIMIT_S:.0f}s VISIT {mission.visited_count()}/{NUM_TARGETS}"
            if elapsed - last_status >= STATUS_EVERY_S:
                print(f"[STATUS] {status} | path={len(mission.path)}")
                last_status = elapsed
            if windows:
                try:
                    if elapsed - last_draw >= 0.2:
                        last_draw = elapsed
                        draw_map(slam.grid, slam.grid.occupancy(), pose, mission.path, book.confirmed(), mission.goal, status)
                        if frame is not None:
                            cv2.imshow("camera", Perception.draw(frame.copy(), dets))
                    if (cv2.waitKey(1) & 0xFF) == ord("s") and frame is not None:
                        filename = Path(__file__).resolve().parent / "frame.jpg"
                        print("[SAVE] frame.jpg" if cv2.imwrite(str(filename), frame) else "[WARN] frame.jpg 저장 실패")
                except cv2.error as exc:
                    windows = False
                    print(f"[WARN] 표시 창 비활성화: {exc}")
    except Exception as exc:
        if mission is not None:
            mission.transition("ERROR", str(exc), elapsed)
        print(f"[ERROR] {exc}")
        raise
    finally:
        for motor in motors:
            try:
                motor.setVelocity(0.0)
            except Exception:
                pass
        if mission is not None:
            save_result(mission, elapsed, pose)
        if windows:
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass


if __name__ == "__main__":
    main()
