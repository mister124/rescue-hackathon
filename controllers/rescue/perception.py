"""
[C] 대상 검출 + 대상 위치 추정
 - main은 Perception.detect()로 검출 목록을 받고, localize()로 월드 좌표를 만들어 TargetBook에 넣는다.
 - 색 분할은 tb3_segmentation / tb3_teleop_cam, YOLO는 tb3_teleop_yolo 코드 기반
"""
import math
import numpy as np
import cv2
from config import (DETECTOR, TARGET_COLORS, YOLO_MODEL, YOLO_CLASSES, YOLO_CONF,
                    TARGET_MIN_AREA, CONFIRM_N, MERGE_R, CAM_HEIGHT)

COLOR_CONV = {"lab": cv2.COLOR_BGR2LAB, "hsv": cv2.COLOR_BGR2HSV}


class Perception:
    def __init__(self, camera):
        self.cam = camera
        self.w, self.h = camera.getWidth(), camera.getHeight()
        self.f = (self.w / 2) / math.tan(camera.getFov() / 2)   # 초점거리 [px]
        self.model = None
        if DETECTOR == "yolo":
            from ultralytics import YOLO
            self.model = YOLO(YOLO_MODEL)
            self.model.to("cpu")

    def grab(self):
        """BGR 프레임 (h, w, 3)"""
        frame = np.frombuffer(self.cam.getImage(), np.uint8).reshape((self.h, self.w, 4))
        return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

    def detect(self, frame):
        """[{"cls", "box": (x, y, w, h), "bearing", "dist"}]. 대상 종류마다 가장 큰 것 하나"""
        boxes = self._detect_yolo(frame) if self.model else self._detect_color(frame)
        return [dict(cls=cls, box=box, **self._box_geometry(box)) for cls, box in boxes]

    def _detect_color(self, frame):
        blr = cv2.GaussianBlur(frame, (11, 11), 0)
        out = []
        for cls, (space, ranges) in TARGET_COLORS.items():
            img = cv2.cvtColor(blr, COLOR_CONV[space])
            mask = np.zeros((self.h, self.w), np.uint8)
            for lo, hi in ranges:
                mask |= cv2.inRange(img, np.array(lo, np.uint8), np.array(hi, np.uint8))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                continue
            cnt = max(contours, key=cv2.contourArea)
            if cv2.contourArea(cnt) >= TARGET_MIN_AREA:
                out.append((cls, cv2.boundingRect(cnt)))
        return out

    def _detect_yolo(self, frame):
        res = self.model.predict(source=frame, conf=YOLO_CONF, classes=list(YOLO_CLASSES), verbose=False)[0]
        best = {}
        for (x1, y1, x2, y2), c in zip(res.boxes.xyxy.tolist(), res.boxes.cls.tolist()):
            cls = YOLO_CLASSES[int(c)]
            box = (int(x1), int(y1), int(x2 - x1), int(y2 - y1))
            if cls not in best or box[2] * box[3] > best[cls][2] * best[cls][3]:
                best[cls] = box
        return list(best.items())

    def _box_geometry(self, box):
        bx, by, bw, bh = box
        u = bx + bw / 2
        bearing = -math.atan((u - self.w / 2) / self.f)         # 왼쪽이 +
        # 물체 아랫변이 바닥에 닿아 있다고 보고 바닥 평면으로 투영 → 거리
        # (LiDAR는 로봇 윗면 높이라 낮은 물체는 못 봄). 아랫변이 화면 끝에 잘리면 신뢰 불가
        v = by + bh
        dist = None
        if v < self.h - 2 and v - self.h / 2 > 5:
            fwd = CAM_HEIGHT * self.f / (v - self.h / 2)
            dist = math.hypot(fwd, fwd * (u - self.w / 2) / self.f)
        return {"bearing": bearing, "dist": dist}

    @staticmethod
    def draw(frame, dets):
        """검출 결과를 프레임에 그림 (tb3_teleop_cam처럼 창으로 확인용)"""
        for d in dets:
            x, y, w, h = d["box"]
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            txt = d["cls"] + (f" {d['dist']:.2f}m" if d["dist"] else "")
            cv2.putText(frame, txt, (x, max(15, y - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        return frame


def range_at(ranges, angles, bearing):
    diff = np.arctan2(np.sin(angles - bearing), np.cos(angles - bearing))
    idx = int(np.argmin(np.abs(diff)))
    win = ranges[max(0, idx - 2): idx + 3]
    win = win[np.isfinite(win)]
    return float(np.median(win)) if len(win) else None


def localize(det, ranges, angles, pose, max_range):
    """검출 하나 → 월드 좌표 (x, y). 거리를 못 구하면 None"""
    d_cam = det["dist"]
    d_lidar = range_at(ranges, angles, det["bearing"])
    # 대상이 LiDAR 높이까지 올라오면 LiDAR가 더 정확. 아니면 카메라 바닥 투영값 사용
    if d_cam is not None and d_lidar is not None and abs(d_lidar - d_cam) < 0.3:
        d = d_lidar
    else:
        d = d_cam
    if d is None or d >= max_range:
        return None
    th = pose[2] + det["bearing"]
    return pose[0] + d * math.cos(th), pose[1] + d * math.sin(th)


class TargetBook:
    """관측을 모아 대상 위치를 평균내고, CONFIRM_N번 이상 보인 것만 대상으로 확정"""

    def __init__(self):
        self.items = []          # {"cls", "x", "y", "n", "visited"}

    def add(self, x, y, cls):
        near = [t for t in self.items
                if t["cls"] == cls and math.hypot(x - t["x"], y - t["y"]) < MERGE_R]
        if near:
            t = min(near, key=lambda t: math.hypot(x - t["x"], y - t["y"]))
            n = min(t["n"], 20)  # 오래된 평균이 너무 굳지 않도록 가중치 상한
            t["x"], t["y"] = (t["x"] * n + x) / (n + 1), (t["y"] * n + y) / (n + 1)
            t["n"] += 1
            if t["n"] == CONFIRM_N:
                print(f"[FOUND] {cls} 대상 ({t['x']:.2f}, {t['y']:.2f}) / 확정 {len(self.confirmed())}개")
        else:
            self.items.append({"cls": cls, "x": x, "y": y, "n": 1, "visited": False})

    def confirmed(self):
        return [t for t in self.items if t["n"] >= CONFIRM_N]
