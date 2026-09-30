import math
import cv2
import numpy as np

from config import (
    DETECTOR, TARGET_COLORS, YOLO_MODEL, YOLO_CLASSES, YOLO_CONF,
    TARGET_MIN_AREA, CONFIRM_N, MERGE_R, CAM_HEIGHT
)

TARGET_CLASS = "apple"
RED_RATIO_THRESHOLD = 0.35
LIDAR_CORRIDOR_HALF_ANGLE = math.radians(10)
OBSTACLE_MARGIN = 0.25
LIDAR_MIN_VALID_RANGE = 0.05


class Perception:
    def __init__(self, camera):
        self.cam = camera
        self.w, self.h = camera.getWidth(), camera.getHeight()
        self.f = (self.w / 2) / math.tan(camera.getFov() / 2)

        from ultralytics import YOLO
        self.model = YOLO(YOLO_MODEL)
        self.model.to("cpu")

    def grab(self):
        frame = np.frombuffer(
            self.cam.getImage(), np.uint8
        ).reshape((self.h, self.w, 4))
        return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

    def detect(self, frame):
        raw = self._detect_yolo(frame)
        out = []

        for item in raw:
            d = {
                "cls": item["cls"],
                "box": item["box"],
                "color": item["color"],
                "red_ratio": item["red_ratio"],
                "mask": item["mask"]
            }
            d.update(self._box_geometry(item["box"]))
            out.append(d)

        return out

    def _detect_yolo(self, frame):
        res = self.model.predict(
            source=frame,
            conf=YOLO_CONF,
            verbose=False
        )[0]
        self.prediction_frame = res.plot()

        if res.boxes is None:
            return []

        boxes = res.boxes.xyxy.cpu().numpy()
        classes = res.boxes.cls.cpu().numpy().astype(int)

        best = None
        best_area = 0

        for xyxy, class_id in zip(boxes, classes):
            cls = self._class_name(class_id)

            if cls != TARGET_CLASS:
                continue

            x1, y1, x2, y2 = xyxy
            x1 = max(0, int(x1))
            y1 = max(0, int(y1))
            x2 = min(self.w, int(x2))
            y2 = min(self.h, int(y2))

            bw, bh = x2 - x1, y2 - y1

            if bw <= 0 or bh <= 0:
                continue

            area = bw * bh

            if area < TARGET_MIN_AREA:
                continue

            # ponytail: box includes background; tune ratio or use segmentation if needed.
            color, red_ratio = self._detect_color(frame[y1:y2, x1:x2], np.ones((bh, bw), dtype=bool))

            if color != "red":
                continue

            box_area = bw * bh

            if box_area > best_area:
                best_area = box_area
                best = {
                    "cls": cls,
                    "box": (x1, y1, bw, bh),
                    "color": color,
                    "red_ratio": red_ratio,
                    "mask": None
                }

        return [best] if best is not None else []

    def _class_name(self, class_id):
        if isinstance(YOLO_CLASSES, dict) and class_id in YOLO_CLASSES:
            return YOLO_CLASSES[class_id]

        names = self.model.names

        if isinstance(names, dict):
            return names.get(class_id, str(class_id))

        return names[class_id] if 0 <= class_id < len(names) else str(class_id)

    def _detect_color(self, frame, object_mask):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        red1 = cv2.inRange(
            hsv,
            np.array((0, 80, 50), np.uint8),
            np.array((12, 255, 255), np.uint8)
        )

        red2 = cv2.inRange(
            hsv,
            np.array((168, 80, 50), np.uint8),
            np.array((179, 255, 255), np.uint8)
        )

        red_mask = (red1 | red2) > 0
        object_pixels = np.count_nonzero(object_mask)

        if object_pixels == 0:
            return "unknown", 0.0

        red_pixels = np.count_nonzero(red_mask & object_mask)
        ratio = red_pixels / object_pixels

        if ratio >= RED_RATIO_THRESHOLD:
            return "red", ratio

        return "other", ratio

    def _box_geometry(self, box):
        bx, by, bw, bh = box
        u = bx + bw / 2
        bearing = -math.atan((u - self.w / 2) / self.f)

        v = by + bh
        dist = None

        if v < self.h - 2 and v - self.h / 2 > 5:
            fwd = CAM_HEIGHT * self.f / (v - self.h / 2)
            lateral = fwd * (u - self.w / 2) / self.f
            dist = math.hypot(fwd, lateral)

        return {"bearing": bearing, "dist": dist}

    @staticmethod
    def draw(frame, dets):
        output = frame.copy()

        for d in dets:
            mask = d.get("mask")

            if mask is not None:
                overlay = output.copy()
                overlay[mask] = (0, 0, 255)
                output = cv2.addWeighted(output, 0.75, overlay, 0.25, 0)

            x, y, w, h = d["box"]

            cv2.rectangle(
                output,
                (x, y),
                (x + w, y + h),
                (0, 255, 0),
                2
            )

            dist_txt = (
                f"{d['dist']:.2f}m"
                if d["dist"] is not None
                else "dist:N/A"
            )

            txt = (
                f"{d['cls']} | "
                f"{d.get('color', 'unknown')} "
                f"{d.get('red_ratio', 0.0) * 100:.0f}% | "
                f"{dist_txt}"
            )

            cv2.putText(
                output,
                txt,
                (x, max(20, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2
            )

            cx = x + w // 2
            cv2.line(
                output,
                (output.shape[1] // 2, output.shape[0] // 2),
                (cx, y + h // 2),
                (255, 255, 0),
                1
            )

        cv2.line(
            output,
            (output.shape[1] // 2, 0),
            (output.shape[1] // 2, output.shape[0]),
            (255, 0, 255),
            1
        )

        return output


def _angle_diff(a, b):
    return np.arctan2(np.sin(a - b), np.cos(a - b))


def range_at(ranges, angles, bearing):
    ranges = np.asarray(ranges, dtype=float)
    angles = np.asarray(angles, dtype=float)

    diff = _angle_diff(angles, bearing)
    idx = int(np.argmin(np.abs(diff)))

    lo = max(0, idx - 2)
    hi = min(len(ranges), idx + 3)

    win = ranges[lo:hi]
    win = win[
        np.isfinite(win) &
        (win > LIDAR_MIN_VALID_RANGE)
    ]

    return float(np.median(win)) if len(win) else None


def _path_clear(ranges, angles, bearing, target_dist):
    ranges = np.asarray(ranges, dtype=float)
    angles = np.asarray(angles, dtype=float)

    diff = np.abs(_angle_diff(angles, bearing))
    corridor = diff <= LIDAR_CORRIDOR_HALF_ANGLE

    if not np.any(corridor):
        return False

    scan = ranges[corridor]
    scan = scan[
        np.isfinite(scan) &
        (scan > LIDAR_MIN_VALID_RANGE)
    ]

    if len(scan) == 0:
        return True

    nearest = float(np.min(scan))
    return nearest >= target_dist - OBSTACLE_MARGIN


def localize(det, ranges, angles, pose, max_range):
    d_cam = det["dist"]

    if (
        d_cam is None or
        not math.isfinite(d_cam) or
        d_cam <= 0 or
        d_cam >= max_range
    ):
        return None

    bearing = det["bearing"]

    if not _path_clear(
        ranges,
        angles,
        bearing,
        d_cam
    ):
        return None

    d_lidar = range_at(
        ranges,
        angles,
        bearing
    )

    if (
        d_lidar is not None and
        abs(d_lidar - d_cam) < 0.3
    ):
        d = d_lidar
    else:
        d = d_cam

    th = pose[2] + bearing

    return (
        pose[0] + d * math.cos(th),
        pose[1] + d * math.sin(th)
    )


class TargetBook:
    def __init__(self):
        self.items = []

    def add(self, x, y, cls):
        near = [
            t for t in self.items
            if (
                t["cls"] == cls and
                math.hypot(x - t["x"], y - t["y"]) < MERGE_R
            )
        ]

        if near:
            t = min(
                near,
                key=lambda t: math.hypot(
                    x - t["x"],
                    y - t["y"]
                )
            )

            n = min(t["n"], 20)

            t["x"] = (t["x"] * n + x) / (n + 1)
            t["y"] = (t["y"] * n + y) / (n + 1)
            t["n"] += 1

            if t["n"] == CONFIRM_N:
                print(
                    f"[FOUND] {cls} 대상 "
                    f"({t['x']:.2f}, {t['y']:.2f}) "
                    f"/ 확정 {len(self.confirmed())}개"
                )
        else:
            self.items.append({
                "cls": cls,
                "x": x,
                "y": y,
                "n": 1,
                "visited": False
            })

    def confirmed(self):
        return [
            t for t in self.items
            if t["n"] >= CONFIRM_N
        ]
