"""
[A] 위치 추정 + 지도
 - main은 Slam.update()를 매 step 부르고, .pose와 .grid만 읽는다.
"""
import math
import numpy as np
from config import (WHEEL_RADIUS, AXLE_LENGTH, MAP_SIZE_M, RES,
                    USE_COMPASS, COMPASS_SIGN, COMPASS_GAIN)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


# ------------------------- Odometry -------------------------
class Odometry:
    def __init__(self, x, y, yaw):
        self.x, self.y, self.yaw = x, y, yaw
        self.prev = None

    def update(self, enc_l, enc_r, gyro_z, dt):
        if self.prev is None:
            self.prev = (enc_l, enc_r)
            return
        dl = (enc_l - self.prev[0]) * WHEEL_RADIUS
        dr = (enc_r - self.prev[1]) * WHEEL_RADIUS
        self.prev = (enc_l, enc_r)
        d = (dl + dr) / 2
        # 회전량은 바퀴보다 자이로가 정확 (미끄러짐 영향 없음)
        dth = gyro_z * dt if gyro_z is not None else (dr - dl) / AXLE_LENGTH
        self.x += d * math.cos(self.yaw + dth / 2)
        self.y += d * math.sin(self.yaw + dth / 2)
        self.yaw = wrap(self.yaw + dth)


# ------------------------- Occupancy Grid -------------------------
L_FREE, L_OCC, L_MIN, L_MAX = -0.4, 0.9, -4.0, 4.0


class GridMap:
    def __init__(self, cx, cy):
        self.n = int(MAP_SIZE_M / RES)
        self.ox, self.oy = cx - MAP_SIZE_M / 2, cy - MAP_SIZE_M / 2
        self.lo = np.zeros((self.n, self.n), np.float32)   # log-odds

    def w2c(self, x, y):
        return int((y - self.oy) / RES), int((x - self.ox) / RES)   # (row, col)

    def c2w(self, r, c):
        return self.ox + (c + 0.5) * RES, self.oy + (r + 0.5) * RES

    def inside(self, r, c):
        return 0 <= r < self.n and 0 <= c < self.n

    def update(self, pose, ranges, angles, max_range):
        x, y, yaw = pose
        fr, fc, orr, oc = [], [], [], []
        for rng, a in zip(ranges, angles):
            if np.isnan(rng) or rng < 0.05:
                continue
            # inf = 최대 거리까지 아무것도 없음 → 빈 공간으로만 기록
            hit = np.isfinite(rng) and rng < max_range * 0.98
            rng = min(rng, max_range)
            th = yaw + a
            ts = np.arange(0, rng - RES, RES)          # 빔이 지나간 칸 = 빈 공간
            fr.append(((y + ts * math.sin(th) - self.oy) / RES).astype(int))
            fc.append(((x + ts * math.cos(th) - self.ox) / RES).astype(int))
            if hit:                                     # 빔이 끝난 칸 = 장애물
                orr.append(int((y + rng * math.sin(th) - self.oy) / RES))
                oc.append(int((x + rng * math.cos(th) - self.ox) / RES))
        if fr:
            r, c = np.concatenate(fr), np.concatenate(fc)
            m = (r >= 0) & (r < self.n) & (c >= 0) & (c < self.n)
            np.add.at(self.lo, (r[m], c[m]), L_FREE)
        if orr:
            r, c = np.array(orr), np.array(oc)
            m = (r >= 0) & (r < self.n) & (c >= 0) & (c < self.n)
            np.add.at(self.lo, (r[m], c[m]), L_OCC)
        np.clip(self.lo, L_MIN, L_MAX, out=self.lo)

    def occupancy(self):
        occ = np.full(self.lo.shape, -1, np.int8)      # -1 모름, 0 빈곳, 100 장애물
        occ[self.lo < -0.5] = 0
        occ[self.lo > 0.5] = 100
        return occ


# ------------------------- main이 쓰는 입구 -------------------------
class Slam:
    def __init__(self, x, y, yaw):
        self.odom = Odometry(x, y, yaw)
        self.grid = GridMap(x, y)
        self.compass_off = None

    @property
    def pose(self):
        return self.odom.x, self.odom.y, self.odom.yaw

    def update(self, enc_l, enc_r, gyro_z, compass, ranges, angles, max_range, dt):
        self.odom.update(enc_l, enc_r, gyro_z, dt)

        # compass 절대 heading 쪽으로 조금씩 당겨 자이로 드리프트 제거 (상보 필터)
        if USE_COMPASS and compass is not None:
            cy = COMPASS_SIGN * math.atan2(compass[1], compass[0])
            if self.compass_off is None:                # 시작 heading에 맞춰 보정값 고정
                self.compass_off = wrap(self.odom.yaw - cy)
            err = wrap(cy + self.compass_off - self.odom.yaw)
            self.odom.yaw = wrap(self.odom.yaw + COMPASS_GAIN * err)

        # TODO(A): scan matching(ICP) 보정은 여기서 self.odom.x/y/yaw를 고친다.
        #          운영진 제공 코드가 있으면 여기에 붙일 것.

        self.grid.update(self.pose, ranges[::2], angles[::2], max_range)
