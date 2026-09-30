"""
[A] 위치 추정 + 지도
 - main은 Slam.update()를 매 step 부르고, .pose와 .grid만 읽는다.
 - 강의의 SLAM 한 tick = Prediction → Scan-to-Map Update → Mapping
   1) Prediction : 엔코더 이동거리 + 자이로 회전량, compass 상보 필터로 헤딩 드리프트 제거
   2) Update     : scan-to-map 상관 매칭(likelihood field)으로 x, y 보정 (compass가 없으면 yaw도)
   3) Mapping    : 보정된 자세로 log-odds occupancy grid 갱신 (cv2.fillPoly 광선 투사)
"""
import math
import numpy as np
from config import (WHEEL_RADIUS, AXLE_LENGTH, MAP_SIZE_M, RES,
                    USE_COMPASS, COMPASS_SIGN, COMPASS_GAIN, COMPASS_GATE_DEG,
                    LIDAR_OFFSET_X, LIDAR_HALF_BEAM, SCAN_SKIP_S,
                    L_FREE, L_OCC, L_MIN, L_MAX, FREE_INF_RANGE,
                    MAP_INSERT_DIST, MAP_INSERT_ANG_DEG, MAP_INSERT_S,
                    USE_SCAN_MATCH, SCAN_MATCH_HZ, SM_WIN, SM_YAW_WIN_DEG,
                    SM_MIN_SCORE, SM_GAIN, SM_DEG_STD)

try:
    import cv2
except ImportError:          # cv2가 없으면 느린 광선 투사로 대체, scan matching은 끔
    cv2 = None

MATCH_OCC_T = 1.2            # 이 log-odds 이상(2번 이상 맞은 칸)만 매칭 기준으로 사용
DYN_FREE_T = -1.5            # 여러 번 빈칸이었던 곳에 새로 찍힌 점 = 움직이는 물체일 가능성
DYN_HIT_SCALE = 0.35         # 그런 점은 장애물 가중치를 줄여서 반영 (보행자 자국 방지)
ROBOT_CLEAR_R = 0.12         # 로봇이 서 있는 자리는 빈칸 [m]
SM_SIGMA = 0.05              # likelihood field 폭 [m]
SM_LUT_HALF = 4.5            # likelihood field 범위 ±[m] (LiDAR 최대 3.5 m + 여유)
SM_MIN_PTS = 60
SM_MAX_PTS = 180
SM_TEMP = 0.01               # 매칭 점수 → 불확실성(공분산) 계산용 온도


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _circ_mean(a):
    return math.atan2(float(np.mean(np.sin(a))), float(np.mean(np.cos(a))))


def _circ_std(a):
    r = math.hypot(float(np.mean(np.sin(a))), float(np.mean(np.cos(a))))
    return math.sqrt(max(-2.0 * math.log(min(max(r, 1e-12), 1.0)), 0.0)) + 0.0


def _bilinear(img, u, v):
    """img[v, u]를 이중선형 보간 (범위 밖은 0)"""
    h, w = img.shape
    ok = (u >= 0) & (v >= 0) & (u < w - 1) & (v < h - 1)
    u = np.clip(u, 0, w - 1.001)
    v = np.clip(v, 0, h - 1.001)
    u0, v0 = u.astype(np.int32), v.astype(np.int32)
    fu, fv = u - u0, v - v0
    val = (img[v0, u0] * (1 - fu) * (1 - fv) + img[v0, u0 + 1] * fu * (1 - fv)
           + img[v0 + 1, u0] * (1 - fu) * fv + img[v0 + 1, u0 + 1] * fu * fv)
    return np.where(ok, val, 0.0)


# ------------------------- Odometry -------------------------
class Odometry:
    """이동거리는 엔코더, 회전량은 밖에서 받음 (자이로/compass). 중점 적분"""

    def __init__(self, x, y, yaw):
        self.x, self.y, self.yaw = float(x), float(y), float(yaw)
        self.prev = None
        self.dist = 0.0                                 # 누적 주행거리 [m]

    def wheels(self, enc_l, enc_r):
        """엔코더 변화 → (이동거리, 바퀴 기준 회전량). 첫 호출이거나 값이 이상하면 None"""
        if enc_l is None or enc_r is None or not (math.isfinite(enc_l) and math.isfinite(enc_r)):
            return None
        if self.prev is None:
            self.prev = (enc_l, enc_r)
            return None
        dl = (enc_l - self.prev[0]) * WHEEL_RADIUS
        dr = (enc_r - self.prev[1]) * WHEEL_RADIUS
        self.prev = (enc_l, enc_r)
        return (dl + dr) / 2, (dr - dl) / AXLE_LENGTH

    def advance(self, d, dth):
        self.x += d * math.cos(self.yaw + dth / 2)
        self.y += d * math.sin(self.yaw + dth / 2)
        self.yaw = wrap(self.yaw + dth)
        self.dist += abs(d)

    def update(self, enc_l, enc_r, gyro_z, dt):
        """(이전 인터페이스) 자이로가 있으면 자이로 회전, 없으면 엔코더 회전"""
        w = self.wheels(enc_l, enc_r)
        if w is not None:
            self.advance(w[0], gyro_z * dt if gyro_z is not None else w[1])


# ------------------------- Occupancy Grid -------------------------
class GridMap:
    """log-odds 점유 격자. lo: log-odds, obs: 한 번이라도 관측된 칸"""

    def __init__(self, cx, cy):
        self.n = int(round(MAP_SIZE_M / RES))
        self.ox, self.oy = cx - MAP_SIZE_M / 2, cy - MAP_SIZE_M / 2
        self.lo = np.zeros((self.n, self.n), np.float32)   # log-odds (0 = 모름)
        self.obs = np.zeros((self.n, self.n), bool)
        self.lut = None                                      # scan matching용 likelihood field
        self.lut_org = (0, 0)                                # lut[0, 0]의 (row, col)
        self.lut_occ = 0

    def w2c(self, x, y):
        return int(math.floor((y - self.oy) / RES)), int(math.floor((x - self.ox) / RES))   # (row, col)

    def c2w(self, r, c):
        return self.ox + (c + 0.5) * RES, self.oy + (r + 0.5) * RES

    def inside(self, r, c):
        return 0 <= r < self.n and 0 <= c < self.n

    def _window(self, x, y, half_m):
        r, c = self.w2c(x, y)
        h = int(math.ceil(half_m / RES)) + 2
        r0, r1 = max(r - h, 0), min(r + h + 1, self.n)
        c0, c1 = max(c - h, 0), min(c + h + 1, self.n)
        return r0, r1, c0, c1

    def update(self, pose, ranges, angles, max_range):
        """pose = LiDAR 원점 (x, y, yaw). ranges는 Webots 원시값 (inf, 0, nan 포함 가능)"""
        x, y, yaw = pose
        r = np.asarray(ranges, dtype=float)
        a = np.asarray(angles, dtype=float) + yaw
        fin = np.isfinite(r) & (r > 0.05)
        hit = fin & (r < max_range * 0.98)
        nohit = (np.isinf(r) & (r > 0)) | (fin & ~hit)
        # inf 주변에 아주 가까운 빔이 있으면 '0.12 m 미만'이었을 수 있음 → 그 방향은 비우지 않음
        near = fin & (r < 0.5)
        amb = near.copy()
        for k in (-2, -1, 1, 2):
            amb |= np.roll(near, k)
        free_len = np.zeros_like(r)
        free_len[hit] = np.maximum(r[hit] - 1.5 * RES, 0.0)     # 끝점 바로 앞까지만 비움 (얇은 다리 보호)
        free_len[nohit & ~amb] = min(FREE_INF_RANGE, max_range)
        ca, sa = np.cos(a), np.sin(a)

        r0, r1, c0, c1 = self._window(x, y, max_range + 0.2)
        if r1 <= r0 or c1 <= c0:
            return
        H, W = r1 - r0, c1 - c0
        hr = np.floor((y + r[hit] * sa[hit] - self.oy) / RES).astype(np.int64) - r0
        hc = np.floor((x + r[hit] * ca[hit] - self.ox) / RES).astype(np.int64) - c0
        ok = (hr >= 0) & (hr < H) & (hc >= 0) & (hc < W)
        hitm = np.zeros((H, W), bool)
        hitm[hr[ok], hc[ok]] = True

        if cv2 is not None:
            # 빔 끝점들로 만든 별 모양 다각형 = 이번 스캔에서 비어 있음이 확인된 영역
            px = (x + free_len * ca - self.ox) / RES - 0.5 - c0
            py = (y + free_len * sa - self.oy) / RES - 0.5 - r0
            ox_, oy_ = (x - self.ox) / RES - 0.5 - c0, (y - self.oy) / RES - 0.5 - r0
            poly = np.vstack([np.stack([px, py], 1), [[ox_, oy_]]])
            poly = np.round(poly * 8.0).astype(np.int32)
            m = np.zeros((H, W), np.uint8)
            cv2.fillPoly(m, [poly], 1, lineType=cv2.LINE_8, shift=3)
            seen = m.astype(bool)
            near_hit = cv2.dilate(hitm.astype(np.uint8), np.ones((3, 3), np.uint8)) > 0
        else:
            seen = np.zeros((H, W), bool)
            for L, cc, ss in zip(free_len, ca, sa):
                ts = np.arange(0.0, L, RES * 0.5)
                rr = np.floor((y + ts * ss - self.oy) / RES).astype(np.int64) - r0
                cl = np.floor((x + ts * cc - self.ox) / RES).astype(np.int64) - c0
                k = (rr >= 0) & (rr < H) & (cl >= 0) & (cl < W)
                seen[rr[k], cl[k]] = True
            near_hit = hitm.copy()
            near_hit[1:, :] |= hitm[:-1, :]
            near_hit[:-1, :] |= hitm[1:, :]
            near_hit[:, 1:] |= hitm[:, :-1]
            near_hit[:, :-1] |= hitm[:, 1:]

        free = seen & ~near_hit                     # 한 칸에 한 스캔당 한 번만 갱신 (np.add.at 중복 누적 방지)
        lo = self.lo[r0:r1, c0:c1]
        dyn = hitm & (lo < DYN_FREE_T)              # 늘 비어 있던 곳에 새로 생긴 점 → 보행자일 가능성
        lo[free] += L_FREE
        lo[hitm & ~dyn] += L_OCC
        lo[dyn] += L_OCC * DYN_HIT_SCALE
        np.clip(lo, L_MIN, L_MAX, out=lo)
        self.obs[r0:r1, c0:c1] |= seen | hitm

    def clear_disc(self, x, y, rad):
        """로봇이 서 있는 원 안은 빈칸"""
        r, c = self.w2c(x, y)
        k = max(int(rad / RES), 1)
        r0, r1 = max(r - k, 0), min(r + k + 1, self.n)
        c0, c1 = max(c - k, 0), min(c + k + 1, self.n)
        if r1 <= r0 or c1 <= c0:
            return
        yy, xx = np.mgrid[r0:r1, c0:c1]
        m = (yy - r) ** 2 + (xx - c) ** 2 <= k * k
        lo = self.lo[r0:r1, c0:c1]
        lo[m] = np.minimum(lo[m], L_FREE)
        self.obs[r0:r1, c0:c1] |= m

    def occupancy(self):
        """-1 모름 / 0 빈칸 / 100 장애물. 관측된 칸은 log-odds 부호로 판정 (애매하면 장애물 쪽)"""
        occ = np.full(self.lo.shape, -1, np.int8)
        occ[self.obs] = 0
        occ[self.obs & (self.lo > 0)] = 100
        return occ

    # ---------- scan matching ----------
    def build_likelihood(self, x, y):
        """(x, y) 주변 확실한 장애물 칸으로부터의 거리 d → exp(-d²/2σ²)"""
        self.lut = None
        if cv2 is None:
            return
        r0, r1, c0, c1 = self._window(x, y, SM_LUT_HALF)
        occ = self.lo[r0:r1, c0:c1] > MATCH_OCC_T
        self.lut_occ = int(occ.sum())
        if self.lut_occ < 40:
            return
        d = cv2.distanceTransform((~occ).astype(np.uint8), cv2.DIST_L2, 5) * RES
        self.lut = np.exp(-(d * d) / (2 * SM_SIGMA ** 2)).astype(np.float32)
        self.lut_org = (r0, c0)

    def match(self, pose, ranges, angles, max_range, yaw_win, win=SM_WIN, step=0.02):
        """LiDAR 원점 pose 근처(±win)에서 스캔이 지도에 가장 잘 겹치는 보정량 탐색 (조→정 2단계).
        반환 dict(dx, dy, dth, score, s0, edge, cov) 또는 건너뛴 이유(str)"""
        if self.lut is None:
            return "no map"
        r = np.asarray(ranges, dtype=float)
        fin = np.isfinite(r) & (r > 0.15) & (r < max_range - 0.1)
        n_fin = int(fin.sum())
        if n_fin < SM_MIN_PTS:
            return "few points"
        if np.count_nonzero(fin & (r < 0.3)) > 0.25 * n_fin:
            return "crowded"                         # 바로 옆에 사람/물체가 시야를 가림
        idx = np.nonzero(fin)[0]
        if len(idx) > SM_MAX_PTS:
            idx = idx[::int(math.ceil(len(idx) / SM_MAX_PTS))]
        a = np.asarray(angles, dtype=float)[idx]
        px, py = r[idx] * np.cos(a), r[idx] * np.sin(a)
        x, y, yaw = pose
        r0, c0 = self.lut_org
        lut = self.lut

        def to_uv(dth):
            c, s = math.cos(yaw + dth), math.sin(yaw + dth)
            u = (x + c * px - s * py - self.ox) / RES - 0.5 - c0
            v = (y + s * px + c * py - self.oy) / RES - 0.5 - r0
            return u, v

        def scores(u, v, DX, DY):
            sc = _bilinear(lut, u[None, :] + DX[:, None] / RES, v[None, :] + DY[:, None] / RES).mean(1)
            return sc - 0.02 * (DX ** 2 + DY ** 2) / win ** 2        # 예측(오도메트리) 근처를 약하게 선호

        u0, v0 = to_uv(0.0)
        s0 = float(_bilinear(lut, u0, v0).mean())
        g = np.arange(-win, win + 1e-9, step)
        DX, DY = [q.ravel() for q in np.meshgrid(g, g)]
        yaws = [0.0] if yaw_win <= 0 else list(np.arange(-yaw_win, yaw_win + 1e-9, math.radians(1.0)))
        best = None
        for dth in yaws:
            u, v = to_uv(dth)
            sc = scores(u, v, DX, DY)
            k = int(np.argmax(sc))
            if best is None or sc[k] > best[0]:
                best = (float(sc[k]), dth, k, sc)
        _, bth, bk, bsc = best
        # 점수 분포 → 위치 불확실성 (긴 복도처럼 한 방향으로 평평하면 그 방향 std가 커짐)
        p = np.exp((bsc - bsc.max()) / SM_TEMP)
        p /= p.sum()
        mx, my = float((p * DX).sum()), float((p * DY).sum())
        cov = np.array([[(p * (DX - mx) ** 2).sum(), (p * (DX - mx) * (DY - my)).sum()],
                        [(p * (DX - mx) * (DY - my)).sum(), (p * (DY - my) ** 2).sum()]])
        # 정밀 탐색
        f = np.arange(-step, step + 1e-9, 0.005)
        FX, FY = [q.ravel() for q in np.meshgrid(f, f)]
        FX, FY = FX + DX[bk], FY + DY[bk]
        fyaws = [bth] if yaw_win <= 0 else [bth + d for d in np.radians([-0.5, -0.25, 0.0, 0.25, 0.5])]
        fbest = None
        for dth in fyaws:
            u, v = to_uv(dth)
            sc = scores(u, v, FX, FY)
            k = int(np.argmax(sc))
            if fbest is None or sc[k] > fbest[0]:
                fbest = (float(sc[k]), dth, float(FX[k]), float(FY[k]))
        score, dth, dx, dy = fbest
        # x, y가 탐색 경계에 붙으면 진짜 최대가 밖에 있을 수 있음 → 버림 (yaw는 경계여도 방향이 맞으므로 사용)
        edge = max(abs(dx), abs(dy)) > win - 1e-6
        return dict(dx=dx, dy=dy, dth=dth, score=score, s0=s0, edge=edge, cov=cov)


# ------------------------- main이 쓰는 입구 -------------------------
class Slam:
    def __init__(self, x, y, yaw):
        self.odom = Odometry(x, y, yaw)
        self.grid = GridMap(x, y)
        self.start = (float(x), float(y), float(yaw))
        self.t = 0.0
        self.moved = False                  # 시작 후 한 번이라도 움직였는지 (그 전 = 센서 보정 구간)
        self.gyro_bias = 0.0
        self.compass_off = None
        self.compass_gain = COMPASS_GAIN
        self.compass_noisy = False          # 정지 중 compass 흔들림이 크면 True → 매칭이 yaw도 보정
        self._g_samples, self._c_samples = [], []
        self._gate_n = 0
        self._ang_key, self._ang = None, None
        self._prev_ranges = None
        self._last_insert = None
        self._sm_t = -1e9
        self._edge_n = 0
        self.trail = []                     # 지나온 자세 (5 cm 간격) — 시각화용
        self.stats = dict(heading="-", compass="off", gyro_bias=0.0, compass_std_deg=None,
                          sm_ok=0, sm_rej=0, sm_skip=0, sm_score=0.0, sm_last=(0.0, 0.0, 0.0),
                          sm_reason="", inserts=0)

    @property
    def pose(self):
        return self.odom.x, self.odom.y, self.odom.yaw

    # ---------- 센서 전처리 ----------
    def _compass_yaw(self, compass):
        if not USE_COMPASS or compass is None or len(compass) < 2:
            return None
        cx, cy = compass[0], compass[1]
        if not (math.isfinite(cx) and math.isfinite(cy)) or math.hypot(cx, cy) < 1e-6:
            return None
        return COMPASS_SIGN * math.atan2(cx, cy)

    def _fix_angles(self, angles):
        """main의 angles = fov/2 - i·Δ 이면 반 칸 보정 (Webots 실제 빔 각도 = fov/2 - (i+0.5)·Δ)"""
        key = (angles.size, float(angles[0]), float(angles[-1]))
        if key != self._ang_key:
            out = angles
            if LIDAR_HALF_BEAM and angles.size > 3:
                d = float(np.median(np.diff(angles)))
                full = abs(abs(d) * angles.size - 2 * math.pi) < 1e-3
                at_edge = abs(abs(float(angles[0])) - math.pi) < 1e-3   # 보정 전 = ±π (Webots fov 6.28318), 보정 후 = ±(π - Δ/2)
                if full and at_edge:
                    out = angles + d / 2
            self._ang_key, self._ang = key, out
        return self._ang

    # ---------- 매 step ----------
    def update(self, enc_l, enc_r, gyro_z, compass, ranges, angles, max_range, dt):
        self.t += dt
        o = self.odom
        wh = o.wheels(enc_l, enc_r)
        gz = gyro_z if (gyro_z is not None and math.isfinite(gyro_z)) else None
        cy = self._compass_yaw(compass)

        # ---- 시작 정지 구간: 자이로 bias, compass 오프셋/노이즈 추정 ----
        if wh is not None and (abs(wh[0]) > 1e-4 or abs(wh[1]) > 1e-3):
            if not self.moved:
                self.moved = True
                self._on_start_moving()
        if not self.moved:
            if gz is not None:
                self._g_samples.append(gz)
                self.gyro_bias = float(np.mean(self._g_samples[-200:]))
            if cy is not None:
                self._c_samples.append(cy)
                # 시작 heading(START_YAW)에 compass를 맞춤 → 월드 좌표계/북쪽 방향이 달라도 동작
                self.compass_off = wrap(self.start[2] - _circ_mean(self._c_samples[-200:]))
        elif cy is not None and self.compass_off is None:
            self.compass_off = wrap(o.yaw - cy)
        c_yaw = wrap(cy + self.compass_off) if (cy is not None and self.compass_off is not None) else None

        # ---- 1) Prediction: 거리 = 엔코더, 회전 = 자이로 > compass > 엔코더 ----
        if wh is not None:
            d, dth_enc = wh
            if gz is not None:
                # Webots 자이로 값 = 그 step 끝의 각속도. 모터 명령이 즉시 반영되므로 직사각형 적분이 실측과 가장 잘 맞음
                dth, src = (gz - self.gyro_bias) * dt, "gyro"
            elif c_yaw is not None:
                dth, src = wrap(c_yaw - o.yaw), "compass"
            else:
                dth, src = dth_enc, "encoder"
            o.advance(d, dth)
            self.stats["heading"] = src + ("+compass" if (src == "gyro" and c_yaw is not None) else "")

        # compass 상보 필터 (PI): 자이로 적분 헤딩을 절대 heading 쪽으로 당기고(P),
        # 남는 오차로 자이로 bias를 계속 추정(I, 임계 감쇠 ki = gain²/(4·dt)) → 주행 중 생기는 bias도 제거
        if c_yaw is not None and gz is not None:
            err = wrap(c_yaw - o.yaw)
            if abs(err) < math.radians(COMPASS_GATE_DEG) or self._gate_n * dt > 3.0:
                o.yaw = wrap(o.yaw + self.compass_gain * err)
                if self.moved and dt > 0:
                    ki = self.compass_gain ** 2 / (4.0 * dt)
                    self.gyro_bias = min(max(self.gyro_bias - ki * err, -0.05), 0.05)
                self._gate_n = 0
            else:
                self._gate_n += 1                           # 순간적인 교란은 무시
        elif gz is not None and self.moved and wh is not None and abs(wh[0]) < 1e-6 and abs(wh[1]) < 1e-5:
            self.gyro_bias += 0.02 * (gz - self.gyro_bias)  # compass 없음: 바퀴가 멈춰 있을 때만 bias 갱신

        # ---- LiDAR ----
        rng = np.asarray(ranges, dtype=float)
        if rng.size < 4:
            return
        ang = self._fix_angles(np.asarray(angles, dtype=float))
        fresh = (self._prev_ranges is None or self._prev_ranges.shape != rng.shape
                 or not np.array_equal(rng, self._prev_ranges, equal_nan=True))
        self._prev_ranges = rng.copy()
        n_valid = int(np.count_nonzero(np.isfinite(rng) & (rng > 0.05)))
        if self.t >= SCAN_SKIP_S and fresh and n_valid >= 10:
            # ---- 2) Update: scan-to-map 매칭 ----
            if USE_SCAN_MATCH and cv2 is not None and self.t - self._sm_t >= 1.0 / SCAN_MATCH_HZ:
                self._sm_t = self.t
                if c_yaw is not None and not self.compass_noisy:
                    yaw_win = 0.0                           # compass가 헤딩을 잡아주면 x, y만
                elif gz is not None:
                    yaw_win = math.radians(SM_YAW_WIN_DEG)
                else:
                    yaw_win = math.radians(max(SM_YAW_WIN_DEG, 5.0))
                self._scan_match(rng, ang, max_range, yaw_win)
            # ---- 3) Mapping ----
            if self._insert_due():
                x, y, yaw = self.pose
                self.grid.update(self._lidar_pose(), rng, ang, max_range)
                self.grid.clear_disc(x, y, ROBOT_CLEAR_R)
                lx, ly, _ = self._lidar_pose()
                self.grid.build_likelihood(lx, ly)
                self._last_insert = (x, y, yaw, self.t)
                self.stats["inserts"] += 1

        if not self.trail or math.hypot(o.x - self.trail[-1][0], o.y - self.trail[-1][1]) > 0.05:
            self.trail.append((o.x, o.y))
            if len(self.trail) > 20000:
                del self.trail[:5000]

    # ---------- 내부 ----------
    def _lidar_pose(self):
        x, y, yaw = self.pose
        return x + LIDAR_OFFSET_X * math.cos(yaw), y + LIDAR_OFFSET_X * math.sin(yaw), yaw

    def _insert_due(self):
        li = self._last_insert
        if li is None:
            return True
        x, y, yaw = self.pose
        return (math.hypot(x - li[0], y - li[1]) > MAP_INSERT_DIST
                or abs(wrap(yaw - li[2])) > math.radians(MAP_INSERT_ANG_DEG)
                or self.t - li[3] > MAP_INSERT_S)

    def _scan_match(self, rng, ang, max_range, yaw_win):
        st = self.stats
        res = self.grid.match(self._lidar_pose(), rng, ang, max_range, yaw_win)
        if isinstance(res, str):
            st["sm_skip"] += 1
            st["sm_reason"] = res
            return
        st["sm_score"] = res["score"]
        if res["edge"] and res["score"] >= SM_MIN_SCORE:
            self._edge_n += 1
            if self._edge_n >= 4:
                # 좁은 창 경계에서 계속 걸림 = 충돌·미끄러짐으로 예측이 10 cm 넘게 틀림 → 한 번 넓게(±3배) 찾아봄
                self._edge_n = 0
                wide = self.grid.match(self._lidar_pose(), rng, ang, max_range, yaw_win, win=3 * SM_WIN, step=0.04)
                if (not isinstance(wide, str) and not wide["edge"] and wide["score"] >= SM_MIN_SCORE
                        and wide["score"] - wide["s0"] > 0.1):
                    res = wide
                    st["sm_wide"] = st.get("sm_wide", 0) + 1
        else:
            self._edge_n = 0
        if res["score"] < SM_MIN_SCORE or res["edge"]:
            st["sm_rej"] += 1
            st["sm_reason"] = "low score" if res["score"] < SM_MIN_SCORE else "edge"
            return
        # 불확실한 방향(복도 길이 방향 등)의 보정은 빼고 잘 잡힌 방향만 반영
        lam, vec = np.linalg.eigh(res["cov"])
        good = np.sqrt(np.maximum(lam, 0.0)) < SM_DEG_STD
        if not good.any():
            st["sm_rej"] += 1
            st["sm_reason"] = "flat"
            return
        delta = np.array([res["dx"], res["dy"]])
        corr = np.zeros(2)
        for k in range(2):
            if good[k]:
                corr += float(delta @ vec[:, k]) * vec[:, k]
        o = self.odom
        o.x += SM_GAIN * corr[0]
        o.y += SM_GAIN * corr[1]
        if yaw_win > 0:
            o.yaw = wrap(o.yaw + SM_GAIN * res["dth"])
        st["sm_ok"] += 1
        st["sm_last"] = (float(corr[0]), float(corr[1]), float(res["dth"]) if yaw_win > 0 else 0.0)
        st["sm_reason"] = "ok" if good.all() else "ok (1 axis)"

    def _on_start_moving(self):
        """처음 움직이기 시작할 때 정지 구간에서 모은 센서 통계 확정"""
        st = self.stats
        st["gyro_bias"] = self.gyro_bias
        if len(self._c_samples) >= 8:
            sd = math.degrees(_circ_std(np.array(self._c_samples[-200:])))
            st["compass_std_deg"] = sd
            if sd > 1.5:                                    # 당일 로봇 compass에 노이즈가 있으면 덜 믿음
                self.compass_gain = COMPASS_GAIN * 0.2
                self.compass_noisy = True
        st["compass"] = "on (gain %.3f)" % self.compass_gain if self.compass_off is not None else "off"
        print("[SLAM] 센서 보정: gyro bias %.2e rad/s, compass %s%s"
              % (self.gyro_bias, st["compass"],
                 "" if st["compass_std_deg"] is None else ", 정지 중 std %.2f deg" % st["compass_std_deg"]))

    def status(self):
        """시각화/로그용 한 줄 요약"""
        s = self.stats
        return ("heading %s | scan match ok %d rej %d skip %d (%s, score %.2f) | last %.1f,%.1f cm | map %d"
                % (s["heading"], s["sm_ok"], s["sm_rej"], s["sm_skip"], s["sm_reason"], s["sm_score"],
                   s["sm_last"][0] * 100, s["sm_last"][1] * 100, s["inserts"]))
