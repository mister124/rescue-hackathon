"""
[B] 탐색 목표 선정 + 경로 계획 + 경로 추종 + 충돌 회피
 - main은 make_blocks() → pick_frontier()/plan() 으로 경로를 받고,
   매 step Follower.step()으로 (v, w)를 받는다.
"""
import math
import heapq
import time
from collections import deque
import numpy as np
import cv2
from config import (RES, INFLATE_M, INFLATE_MIN_M, ROBOT_RADIUS, V_MAX, W_MAX,
                    SAFE_DIST, LOOKAHEAD, STUCK_S, GOAL_TOL)


# ------------------------- Costmap -------------------------
def inflate(obst, radius_cells):
    """장애물 지도에 radius_cells만큼 팽창한 장애물을 반환함. obst: bool 2D array"""
    out = obst.copy()
    R = radius_cells
    for dr in range(-R, R + 1):
        for dc in range(-R, R + 1):
            if dr * dr + dc * dc <= R * R and (dr or dc):
                out |= np.roll(obst, (dr, dc), axis=(0, 1))
    return out


def make_blocks(occ):
    """(넉넉한 안전거리 지도, 최소 안전거리 지도, 탈출용 지도). 경로 계획은 앞에 것부터 시도.
    탈출용(로봇 반경만)은 좁은 틈에 갇혔을 때 빠져나오는 최후 수단. frontier 선택에는 안 씀"""
    obst = occ == 100
    return (inflate(obst, int(math.ceil(INFLATE_M / RES))),
            inflate(obst, int(math.ceil(INFLATE_MIN_M / RES))),
            inflate(obst, int(ROBOT_RADIUS / RES)))


# ------------------------- Frontier -------------------------
def find_frontiers(occ, blocked):
    """occ: occupancy grid. blocked: 경로 계획에서 막힌 칸. 반환: (r, c) frontier 좌표 배열"""
    free, unk = occ == 0, occ == -1
    nb_unk = np.zeros_like(unk)
    for d in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        nb_unk |= np.roll(unk, d, axis=(0, 1))
    fr = free & nb_unk & ~blocked
    cnt = np.zeros(fr.shape, np.int16)            # 고립된 잡음 frontier 제거
    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            cnt += np.roll(fr, (dr, dc), axis=(0, 1))
    return np.argwhere(fr & (cnt >= 3))

MIN_FRONTIER_CELLS = 5          # 이보다 작은 frontier 덩어리는 잡음(가구 밑 틈 등)으로 보고 무시

NO_GO = []                      # 막혔던 위치 (x, y, 등록 시각). 경로 계획과 frontier 선택에서 장애물로 취급
NO_GO_R = 0.35                  # 진입 금지 반경 [m]
NO_GO_TTL = 60.0                # 진입 금지 유지 시간 [s] (보행자 때문에 막힌 문이 영원히 닫히지 않게)

def with_no_go(grid, blocks):
    """blocks에 진입 금지 구역(NO_GO)을 원으로 칠한 사본
    (침대 밑처럼 LiDAR에는 빈 공간인데 들어가면 막히는 곳에 다시 들어가지 않게)"""
    NO_GO[:] = [e for e in NO_GO if len(e) < 3 or time.monotonic() - e[2] < NO_GO_TTL]
    if not NO_GO:
        return blocks
    out = tuple(b.copy() for b in blocks)
    rad = int(math.ceil(NO_GO_R / RES))
    for x, y, *_ in NO_GO:
        r, c = grid.w2c(x, y)
        for b in out:
            cv2.circle(b.view(np.uint8), (c, r), rad, 1, -1)
    return out


def pick_frontier(grid, occ, blocks, pose, blacklist):
    """frontier를 덩어리로 묶고, 가까운 덩어리부터 경로가 나오는 것을 고름.
    덩어리마다 중심에 가장 가까운 칸이 대표점. 가던 목표가 남아 있으면 계속 그쪽.
    반환 (goal, path) / 없으면 (None, [])"""
    walls = blocks[-1]                          # 실제 벽 판정용은 진입 금지를 칠하기 전 지도
    blocks = with_no_go(grid, blocks)
    goal, path = _pick(grid, occ, blocks, pose, blacklist, walls)
    if path:
        return goal, path
    # 진단: frontier를 못 찾은 순간의 지도를 저장 (처음 3번만). controllers/rescue/nav_dump_N.npz
    if _dumps[0] < 3:
        _dumps[0] += 1
        np.savez_compressed(f"nav_dump_{_dumps[0]}.npz", occ=occ, b0=blocks[0], b1=blocks[1], b2=blocks[2],
                            walls=walls, pose=np.array(pose), blacklist=np.array(blacklist).reshape(-1, 2),
                            no_go=np.array([e[:2] for e in NO_GO]).reshape(-1, 2), origin=np.array([grid.ox, grid.oy]))
        print(f"[NAV] 진단 지도 저장: nav_dump_{_dumps[0]}.npz", flush=True)
    log(f"[NAV] 갈 frontier 없음 → 제자리 회전 (blacklist {len(blacklist)}곳, 진입금지 {len(NO_GO)}곳)")
    return None, []


_dumps = [0]
_retried = []                   # blacklist에서 재시도한 frontier (한 번만)
COMMIT_R = 0.8                  # 가던 목표에서 이 거리 안에 frontier가 남아 있으면 계속 그쪽으로
TURN_W = 0.5                    # 새 목표를 고를 때 돌아야 하는 각도 1 rad당 0.5m 먼 것으로 침
ARRIVE_R = 0.6                  # 가던 목표에 이만큼 다가가면 완료 처리 (blacklist)
_cur = [None]                   # 지금 가고 있는 frontier 목표


def _pick(grid, occ, blocks, pose, blacklist, walls):
    # 가던 목표에 도착했는데도 frontier가 남아 있으면 관측할 수 없는 곳(가구 밑 등) → 다시 안 감
    if _cur[0] and math.hypot(pose[0] - _cur[0][0], pose[1] - _cur[0][1]) < ARRIVE_R:
        blacklist.append(_cur[0])
        _cur[0] = None
    # 최소 안전거리 지도 기준: 문틀이 LiDAR 잡음으로 두껍게 찍혀도 방 안 frontier가 후보에서 빠지지 않게
    cells = find_frontiers(occ, blocks[1])
    if not len(cells):
        return None, []
    mask = np.zeros(occ.shape, np.uint8)
    mask[cells[:, 0], cells[:, 1]] = 1
    _, labels = cv2.connectedComponents(mask, connectivity=8)
    lab = labels[cells[:, 0], cells[:, 1]]
    pts = np.array([grid.c2w(r, c) for r, c in cells])
    dist = np.hypot(pts[:, 0] - pose[0], pts[:, 1] - pose[1])
    reach = reachable(grid, blocks[:2], pose, walls)

    reps, retry = [], []                        # (비용, 대표점), blacklist에 있던 것
    for k in np.unique(lab):
        idx = np.where(lab == k)[0]
        if len(idx) < MIN_FRONTIER_CELLS:
            continue
        # 대표점 = 덩어리 중심에 가장 가까운 칸 (로봇에서 가장 가까운 칸으로 하면 다가갈수록
        # 대표점이 옆·뒤로 밀려나 frontier 주위를 빙빙 돎)
        cen = pts[idx].mean(axis=0)
        i = idx[np.argmin(np.hypot(pts[idx, 0] - cen[0], pts[idx, 1] - cen[1]))]
        if dist[i] < ARRIVE_R:
            continue
        p = (float(pts[i][0]), float(pts[i][1]))
        # 지금 갈 수 없는 곳은 이번만 건너뜀 (blacklist에 넣으면 로봇이 벽에 붙은 한순간에
        # 모든 frontier가 영구 제외되어 "더 탐색할 곳 없음"으로 일찍 복귀함)
        if not reach[tuple(cells[i])]:
            continue
        near = lambda pts_: any(math.hypot(p[0] - b[0], p[1] - b[1]) < 0.5 for b in pts_)
        if near(blacklist):
            # blacklist는 후순위: 다른 곳이 없을 때 한 번 더 시도 (보행자에 막혀 등록된 경우가 많음.
            # 영구 제외하면 남은 frontier가 전부 빠져 탐색이 일찍 끝남)
            if not near(_retried):
                retry.append((dist[i], p))
            continue
        # 가던 목표가 아직 남아 있으면 최우선 (매번 가장 가까운 것을 새로 고르면 목표가 1.5초마다
        # 이리저리 바뀌어 방향이 크게 틀어지고 제자리 회전만 반복함). 아니면 거리 + 회전량
        if _cur[0] and math.hypot(p[0] - _cur[0][0], p[1] - _cur[0][1]) < COMMIT_R:
            cost = -1.0
        else:
            turn = abs((math.atan2(p[1] - pose[1], p[0] - pose[0]) - pose[2] + math.pi) % (2 * math.pi) - math.pi)
            cost = dist[i] + TURN_W * turn
        reps.append((cost, p))

    for _, p in sorted(reps):
        path = plan(grid, blocks, pose, p, walls)
        if path:
            _cur[0] = p
            return p, path
        blacklist.append(p)
    for _, p in sorted(retry):
        path = plan(grid, blocks, pose, p, walls)
        if path:
            _retried.append(p)
            _cur[0] = p
            print(f"[NAV] 막혔던 frontier ({p[0]:.2f}, {p[1]:.2f}) 재시도", flush=True)
            return p, path
    _cur[0] = None
    return None, []


# ------------------------- A* -------------------------
MOVES = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
         (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)]

CENTER_M = 0.3                  # 막힌 칸에서 이 거리 안쪽은 가까울수록 이동 비용 증가 [m]
CENTER_W = 4.0                  # 막힌 칸 바로 옆 칸의 비용 = 기본 × (1 + CENTER_W)


def wall_penalty(blocked):
    """칸마다 0~1: 막힌 칸에 가까울수록 1. A*가 문·복도 한가운데로 지나가게 함
    (없으면 경로가 막힌 영역 경계에 딱 붙어 문틀 모서리를 스치고, DWA가 그 궤적을 거부해 못 들어감)"""
    d = cv2.distanceTransform((~blocked).astype(np.uint8), cv2.DIST_L2, 3) * RES
    return np.clip((CENTER_M - d) / CENTER_M, 0.0, 1.0)


def astar(blocked, start, goal, max_iter=150000, pen=None):
    """blocked: bool 2D array. start, goal: (r, c). pen: 막힌 칸 근처 비용 지도. 반환: (r, c) 경로 배열 / 없으면 None"""
    if blocked[goal]:
        return None
    n_r, n_c = blocked.shape
    h = lambda a: math.hypot(a[0] - goal[0], a[1] - goal[1])
    g, parent = {start: 0.0}, {start: None}
    openq, closed, it = [(h(start), start)], set(), 0
    while openq and it < max_iter:
        it += 1
        _, cur = heapq.heappop(openq)
        if cur in closed:
            continue
        if cur == goal:
            path = []
            while cur is not None:
                path.append(cur)
                cur = parent[cur]
            return path[::-1]
        closed.add(cur)
        for dr, dc, cost in MOVES:
            nb = (cur[0] + dr, cur[1] + dc)
            if not (0 <= nb[0] < n_r and 0 <= nb[1] < n_c) or blocked[nb]:
                continue
            ng = g[cur] + (cost if pen is None else cost * (1.0 + CENTER_W * pen[nb]))
            if ng < g.get(nb, float("inf")):
                g[nb], parent[nb] = ng, cur
                heapq.heappush(openq, (ng + h(nb), nb))
    return None


def nearest_free(blocked, cell, max_r=12):
    """blocked[cell]이 True면, 주변에서 가장 가까운 False 칸을 반환. 없으면 None"""
    for rad in range(max_r + 1):
        for dr in range(-rad, rad + 1):
            for dc in range(-rad, rad + 1):
                if max(abs(dr), abs(dc)) != rad:
                    continue
                r, c = cell[0] + dr, cell[1] + dc
                if 0 <= r < blocked.shape[0] and 0 <= c < blocked.shape[1] and not blocked[r, c]:
                    return (r, c)
    return None


def start_cell(grid, blocked, pose, tight, max_r=16):
    """로봇 위치 칸. 벽에 붙어 막힌 칸 안에 있으면, 실제 벽(tight)을 넘지 않고 갈 수 있는
    가장 가까운 빈칸 (막힌 칸에서 A*를 시작하면 못 빠져나옴. 단순히 가장 가까운 빈칸을 쓰면
    얇은 벽 건너편 칸이 뽑혀 벽을 뚫는 경로가 나옴). 없으면 로봇 칸 그대로"""
    s = grid.w2c(pose[0], pose[1])
    if not blocked[s]:
        return s
    seen, q = {s}, deque([s])
    while q:
        cur = q.popleft()
        if not blocked[cur]:
            return cur
        for dr, dc, _ in MOVES:
            nb = (cur[0] + dr, cur[1] + dc)
            far = max(abs(nb[0] - s[0]), abs(nb[1] - s[1]))
            if nb in seen or far > max_r or not grid.inside(*nb):
                continue
            if tight[nb] and far > 2:           # 로봇 바로 옆(벽에 붙은 경우)만 예외
                continue
            seen.add(nb)
            q.append(nb)
    return s


def reachable(grid, blocks, pose, tight):
    """로봇이 지금 갈 수 있는 칸 (bool 지도). blocks 중 하나로라도 이어지면 갈 수 있음"""
    out = np.zeros(blocks[0].shape, bool)
    for blocked in blocks:
        _, lab = cv2.connectedComponents((~blocked).astype(np.uint8), connectivity=8)
        s = start_cell(grid, blocked, pose, tight)
        if not blocked[s]:
            out |= lab == lab[s]
    return out


_last_log = [0.0]


def log(msg):
    """진단 출력. 콘솔이 넘치지 않게 3초에 한 번만"""
    if time.monotonic() - _last_log[0] > 3.0:
        _last_log[0] = time.monotonic()
        print(msg, flush=True)


def plan(grid, blocks, pose, goal, walls=None):
    """world 좌표 경로 [(x, y), ...]. 넉넉한 안전거리부터 시도하고 안 되면 다음 지도로. 없으면 []
    walls: 출발칸 탈출 때 넘으면 안 되는 실제 벽. 진입 금지 구역을 넣으면 안 됨
    (넣으면 막혀서 후진한 로봇이 아직 금지 구역 안이라 영영 못 빠져나옴)"""
    if walls is None:
        walls = blocks[-1]
    blocks = with_no_go(grid, blocks)
    s0, g0 = grid.w2c(pose[0], pose[1]), grid.w2c(*goal)
    why = []
    for blocked in blocks:
        s = start_cell(grid, blocked, pose, walls)
        g = nearest_free(blocked, grid.w2c(*goal))
        if g is None:
            why.append("목표 주변 0.6m가 전부 막힘")
            continue
        cells = astar(blocked, s, g, pen=wall_penalty(blocked))
        if cells is not None:
            path = [grid.c2w(r, c) for r, c in cells[::4]]
            path.append(tuple(goal) if g == grid.w2c(*goal) else grid.c2w(*cells[-1]))
            return path
        why.append("출발칸 막힘" if blocked[s] else "이어진 길 없음")
    log(f"[NAV] 경로 없음 → 제자리 회전. 로봇 ({pose[0]:.2f}, {pose[1]:.2f}) 목표 ({goal[0]:.2f}, {goal[1]:.2f}), "
        f"지도별(넉넉/최소/탈출): {why}, 진입금지 {len(NO_GO)}곳")
    return []


# ------------------------- DWA -------------------------
W_HEAD = 1.0                    # 궤적 끝에서 목표를 바라보는 정도
W_CLEAR = 0.5                   # 장애물까지 여유 (1m에서 상한)
W_VEL = 0.3                     # 빠를수록
W_PROG = 0.0                    # 궤적 끝이 목표에 실제로 가까워진 정도
V_SAMPLES = np.linspace(0.0, V_MAX, 6)
W_SAMPLES = np.linspace(-W_MAX, W_MAX, 13)
SIM_T = np.linspace(0.1, 1.2, 10)


def dwa(goal_local, obs_pts):
    """goal_local: 로봇 기준 목표 (x, y). obs_pts: 로봇 기준 LiDAR 점 (N, 2)."""
    V, W = np.meshgrid(V_SAMPLES, W_SAMPLES)
    V, W = V.ravel(), W.ravel()
    Ws = np.where(np.abs(W) < 1e-3, 1e-3, W)
    T = SIM_T[None, :]
    xs = V[:, None] / Ws[:, None] * np.sin(Ws[:, None] * T)
    ys = V[:, None] / Ws[:, None] * (1 - np.cos(Ws[:, None] * T))
    ths = W[:, None] * T
    if len(obs_pts):
        dx = xs[:, :, None] - obs_pts[None, None, :, 0]
        dy = ys[:, :, None] - obs_pts[None, None, :, 1]
        clear = np.sqrt(dx ** 2 + dy ** 2).min(axis=(1, 2))
        cur_clear = float(np.hypot(obs_pts[:, 0], obs_pts[:, 1]).min())
    else:
        clear = np.full(len(V), 5.0)
        cur_clear = 5.0
    # 이미 SAFE_DIST 안에 들어와 있으면, 지금보다 더 가까워지지 않는 궤적은 허용
    # (안 그러면 좁은 곳에서 모든 궤적이 금지되어 얼어붙음). 몸체 반경은 절대 금지
    thr = max(ROBOT_RADIUS + 0.01, min(SAFE_DIST, cur_clear - 0.01))
    ang = np.arctan2(goal_local[1] - ys[:, -1], goal_local[0] - xs[:, -1]) - ths[:, -1]
    heading = 1 - np.abs((ang + np.pi) % (2 * np.pi) - np.pi) / np.pi
    d0 = math.hypot(goal_local[0], goal_local[1])
    prog = (d0 - np.hypot(goal_local[0] - xs[:, -1], goal_local[1] - ys[:, -1])) / (V_MAX * SIM_T[-1])
    score = (W_HEAD * heading + W_CLEAR * np.minimum(clear, 1.0) + W_VEL * V / V_MAX
             + W_PROG * prog)
    score[clear < thr] = -np.inf                # 충돌 궤적 제거
    best = int(np.argmax(score))
    if not np.isfinite(score[best]):
        return 0.0, 0.0                         # 안전한 궤적 없음 → 정지
    return float(V[best]), float(W[best])


# ------------------------- 경로 추종 -------------------------
BACKUP_V, BACKUP_S = -0.08, 3.0  # 막혔을 때 후진 속도 [m/s], 시간 [s]
SPIN_MOVE = 0.3                  # 이 거리도 못 벗어나면서
SPIN_TURN = 2 * math.pi          # 이만큼 누적 회전하면 "빙빙 돎" → 막힘과 같이 처리


def rear_clear(ranges, angles):
    """로봇 뒤쪽(±30도)에 후진할 공간이 있는지"""
    r = ranges[np.abs(angles) > math.radians(150)]
    r = r[np.isfinite(r)]
    return not len(r) or r.min() > ROBOT_RADIUS + 0.05


class Follower:
    def __init__(self):
        self.progress_pos, self.progress_t = None, 0.0
        self.backup_until = None
        self.spin_ref, self.spin_acc, self.last_yaw = None, 0.0, None

    def _backup(self, pose, t):
        """지금 자리를 진입 금지로 등록하고 후진 시작"""
        NO_GO.append((pose[0], pose[1], time.monotonic()))
        self.backup_until = t + BACKUP_S
        self.spin_ref = None
        return BACKUP_V, 0.0, False

    def step(self, path, pose, ranges, angles, t):
        """path를 앞에서부터 소비하며 (v, w, stuck) 반환. path가 비면 (0, 0, False)
        막히면: 그 자리를 NO_GO에 넣고 BACKUP_S 동안 후진한 뒤 stuck=True (main이 재계획)"""
        if self.backup_until is not None:
            if t < self.backup_until and rear_clear(ranges, angles):
                return BACKUP_V, 0.0, False
            self.backup_until, self.progress_pos = None, None
            return 0.0, 0.0, True
        # 지나온 waypoint 제거. 마지막 점은 도착 판정(GOAL_TOL)의 절반 안까지 와야 제거
        # (여유 없이 지우면 경로는 끝났는데 도착 판정은 못 넘어 제자리 회전만 반복)
        while path and math.hypot(path[0][0] - pose[0], path[0][1] - pose[1]) < (
                0.25 if len(path) > 1 else GOAL_TOL / 2):
            path.pop(0)
        if not path:
            return 0.0, 0.0, False

        # 빙빙 돎 감지: SPIN_MOVE 안에서 누적 회전이 SPIN_TURN을 넘으면 (낮은 장애물에 걸렸거나
        # 목표 방향과 DWA 선택이 번갈아 뒤집히는 경우. 조금씩 움직여서 막힘 감지로는 안 잡힘)
        if self.last_yaw is not None:
            self.spin_acc += abs((pose[2] - self.last_yaw + math.pi) % (2 * math.pi) - math.pi)
        self.last_yaw = pose[2]
        if self.spin_ref is None or math.hypot(pose[0] - self.spin_ref[0], pose[1] - self.spin_ref[1]) > SPIN_MOVE:
            self.spin_ref, self.spin_acc = (pose[0], pose[1]), 0.0
        elif self.spin_acc > SPIN_TURN:
            front = ranges[np.abs(angles) < math.radians(30)]
            front = front[np.isfinite(front)]
            print(f"[SPIN] ({pose[0]:.2f}, {pose[1]:.2f})에서 빙빙 돎 → 진입 금지 후 후진. "
                  f"목표 {tuple(round(v, 2) for v in path[-1])}, 남은 점 {len(path)}, "
                  f"정면 최소거리 {front.min() if len(front) else float('inf'):.2f}m", flush=True)
            return self._backup(pose, t)

        la = next((p for p in path if math.hypot(p[0] - pose[0], p[1] - pose[1]) > LOOKAHEAD), path[-1])
        dx, dy = la[0] - pose[0], la[1] - pose[1]
        c, s = math.cos(-pose[2]), math.sin(-pose[2])
        goal_local = (dx * c - dy * s, dx * s + dy * c)
        valid = np.isfinite(ranges) & (ranges < 2.0)
        obs = np.stack([ranges[valid] * np.cos(angles[valid]),
                        ranges[valid] * np.sin(angles[valid])], axis=1)[::2]
        if abs(math.atan2(goal_local[1], goal_local[0])) > 1.0:
            v, w = 0.0, math.copysign(0.8, goal_local[1])   # 방향이 크게 틀리면 제자리 회전
        else:
            v, w = dwa(goal_local, obs)

        # 막힘 감지: STUCK_S 동안 0.1m도 못 가면 stuck
        if self.progress_pos is None or math.hypot(pose[0] - self.progress_pos[0],
                                                   pose[1] - self.progress_pos[1]) > 0.1:
            self.progress_pos, self.progress_t = (pose[0], pose[1]), t
        elif t - self.progress_t > STUCK_S:
            return self._backup(pose, t)
        return v, w, False
