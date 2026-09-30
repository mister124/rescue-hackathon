"""
[B] 탐색 목표 선정 + 경로 계획 + 경로 추종 + 충돌 회피
 - main은 make_blocks() → pick_frontier()/plan() 으로 경로를 받고,
   매 step Follower.step()으로 (v, w)를 받는다.
"""
import math
import heapq
import numpy as np
from config import (RES, INFLATE_M, INFLATE_MIN_M, ROBOT_RADIUS, V_MAX, W_MAX,
                    SAFE_DIST, LOOKAHEAD, STUCK_S, GOAL_TOL)


# ------------------------- Costmap -------------------------
def inflate(obst, radius_cells):
    out = obst.copy()
    R = radius_cells
    for dr in range(-R, R + 1):
        for dc in range(-R, R + 1):
            if dr * dr + dc * dc <= R * R and (dr or dc):
                out |= np.roll(obst, (dr, dc), axis=(0, 1))
    return out


def make_blocks(occ):
    """(넉넉한 안전거리 지도, 최소 안전거리 지도). 경로 계획은 앞에 것부터 시도"""
    obst = occ == 100
    return (inflate(obst, int(math.ceil(INFLATE_M / RES))),
            inflate(obst, int(math.ceil(INFLATE_MIN_M / RES))))


# ------------------------- Frontier -------------------------
def find_frontiers(occ, blocked):
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


def pick_frontier(grid, occ, blocks, pose, blacklist):
    """가까운 frontier부터 경로가 나오는 것을 고름. 반환 (goal, path) / 없으면 (None, [])"""
    cands = find_frontiers(occ, blocks[0])
    if not len(cands):
        return None, []
    pts = np.array([grid.c2w(r, c) for r, c in cands])
    dist = np.hypot(pts[:, 0] - pose[0], pts[:, 1] - pose[1])
    for i in np.argsort(dist)[:30]:
        if dist[i] < 0.6:
            continue
        p = tuple(pts[i])
        if any(math.hypot(p[0] - b[0], p[1] - b[1]) < 0.5 for b in blacklist):
            continue
        path = plan(grid, blocks[:1], pose, p)
        if path:
            return p, path
        blacklist.append(p)
    return None, []


# ------------------------- A* -------------------------
MOVES = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
         (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)]


def astar(blocked, start, goal, max_iter=150000):
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
            ng = g[cur] + cost
            if ng < g.get(nb, float("inf")):
                g[nb], parent[nb] = ng, cur
                heapq.heappush(openq, (ng + h(nb), nb))
    return None


def nearest_free(blocked, cell, max_r=12):
    for rad in range(max_r + 1):
        for dr in range(-rad, rad + 1):
            for dc in range(-rad, rad + 1):
                if max(abs(dr), abs(dc)) != rad:
                    continue
                r, c = cell[0] + dr, cell[1] + dc
                if 0 <= r < blocked.shape[0] and 0 <= c < blocked.shape[1] and not blocked[r, c]:
                    return (r, c)
    return None


def plan(grid, blocks, pose, goal):
    """world 좌표 경로 [(x, y), ...]. 넉넉한 안전거리부터 시도하고 안 되면 다음 지도로. 없으면 []"""
    for blocked in blocks:
        blocked = blocked.copy()
        s = grid.w2c(pose[0], pose[1])
        blocked[max(0, s[0] - 2):s[0] + 3, max(0, s[1] - 2):s[1] + 3] = False
        g = nearest_free(blocked, grid.w2c(*goal))
        if g is None:
            continue
        cells = astar(blocked, s, g)
        if cells is not None:
            return [grid.c2w(r, c) for r, c in cells[::4]] + [grid.c2w(*cells[-1])]
    return []


# ------------------------- DWA -------------------------
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
    score = 1.0 * heading + 0.5 * np.minimum(clear, 1.0) + 0.3 * V / V_MAX
    score[clear < thr] = -np.inf                # 충돌 궤적 제거
    best = int(np.argmax(score))
    if not np.isfinite(score[best]):
        return 0.0, 0.0                         # 안전한 궤적 없음 → 정지
    return float(V[best]), float(W[best])


# ------------------------- 경로 추종 -------------------------
class Follower:
    def __init__(self):
        self.progress_pos, self.progress_t = None, 0.0

    def step(self, path, pose, ranges, angles, t):
        """path를 앞에서부터 소비하며 (v, w, stuck) 반환. path가 비면 (0, 0, False)"""
        # 지나온 waypoint 제거. 마지막 점은 도착 판정(GOAL_TOL) 안에 들어와야 제거
        # (0.25로 지우면 목표 0.2~0.25m 앞에서 경로도 없고 도착도 아닌 상태로 제자리 회전)
        while path and math.hypot(path[0][0] - pose[0], path[0][1] - pose[1]) < (
                0.25 if len(path) > 1 else GOAL_TOL):
            path.pop(0)
        if not path:
            return 0.0, 0.0, False
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
            self.progress_t = t
            return v, w, True
        return v, w, False
