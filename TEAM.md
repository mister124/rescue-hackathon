# 오늘 해커톤 분업 (3시간)

**원칙:** `controllers/rescue/`의 뼈대는 이미 끝까지 연결되어 있다(검출 → 접근 → 복귀). 각자 **자기 파일만** 개발하고, 파일끼리는 아래 함수로만 호출한다.

## 파일과 담당

```
controllers/rescue/
├── rescue.py       # D  장치 초기화, main loop, 상태 머신, 제한 시간, 시각화, 수동 조종(M키 + WASD)
├── config.py       # 공용 설정. [A] [B] [C] [D] 구역으로 나뉨, 자기 구역만 수정
├── slam.py         # A  Odometry, GridMap, compass 보정, (TODO) scan matching
├── nav.py          # B  costmap, frontier 선정, A*, DWA, 경로 추종, 막힘 감지
└── perception.py   # C  색(LAB/HSV) 또는 YOLO 검출, 거리 추정, 대상 목록
```

| 담당 | 파일 | main이 부르는 함수 (바꾸지 말 것) | 개발할 것 |
|---|---|---|---|
| **A. 위치·지도** | `slam.py` | `Slam(x, y, yaw)`, `.update(enc_l, enc_r, gyro_z, compass, ranges, angles, max_range, dt)`, `.pose → (x, y, yaw)`, `.grid` | compass 보정 검증(`USE_COMPASS`), scan matching(`update()` 안의 TODO 자리), 지도 품질 |
| **B. 주행·안전** | `nav.py` | `make_blocks(occ)`, `pick_frontier(grid, occ, blocks, pose, blacklist) → (goal, path)`, `plan(grid, blocks, pose, goal) → path`, `Follower().step(path, pose, ranges, angles, t) → (v, w, stuck)` | 안전거리 유지, 좁은 문 통과, 보행자 회피, 막힘 복구, frontier 선택 기준 |
| **C. 대상 검출** | `perception.py` | `Perception(camera)`, `.grab() → frame`, `.detect(frame) → [{cls, box, bearing, dist}]`, `localize(det, ranges, angles, pose, max_range) → (x, y)`, `TargetBook().add(x, y, cls)`, `.confirmed()` | 당일 대상에 맞춘 검출(`TARGET_COLORS` 또는 YOLO), 오검출 제거, 거리 정확도 |
| **D. 통합·미션** | `rescue.py`, 월드 | 위 함수들을 호출 | 상태 머신, 제한 시간 복귀, 장치 이름과 시작 위치, 전체 실행과 통합, 제출 |

**데이터 규약:** 좌표는 시작점 기준 월드 좌표(m, rad, 반시계 +). `ranges`, `angles`는 `np.ndarray`, angles는 로봇 기준(왼쪽 +). `occ`는 -1 모름 / 0 빈칸 / 100 장애물. `path`는 `[(x, y), ...]`.

## 테스트 방법

- **월드:** `worlds/breakroom_rescue.wbt` (초록 사과가 대상, `supervisor TRUE`)
- **위치 오차 확인:** `config.py`에서 `USE_GT_DEBUG = True`로 두면 5초마다 실제 위치와의 오차를 출력한다. 제출 때는 반드시 `False`
- **수동 조종:** 3D 화면을 클릭한 뒤 `M` → WASD로 조종. 지도와 검출은 계속 갱신되므로 A와 C가 테스트할 때 쓴다
- **카메라 프레임 저장:** OpenCV 창에서 `s` → `controllers/rescue/frame.jpg`

## 협업 방식

- 각자 **자기 파일과 `config.py`의 자기 구역만** 수정한다. 그러면 git merge 충돌이 거의 없다.
- 위 표의 함수 시그니처를 바꿔야 하면 먼저 D에게 알린다.
- D는 성공한 버전마다 git 태그(`good-1`, `good-2` …)를 남긴다. 막히면 바로 되돌린다.

## 타임라인

| 시간 | 전원 / D | A 위치·지도 | B 주행·안전 | C 대상 검출 |
|---|---|---|---|---|
| **0:00–0:15** | 공지 확인(대상, 제한 시간, 월드). 모두 Webots + numpy + opencv 동작 확인. 제공 코드를 담당 구역별로 나누기 | 월드 열고 `list_devices()` 출력 확인 | 맵 구조와 문 폭 확인 | 대상 캡처 저장 |
| **0:15–0:45** | D: 장치 이름·시작 위치 넣고 **0:45까지 첫 전체 실행** | teleop으로 지도 모양 확인. 벽이 두 겹이면 부호나 축 문제 | 안전거리 초기값 결정 | **색 범위 확정**(캡처로 HSV 측정) |
| **0:45–2:15** | D: 실행 → 가장 큰 문제 1개를 담당자에게 배정 → 반영 → 재실행. 시간 제한 복귀 로직 추가 | 위치 튐, 지도 번짐 대응 | 충돌, 막힘, 사람 대응 | 가짜 대상, 중복, 위치 오차 대응 |
| **2:15** | **기능 동결.** 이후로 설정값만 조정 | | | |
| **2:15–2:45** | 연속 2~3회 성공을 확인하고 최종본 고정 | | | |
| **2:45–3:00** | 최종 실행, 제출, 발표용 지도 화면 확보 | | | |

## 막혔을 때 기본값으로 돌아가는 규칙

30분 넘게 안 풀리면 버린다.

| 증상 | 대응 |
|---|---|
| 좁은 문을 못 지나감 | `INFLATE_MIN_M`을 `ROBOT_RADIUS + 0.02`로 낮추기 |
| 가짜 대상이 계속 잡힘 | `TARGET_MIN_AREA`와 `CONFIRM_N` 올리기, S와 V 하한 올리기 |
| 지도가 번짐 | 속도(`V_MAX`, `W_MAX`)를 낮춰 미끄러짐 줄이기 |
| 대상까지 경로가 없다고 포기함 | `APPROACH_DIST`를 늘려 대상에서 더 먼 곳을 목표로 잡기 |
| 시간이 부족함 | `NUM_TARGETS`만큼 찾으면 바로 복귀하므로 탐색 순서는 그대로 두고, 제한 시간 복귀만 확실히 |
