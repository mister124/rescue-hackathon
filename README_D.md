# D 통합·미션 완성본

첨부 TEAM.md에 따라 `controllers/rescue/rescue.py`와 `config.py`의 [D] 구역만 수정했다.
`slam.py`, `nav.py`, `perception.py`, config의 [A]/[B]/[C] 구역은 원본과 동일하다.

## 1. 적용

이 ZIP은 Webots 프로젝트 루트 기준 구조다. `controllers/rescue/` 안의 Python 파일이 컨트롤러이며, 원래 ZIP처럼 Python 파일만 최상위에 놓는 구조와 다르다.

1. Webots 시뮬레이션을 멈추고 기존 프로젝트를 먼저 Git 커밋하거나 별도 백업한다.
2. 기존 프로젝트에서 `controllers/rescue/rescue.py`를 이 버전으로 교체한다.
3. `config.py`는 [D] 구역만 병합한다. 팀원이 [A]/[B]/[C] 값을 바꿨다면 파일 전체를 덮어쓰지 않는다.
4. tests/, README_D.md, GIT_GUIDE.md, requirements.txt, .gitignore를 프로젝트 루트에 추가한다. 기존 .gitignore가 있으면 규칙만 추가한다.
5. 팀원의 A/B/C 개발분은 기존 것을 유지한다. ZIP에 포함한 A/B/C는 사용자 첨부 원본이며 최신 팀원 작업을 대체하는 용도가 아니다.
6. 기존 `worlds/breakroom_rescue.wbt`를 Webots에서 열고 로봇의 `controller`를 `rescue`로 설정한 뒤 월드를 저장한다.
7. 시뮬레이션을 Reset/Revert하여 시작 위치부터 다시 실행한다. 컨트롤러만 재시작하면 지도/타이머가 현재 위치에서 새로 시작하므로 공식 실행 전에는 월드도 초기화한다.

월드 파일, PROTO, 모델/텍스처는 첨부에 없어 이 ZIP에 포함하지 않았다. 기존 프로젝트의 해당 폴더를 그대로 사용한다. 장치 목록과 월드 실제 배치는 사용자 PC에서 확인해야 한다.

## 2. Python 환경

Webots가 쓰는 Python에 설치:

```powershell
python -m pip install -r requirements.txt
```

`python`이 Webots 콘솔에 표시되는 실행 파일과 같은지 확인한다. 다르다면 표시된 python.exe의 경로로 실행한다. PowerShell에서 경로에 공백이 있으면 다음 형식 사용:

```powershell
& "C:\실제경로\python.exe" -m pip install -r requirements.txt
```

기본 색 검출은 numpy와 opencv-python 사용. YOLO를 선택하면 C 담당이 ultralytics와 모델 파일도 준비해야 한다. `controller` 모듈은 Webots가 제공하므로 별도 pip 패키지를 설치하지 않는다. `python rescue.py`를 일반 터미널에서 직접 실행하는 대신 Webots에서 실행한다.

## 3. 상태 머신과 성공 기준

| 상태 | 행동 | 다음 상태 |
|---|---|---|
| WARMUP | 센서와 지도 초기화, 자동 주행 정지 | 설정 시간 뒤 EXPLORE |
| EXPLORE | frontier 탐색, 주행 중 기존 목표 유지 | 미방문 대상 발견 시 GOTO; 경로 반복 실패 시 RETURN |
| GOTO | 가까운 대상부터 접근, 대상 앞 APPROACH_DIST 지점 계획 | 도착 시 방문 기록; 실패 시 보류 후 EXPLORE |
| RETURN | 시작 좌표까지 재계획/주행; 경로 없으면 정지하며 재시도 | 시작점 도착 시 DONE |
| DONE | 계속 정지 | 재실행 전까지 유지 |
| TIMEOUT | 제한 시간 도달 즉시 정지 | 재실행 전까지 유지 |
| ERROR | 예외 기록, 모터 정지 후 오류 출력 | 원인 수정 후 재실행 |

- 성공은 **필요 수만큼 실제 접근 완료 + 제한 시간 내 시작점 복귀**다.
- 확정 검출 수와 방문 수를 구분한다. 단순 검출 또는 경로 실패는 방문 성공이 아니다.
- 대상 거리 <= APPROACH_DIST + GOAL_TOL이면 방문 처리한다. 기본값 0.5 + 0.2 = 0.7m. 별도 집기 동작은 포함하지 않는다.
- 대상 접근이 10초간 진전되지 않거나 한 시도가 45초를 넘으면 중단한다. 20초 후 다시 후보가 되며, 최대 2회 시도한다. 탐색이 끝나거나 시간 복귀 조건이 먼저 충족되면 재시도보다 복귀가 우선이다.
- 현재 C의 TargetBook.confirmed()가 기존 target dict 객체를 반환한다는 규약을 사용한다. C가 복사본을 반환하도록 바꾸려면 D와 방문 정보 공유 방식을 먼저 맞춰야 한다.
- 종료 `DONE`만으로 미션 성공은 아니다. 대상 방문 부족으로 일찍 돌아오면 `[RESULT] INCOMPLETE`다.
- `mission_result.json`은 종료 상태/성공 여부/방문 수/대상 좌표/경과 시간을 저장한다. 다음 실행이 덮어쓰므로 기록이 필요하면 별도 파일명으로 복사한다.

## 4. 시간과 키보드

기본값은 시뮬레이션 경과 600초 제한, 480초에 강제 복귀다. 첫 시작의 `robot.getTime()`을 빼므로 컨트롤러를 늦게 시작해도 경과 시간을 계산한다. 컴퓨터 실제 시계나 대회 운영진의 외부 타이머와는 다르다. 대회가 월드 전체 경과 시간을 기준으로 한다면 그 규칙에 맞게 타이머 기준을 조정해야 한다.

| 입력 | 동작 |
|---|---|
| 3D 화면 클릭 후 M | 자동/수동 전환. 자동 재개 시 오래된 경로 폐기 |
| W / S | 수동 전진 / 후진 |
| A / D | 수동 좌회전 / 우회전. W+A 같은 조합 가능 |
| 키 없음 | 수동 모드에서 정지 |
| Space | 정지 잠금. 키를 놓아도 유지 |
| P | 정지 잠금 해제 |
| R | 즉시 자동 복귀 요청 |
| OpenCV 창에서 s | 컨트롤러 폴더에 frame.jpg 저장 |

- RETURN 진입 시 수동 모드 해제. M으로 복귀를 취소할 수 없다.
- Space 정지 잠금은 자동 복귀보다 우선하며, P를 누를 때까지 정지한다.
- 수동/정지 중에도 시뮬레이션 제한 시간은 계속 흐른다.
- 수동 주행은 nav의 장애물 회피를 거치지 않는다. 모터 최대 속도 제한만 적용하므로 관찰하면서 조종한다.
- DONE/TIMEOUT 이후에는 M/P로 다시 출발하지 않는다. 재실행으로 미션을 새로 시작한다.
- 지도/검출은 수동/정지 중에도 갱신하지만 미션 종료 후에는 정지 상태를 유지한다.

## 5. 장치와 설정 확인

시작 시 장치 이름 전체를 출력한다. config.py [D]의 이름을 콘솔 목록과 맞춘다.

| 설정 | 기본값/의미 |
|---|---|
| LEFT_MOTOR / RIGHT_MOTOR | left wheel motor / right wheel motor |
| LEFT_ENC / RIGHT_ENC | left wheel sensor / right wheel sensor; 없으면 해당 모터 연결 센서 조회 |
| LIDAR_NAME / CAMERA_NAME | LDS-01 / camera |
| REQUIRE_CAMERA | True; 지도/수동 단독 테스트만 False |
| START_X / START_Y / START_YAW | 보통 0 / 0 / 0, 시작점 기준 좌표 |
| NUM_TARGETS | 기본 1, 실제 대회 조건 확인 |
| TIME_LIMIT_S / RETURN_MARGIN_S | 기본 600 / 120초 |
| USE_GT_DEBUG | 제출 시 False; True면 월드에 supervisor TRUE 필요 |
| SHOW_WINDOWS | 기본 True; 환경변수 RESCUE_SHOW=0 또는 설정 False로 표시 끄기 |

현재 A의 자이로/컴패스와 D의 GT 진단은 XY 평면, z-up 기준이다. 월드가 다르면 A와 좌표축을 맞춰야 한다. 월드 translation을 START_X/Y에 그대로 복사할 필요는 없다. 라이다 왼쪽/오른쪽이 반대로 찍히는 문제는 A의 LIDAR_ANGLE_SIGN 확인 대상이다.

LiDAR의 +inf는 측정 범위 안에 반사점이 없다는 값으로 유지한다. 0/음수는 무효 빔으로 제외하며 모든 빔이 무효면 오류 정지한다. 일부 NaN/무효 영역의 충돌 회피 정책은 B가 검토해야 한다. 이 검사는 센서 전체의 정상 여부를 보증하지 않는다.

## 6. 검증 결과와 실제 주행 확인

실행한 명령:

```powershell
python -m unittest discover -s tests -v
python -m compileall -q controllers tests
```

21개 테스트 통과. 상태 전환, 방문/실패 구분, 재시도 제한, 복귀 경로 없음, 수동 모드의 시간 제한, 정지 잠금/재개, 센서/장치 오류, 모터 속도 제한을 검증했다. 실제 A/B 함수 연결 테스트도 포함한다. main 테스트는 Webots 장치와 검출을 대체한 테스트 더블 사용.

**Webots·OpenCV 영상 처리·월드 물리 시뮬레이션은 이 작업 환경에서 실행하지 못했다. 실제 주행 성공 태그 good-1은 아직 부여하지 않았다.**

사용자 PC에서 확인:

1. 콘솔 `[START]`와 장치 이름 확인, 오류 메시지 유무 확인.
2. M → W/A로 수동 주행, 지도와 카메라 갱신 확인. M으로 자동 복귀 시 현재 위치에서 재계획하는지 확인.
3. Space를 눌러 정지, 키를 놓아도 정지 유지, P로 재개 확인.
4. 기본 대상에 접근해 `[VISIT]`, `[STATE] RETURN`, `[RESULT] SUCCESS` 확인.
5. 시간 조건만 빠르게 시험하려면 D 설정을 임시로 TIME_LIMIT_S=30, RETURN_MARGIN_S=10으로 변경. 수동 모드로 시작점에서 이동한 뒤 20초에 자동 복귀 전환 확인. 30초 전에 복귀하면 DONE 유지, 복귀하지 못했다면 TIMEOUT 정지 확인. 시험 뒤 600/120으로 복원.
6. 좁은 문/동적 장애물/도달 불가능 대상에서 관찰. 막힘은 B, 위치 드리프트는 A, 색 오검출은 C와 해결.
7. 같은 월드를 시작 상태로 초기화하여 2~3회 성공한 뒤 good-1 태그 지정.

고정 RETURN_MARGIN_S는 복귀 시간을 예약하는 기준이며 실제 귀환을 보장하지 않는다. 복귀 경로가 계속 막히면 정지 재계획을 반복하다 TIMEOUT으로 끝난다. 지도의 미지 영역 처리, A* 모서리 통과, 로컬 회피 성능은 원본 B 코드 동작을 따른다.
