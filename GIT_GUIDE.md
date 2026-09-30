# D 담당용 Git 버전 관리

명령은 VS Code 터미널(PowerShell)에서 **controllers와 worlds 폴더가 들어 있는 Webots 프로젝트 루트**에서 실행한다. ZIP에는 .git 이력이나 GitHub 연결을 넣지 않았다. 원격 저장소 생성/푸시는 직접 아래 절차로 진행한다.

## 1. 세 가지 개념

- commit: 현재 코드와 월드 파일의 저장 지점. 수정할 때마다 의미 있는 단위로 기록.
- branch: main과 분리한 작업 흐름. A/B/C/D가 각자 수정할 때 사용.
- tag: 특정 커밋에 붙이는 고정 이름. 실제 실행에 성공한 버전을 good-1, good-2로 표시.

Git은 커밋한 파일 상태를 저장한다. Webots의 아직 저장하지 않은 월드 수정, 실행 중 지도/로봇 위치, 열린 에디터의 미저장 내용은 복원하지 않는다. 월드는 저장한 .wbt 파일로 함께 관리한다.

## 2. 기존 코드부터 보관

기존 팀 저장소를 clone한 프로젝트라면 git init을 하지 말고 해당 저장소에서 작업한다. 다음으로 현재 위치가 저장소에 포함되는지 확인:

```powershell
git status
git rev-parse --show-toplevel
```

저장소가 아니라고 나오면 프로젝트 루트에서 처음 한 번만:

```powershell
git init -b main
git config user.name "박진성"
git config user.email "본인의 Git 커밋용 이메일"
```

이메일 문구는 실제 값으로 교체. 이 설정은 현재 저장소에만 적용된다. 이미 설정되어 있으면 반복 불필요.

제공 .gitignore의 규칙을 프로젝트 루트에 적용한다. __pycache__, pyc, 가상환경, 자동 생성 frame.jpg/mission_result.json 등을 제외한다. 월드, PROTO, 필요한 모델/텍스처는 프로젝트에 맞게 추적한다. 이미 추적한 파일은 .gitignore 추가만으로 추적이 해제되지는 않는다.

**D 완성본을 덮어쓰기 전**, 현재 파일을 먼저 보관:

```powershell
git status --short
git add .
git diff --cached --stat
git diff --cached
git commit -m "chore: save baseline before D integration"
git tag -a baseline-before-d -m "Original project before D integration"
```

`git add .`는 프로젝트 전체를 선택하므로 staged 목록을 반드시 확인한다. 기존 팀 저장소에 다른 사람의 미완성 작업이 섞여 있으면 변경 전 담당자와 상태를 확인하고 필요한 경로만 add한다. 이미 완성본을 덮어쓴 뒤라면 이 커밋은 원본 백업이 될 수 없다. 원본 ZIP/기존 팀 브랜치로 원본을 확보한 다음 진행한다.

## 3. D 작업과 커밋

```powershell
git switch -c feature/d-mission
```

README_D.md 절차로 rescue.py와 config.py의 [D] 구역을 적용하고, 안내/테스트 파일을 루트에 추가한다. A/B/C 파일은 팀원 최신본 유지.

```powershell
python -m unittest discover -s tests -v
python -m compileall -q controllers tests
git diff -- controllers/rescue/rescue.py controllers/rescue/config.py
git add controllers/rescue/rescue.py
git add -p controllers/rescue/config.py
git add tests README_D.md GIT_GUIDE.md requirements.txt .gitignore
git diff --cached --stat
git commit -m "feat: complete D mission and timed return"
```

`git add -p`에서는 D 구역만 y로 선택, 타 담당 변경은 n. 한 덩어리에 함께 나오면 s로 분할하거나 VS Code Source Control에서 줄/범위를 선택해 Stage Selected Ranges 사용.

이후 수정할 때 반복:

```powershell
git status
git diff
git add controllers/rescue/rescue.py
git add -p controllers/rescue/config.py
git commit -m "fix: describe the specific change"
```

월드 설정까지 바꿨다면 저장 후 다음 경로도 함께 add:

```powershell
git add worlds/breakroom_rescue.wbt
```

실제 파일명이 다르면 해당 경로 사용. 컨트롤러가 예전 파일로 돌아가는 문제를 추적할 때 .wbt의 controller 값도 함께 비교한다.

## 4. GitHub 연결

이미 origin이 있으면 그대로 사용하고 중복 추가하지 않는다:

```powershell
git remote -v
```

새 프로젝트라면 GitHub에서 **빈 저장소**를 만든다. 기존 로컬 코드와 합칠 것이므로 원격에 README/.gitignore/license를 자동 생성하지 않는 편이 단순하다. 아래 URL은 본인/팀 저장소 주소로 교체:

```powershell
git remote add origin https://github.com/OWNER/REPO.git
git push -u origin main
git push -u origin feature/d-mission
git push origin baseline-before-d
```

여기까지는 코드 공유다. 아직 물리 주행 성공을 의미하지 않는다. 기존 origin 주소를 고쳐야 할 때만 `git remote set-url origin 실제주소` 사용. 원격에 이미 팀 코드가 있으면 별도 init 후 강제 push하지 말고 팀 저장소를 clone해서 D 변경을 적용한다.

## 5. A/B/C와 협업

| 사람 | 브랜치 예 | 주 작업 |
|---|---|---|
| A | feature/a-slam | slam.py, config [A] |
| B | feature/b-nav | nav.py, config [B] |
| C | feature/c-perception | perception.py, config [C] |
| D | feature/d-mission | rescue.py, config [D], 월드 통합 |

팀원은 처음 clone한 후 자기 브랜치 생성:

```powershell
git clone https://github.com/OWNER/REPO.git
cd REPO
git switch -c feature/a-slam
```

브랜치 이름은 본인 담당에 맞게 바꾼다. 수정 후 자기 파일과 config의 자기 구역만 커밋하고 push. GitHub에서 main 대상 PR을 만든 뒤 D가 통합한다.

D는 충돌 해결/테스트가 끝난 PR을 main에 병합하고 로컬 갱신:

```powershell
git switch main
git pull --ff-only origin main
python -m unittest discover -s tests -v
```

기존 D 브랜치에서 최신 main을 반영하려면 먼저 현재 수정을 커밋한 다음:

```powershell
git switch feature/d-mission
git fetch origin
git merge origin/main
```

config.py 충돌은 [A]/[B]/[C]/[D] 구역별 최종 값을 모두 보존해 해결한다. 파일 전체에 무조건 Accept Current/Incoming을 누르면 다른 담당 설정을 잃을 수 있다. 수정 후 `git add controllers/rescue/config.py`, `git commit`으로 마무리. 병합 전체를 취소하려면 진행 중인 merge에서 `git merge --abort`.

## 6. 실제 성공 버전 태그

main에 통합된 코드/월드가 깨끗한 상태인지 확인하고 Webots에서 2~3회 반복 성공한 뒤:

```powershell
git switch main
git status --short
git tag -a good-1 -m "Webots: target visited and returned in 3 runs"
git push origin main
git push origin good-1
```

`git status --short` 출력이 있다면 먼저 검토/커밋해야 한다. 태그는 마지막 커밋을 가리키므로 미커밋 수정은 태그에 포함되지 않는다. 다음 성공 버전은 good-2 등 새로운 이름 사용. 실행하지 않은 버전에 성공 태그를 붙이지 않는다.

기록과 비교:

```powershell
git log --oneline --graph --decorate --all
git tag --list
git diff good-1 HEAD -- controllers/rescue/rescue.py controllers/rescue/config.py
git show good-1:worlds/breakroom_rescue.wbt
```

## 7. 문제가 생겼을 때

### 성공 버전으로 별도 복구 브랜치 생성

먼저 현재 변경을 커밋하거나 임시 보관한다. 변경이 있을 때만 stash:

```powershell
git status
git stash push -u -m "before recovery"
git switch -c recovery/good-1 good-1
```

good-1의 파일 상태로 새 브랜치가 열리고 최신 main은 그대로 남는다. Webots에서 해당 프로젝트의 월드를 다시 열거나 Revert하여 저장된 파일/시작 배치를 불러오고 실행한다. 자동 저장으로 열린 예전 에디터 내용이 복구 파일을 덮지 않도록 시뮬레이터를 먼저 멈춘다.

복구 버전에서 새 수정을 계속하려면 그대로 commit. 원래 D 브랜치로 돌아가려면:

```powershell
git switch feature/d-mission
git stash list
```

앞에서 실제로 만든 stash가 있다면 목록에서 해당 항목을 확인한 뒤 `git stash pop`으로 복원한다. stash를 만들지 않았다면 기존의 다른 stash를 pop하지 않는다.

### 공유한 일반 커밋 하나를 취소

```powershell
git log --oneline -8
git revert 취소할커밋해시
git push
```

`취소할커밋해시`를 실제 해시로 교체. revert는 취소용 새 커밋을 만든다. merge 커밋 취소에는 부모 선택이 필요하므로 위 명령을 그대로 적용하지 않는다. 여러 팀원이 공유하는 main에는 무작정 reset --hard 또는 force push를 사용하지 않는다.

## 공식 참고

- Git 브랜치 전환/생성: https://git-scm.com/docs/git-switch
- Git 태그: https://git-scm.com/book/en/v2/Git-Basics-Tagging
- Git 되돌리기 커밋: https://git-scm.com/docs/git-revert
- Webots 모터/연결 센서 API: https://github.com/cyberbotics/webots/blob/master/docs/reference/motor.md
