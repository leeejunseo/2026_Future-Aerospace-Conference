# 회전 우주 파편 포획 시뮬레이터 (포획 순간 단계)

회전하는 우주 파편을 포획할 때 대상의 회전 속도·질량에 따라 **로봇팔 / 그물 / 작살** 중
어떤 방식이 유효한지 PyBullet 물리 시뮬레이션으로 정량 비교한다.

## 환경 설정

PyPI 의 `pybullet` 은 최신 파이썬(3.12+)용 윈도우 휠이 없어 conda-forge 를 사용한다.

```bash
conda create -n debris-sim -c conda-forge python=3.12 pybullet
conda activate debris-sim
pip install "numpy<2.5"   # conda-forge numpy 2.5 빌드는 BLAS DLL 문제로 행렬곱에서 크래시 → pip 휠 사용
```

## 실행

이 PC 에는 위 환경이 이미 `debris-sim` 이름으로 만들어져 있다. 기본 `python`(3.13) 에는 PyBullet 이 없으므로
반드시 환경을 활성화하거나 환경의 파이썬을 직접 지정해 실행한다.

```powershell
# 방법 1: 환경 활성화 후 실행
conda activate debris-sim
python capture_sim.py --method net --target large --spin 20 --gui

# 방법 2: 환경의 파이썬을 직접 지정
& "C:\Users\User\anaconda3\envs\debris-sim\python.exe" capture_sim.py --method net --target large --spin 20 --gui
```

```bash
# 단일 시행 (헤드리스)
python capture_sim.py --method arm --target medium --spin 10
# GUI 시각화
python capture_sim.py --method net --target large --spin 20 --gui
# 세차 운동 대상 (장동각 20 deg)
python capture_sim.py --method harpoon --target small --spin 30 --spin-mode precession --nutation 20
# 회전축 변경 (x: 접근축과 일치 → 파지점이 회전축 위에 옴)
python capture_sim.py --method arm --target medium --spin 20 --spin-axis x
# 전체 조합 스윕 → CSV (방식 3 x 대상 3 x 회전속도 N x 시드 M)
python capture_sim.py --sweep --spins 0 5 10 20 30 --seeds 0 1 2 --out results.csv
```

파이썬에서 직접 호출:

```python
from capture_sim import run_capture, CONFIG
CONFIG["harpoon"]["winch_tension"] = 20.0      # 가정값 변경 예
r = run_capture("net", "large", spin_deg_s=15, seed=3)
print(r["success"], r["capture_time"], r["impulse"], r["servicer_attitude_error"])
```

## 웹 3D 리플레이 뷰어 (권장 시각화)

`--viewer out.html` 을 붙이면 헤드리스로 시뮬레이션을 돌린 뒤 **three.js 기반 3D 리플레이가 통째로 들어간 HTML 파일 하나**를
만든다. 브라우저에서 더블클릭으로 열리며, 물리는 PyBullet 이 계산한 궤적을 그대로 재생하므로 시각화가 결과를 바꾸지 않는다.
PyBullet 내장 창보다 렌더링(그림자 없는 PBR 재질, 대기 산란, 실제 각크기의 지구, 5000개 별)과 조작이 훨씬 낫다.

```powershell
conda activate debris-sim
python capture_sim.py --method net --target medium --spin 10 --viewer out.html --open
python capture_sim.py --method arm --target large --spin 20 --viewer arm20.html --trajectory arm20.json
```

`demo/` 폴더에 방식별 샘플이 이미 들어 있다. 브라우저로 바로 열어 보면 된다.

뷰어가 제공하는 것:

- **재생 제어**: 재생/일시정지, 타임라인 스크럽, 0.25~4배속, `,` `.` 한 프레임 이동. 타임라인과 그래프에 접촉·결합·이탈 시점이 표시된다.
- **카메라**: 마우스 드래그 회전, 휠 줌, Shift/오른쪽 드래그 팬, 터치 핀치 줌. 중점·대상·서비서·자유 4가지 추적 모드(키 1~4), `R` 리셋.
- **텔레메트리**: 시각, 상대 각속도, 대상 각속도, 거리, 결합 상태, 방식별 상태(팔 단계 / 그물 접촉 질점 수 / 작살 상태)를 실시간 표시.
- **상대 각속도 그래프**: 성공 임계값 1 deg/s 선, 결합 유지 구간 음영, 이벤트 표시선. 그래프를 클릭하면 그 시각으로 이동한다.
- **표시 토글**: 스프링, 회전축, 파지점, 대상 궤적, 지구.
- **스크린샷**과 전체화면. `?` 또는 `H` 로 조작법.

뷰어 HTML 은 자기 완결형(지구 텍스처는 data URI 로 내장)이지만 **three.js 는 CDN 에서 불러오므로 인터넷 연결이 필요하다.**
UI 틀은 [viewer_template.html](viewer_template.html) 에 있고, 궤적은 `--trajectory out.json` 으로 따로 저장할 수도 있다.

## 궤도 환경과 PyBullet 내장 GUI

**궤도 상대 역학 (기본 켜짐, `--no-orbit` 으로 끔)**: 시뮬레이션 좌표계를 기준 원궤도(고도 500 km 가정)의
LVLH 프레임(x: 진행 방향, z: 천정, y: 궤도면 수직)으로 해석하고, 각 강체에 Hill(Clohessy–Wiltshire) 방정식의
겉보기 가속도와 중력구배 토크 3n²(r̂×Ir̂)를 매 스텝 외력으로 적용한다. 60 s 포획 구간에서 효과는 mm~cm,
mN·m 수준으로 작지만 실제 궤도 환경을 재현한다. 회전좌표계라 관성 각속도와 상대 각속도가 궤도 각속도(0.06 deg/s)만큼
다르며, 판정 임계값 1 deg/s 대비 무시 가능하다.

**PyBullet 내장 GUI (`--gui`)**: 시뮬레이션을 실시간으로 보며 조작하고 싶을 때 쓴다. 완성도 높은 화면이 필요하면 위의 웹 뷰어를 쓰는 편이 낫다.
검은 우주 배경, 실제 고도에서 보이는 각크기에 맞춘 지구(NASA Blue Marble 텍스처, `assets/earth.jpg`),
배경 별, 태양광 방향, 대상 회전축(빨간 선)과 파지·조준점(노란 선) 표시, 시각·상대 각속도·단계 HUD, 대상–서비서 중점을
따라가는 카메라(마우스 회전·줌은 유지). 장면 연출은 물리에 영향을 주지 않는다.

**카메라 조작 (GUI)**

| 키 | 동작 |
|---|---|
| ← → ↑ ↓ | 카메라 회전 (요 / 피치) |
| Z / X | 줌 인 / 아웃 |
| W A S D, Q E | 앞뒤·좌우 이동, 위·아래 이동 |
| F | 추적 모드 순환: mid(대상–서비서 중점) → target → servicer → free |
| 1 / 2 / 3 | 프리셋: 전체 보기 / 대상 근접 / 서비서 시점 |
| R | 카메라 리셋 |
| Space | 일시정지 / 재개 (정지 중에도 카메라 조작 가능) |
| [ / ] | 슬로모션(0.25배) / 정상 속도 |
| H | HUD 켜기 / 끄기 |
| 마우스 | Ctrl + 왼쪽 드래그 회전, 휠 줌 (PyBullet 기본). 물체 드래그(피킹)는 물리 교란 방지를 위해 꺼 둠 |

**세부 외형**: GUI/녹화 시 서비서(MLI 금박 본체, 태양전지 날개, 고이득 안테나, 추력기, 센서, 라디에이터), 소형 파편(큐브샛: 레일,
전지판 면, 안테나), 중형 파편(본체, 프레임 있는 전지판, 요크, 안테나, 노즐, 그래플 픽스처), 대형 파편(로켓 상단: 도장 텍스처, 인터스테이지 링,
상단 돔, 노즐 벨, 레이스웨이), 3지 그리퍼를 절차적 텍스처와 기본 도형 조합으로 그린다. **충돌 형상·질량·관성은 단순 형상 그대로**라
물리 결과는 헤드리스 실행과 동일하다(안테나·노즐 등 시각 부품은 접촉에 관여하지 않음).

**녹화**: `--record-frames DIR` 로 PNG 프레임(기본 1280×720, 8스텝마다)을 저장한다. GUI 모드에서는 지구·조명이 포함되고,
헤드리스에서는 PyBullet 내장 소프트웨어 렌더러의 한계로 지구 배경 없이 포획 장면만 검은 배경에 기록된다.
`--record-video FILE.mp4` 는 GUI 모드에서 ffmpeg 이 PATH 에 있을 때 동작한다.

```powershell
python capture_sim.py --method net --target large --spin 20 --gui --record-frames frames_net
```

**기존 우주 환경 시뮬레이터에 대해**: Basilisk(Vizard), Orekit, GMAT 같은 우주 환경 도구는 궤도·자세 역학과 시각화에
특화되어 있고, 로봇팔·그물·작살의 접촉·다물체 역학은 지원하지 않는다. 반대로 PyBullet, MuJoCo, Gazebo 같은 물리 엔진은
접촉은 잘 다루지만 궤도 환경이 없다. 포획 순간 연구에는 접촉 물리가 핵심이라 이 코드는 물리 엔진(PyBullet) 위에
궤도 환경의 유효 효과(무중력, 토크-프리 회전, CW 상대 역학, 중력구배)를 직접 얹는 구성을 택했다.

## 격자 몬테카를로 실험 (`run_grid_experiment.py`)

회전 속도 0~30 deg/s(2 간격) × 질량 3구간 × 방식 3종 × 반복 30회(시드 변경) = 4320 시행을 병렬로 실행한다.
매 반복마다 실제 임무의 불확실성(회전축·회전속도 추정 오차, 상대 위치·잔류 속도 오차, 서비서 지향 오차,
초기 자세)을 시드로 섭동하며, 같은 반복 번호는 모든 조합에서 같은 시드를 써 방식 간 비교가 짝지어진다.

```powershell
conda activate debris-sim
python run_grid_experiment.py --workers 10           # 전체 격자. 12스레드 노트북 기준 약 5~7시간
python run_grid_experiment.py --resume --workers 10  # 중단 후 이어서
python run_grid_experiment.py --plots-only           # CSV 로부터 요약·그래프만 재생성
python run_grid_experiment.py --spins 0 10 20 30 --repeats 3 --out results_quick   # 빠른 확인용
```

출력(`results/`): `results_raw.csv`(시행별), `results_summary.csv`(조합별 성공률 + Wilson 95 % CI + 보조 지표),
`heatmap_success.png`(방식별 회전속도×질량 히트맵), `curves_success.png`(질량별 성공률 곡선, CI 밴드),
`experiment_config.txt`(사용된 모든 설정값 기록).

## 출력 (시행당 dict)

| 키 | 의미 |
|---|---|
| `success` | 상대 각속도 ≤ 1 deg/s 가 `hold_time` 동안 유지되고 결합이 유지되면 True |
| `capture_time` | 첫 접촉부터 상대 회전 감쇄(임계값 진입)까지 시간 [s] |
| `impulse` | 대상에 전달된 스텝당 최대 선형 충격량 [N·s] (운동량 변화로 측정, 2차 파편 위험 지표) |
| `servicer_attitude_error` | 초기 자세 대비 서비서 자세 변화 최대값 [deg] |
| 보조 | `angular_impulse`, `peak_contact_force`, `max_interaction_force`, `final_rel_omega_deg_s`, `first_contact_time`, `bond_lost_time`, `notes` 등 |

## 모델링 요약

- 중력 0, 서비서(1000 kg 가정)는 자유 부유. 포획 반동이 그대로 자세 변화로 나타난다.
- 대상: 소형 정육면체(10 kg) / 중형 본체+태양전지판(500 kg) / 대형 원통(2000 kg). 관성텐서는 해석식으로 계산해 명시 설정.
- 회전: 주축 회전(`principal`) 또는 각속도 벡터를 장동각만큼 기울인 토크-프리 세차(`precession`).
  PyBullet 적분 결과를 오일러 방정식 RK4 해와 비교해 일치함을 확인했다 (각감쇠 0 필수).
- 로봇팔: 6관절 + 그리퍼, IK 추종. 예비 접근점 → 대기점 → 파지점의 3단계 접근(데카르트 직선 보간, 접근 원뿔 60°).
  파지는 고정 제약으로 모델링하되 그리퍼 닫힘 시간 동안 힘 한계를 0→유지력으로 올리는 순응 파지.
  유지력 초과 시 미끄러짐 → 파지점 이탈 거리로 놓침 판정. 파지 후 관절은 토크 제한 브레이크.
- 그물: 9×9 질점을 인장 전용 Kelvin-Voigt 스프링(구조+전단)으로 연결. 접힌 상태로 발사되어 모서리 추의 측방 속도로 전개.
  첫 접촉 후 둘레 스프링 자유길이를 축소하는 조임 기구. 모서리 4점은 서비서와 테더로 연결.
- 작살: 캡슐 발사체. 충돌 전 법선 속도 ≥ 임계값이고 입사각 ≤ 허용각이면 관통(고정 제약), 아니면 반발.
  테더는 대상 둘레에 감길 수 있도록 질점 체인으로 모델링. 고정부 힘이 한계를 넘으면 이탈.

## 주의

- 모든 가정값은 `capture_sim.py` 상단 `CONFIG` 에 있으며, 문헌 근거가 없는 값은 `# TODO: 출처 확인 필요` 로 표시했다.
  논문에 쓰기 전에 반드시 출처를 확인하거나 민감도 분석을 수행할 것.
- 기본 회전축은 z(접근축 x 에 수직). 중형 대상은 태양전지판이 접근 평면을 쓸고 지나가므로
  팔 방식이 불리하다. 회전축을 x 로 두면(`--spin-axis x`) 파지점이 회전축 위에 온다. 두 시나리오 모두 연구 조건으로 명시할 것.
- 작살의 상대 회전 감쇄는 테더 장력 제어(`winch_tension`, 기본 0)에 크게 좌우된다.
- 그물 시행은 질점 수가 많아 1회 약 20~40 초(헤드리스) 소요된다.
