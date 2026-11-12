# Isaacsim_Edu_ForkliftAI — 지게차 AI (Lv2 Lesson 2)

교육생이 Lesson 1 지게차를 직접 운전해 파렛트를 2층 선반(윗면 1.325 m)에 올리는 과정을 녹화하고,
그 시연으로 AI를 학습시켜 AI가 같은 일을 하게 만든다.

```
① 시연 녹화 (이 익스텐션)  →  ② 학습: 경로 학습(tools/learn_paths.py) + YOLO 학습(교육생이 직접)  →  ③ AI 운전 (이 익스텐션)
```

원리는 저장소의 [docs/03_모방학습_원리.md](../../docs/03_모방학습_원리.md)에 있다.

## 실행

`Isaacsim_Edu_Lv2` 런처의 **Lesson 2 · 지게차 AI** 버튼으로 켠다. 의존하는 `Isaacsim_Edu_Lv2_Lesson1`(지게차 조작)이 함께 켜지고,
이 익스텐션은 지게차 AI 전용 장면 `C:\isaacsimProjects\iw_hub_warehouse_navigation\iw_hub_warehouse_forklift_ai.usd`를 연다.

## 창 "AZ Edu Lv2 - Lesson 2"

| 영역 | 기능 |
|---|---|
| 지게차 출발 위치 | 세로 · 가로 · 회전 슬라이더, 파렛트 뒤로. **랜덤**(시작할 때마다 무작위, 다른 물체와 겹치지 않게), **확장**(가로 -5~5 m), **위치 자유**(시험 기능: AI가 지금 서 있는 곳에서 출발) |
| 시연 녹화 | ● 녹화 시작 / ■ 녹화 끝내기, 모인 시연 표, 마지막 녹화 지우기, 데이터 폴더 열기, AI 카메라 화면 보기 |
| AI 운전 | ▶ AI 운전 시작(운전 중에는 일시 정지 버튼) / ■ AI 멈춤, 파렛트 거리 판단(YOLO 사용 / YOLO 없음), 자리 고쳐 잡기, 누르는 키 표시, AI 운전 기록 표 |

- 지게차가 선반 · 기둥 · 파렛트와 겹친 채로는 녹화도 AI 운전도 시작하지 않는다 (상태 글로 알림).
- AI가 운전하는 동안, 그리고 운전이 끝나 멈춘 장면이 남아 있는 동안에는 **녹화 시작이 꺼진다.** Stop이나 'AI 멈춤'으로 장면을 초기화하면 다시 켜진다.

## 데이터 — `C:\isaacsimProjects\forklift_ai\`

| 경로 | 내용 |
|---|---|
| `demos\demo_<시각>\steps.csv` | 15 Hz. 시각, 누른 키 6개(전진 · 후진 · 좌 · 우 · Lift Up · Lift Down), 지게차 x · y · 방향 · 속도 · 조향각 · 포크 높이, 파렛트 x · y · z · 방향, 그 순간 이미지 파일 |
| `demos\...\images\*.png` | 5 Hz, 640×480, **AI 카메라**(포크 사이, 포크와 함께 오르내림) |
| `demos\...\labels\*.txt` | YOLO 형식 정답 박스(0 = pallet, 1 = rack_level2). Replicator가 자동 생성 |
| `demos\...\meta.json` | 출발 상태, 성공 여부(파렛트가 2층 선반 위), 박스 수 |
| `models\paths.json` | 경로 학습 결과 (`tools/learn_paths.py`가 만듦) |
| `runs\<이름>\weights\best.onnx` | YOLO 학습 결과. 가장 최근 것을 자동으로 씀 |
| `ai_runs.csv` | AI 운전 기록 표 |

## 도구 (`tools/`)

| 스크립트 | 하는 일 | 실행 |
|---|---|---|
| `learn_paths.py` | 성공한 시연에서 경로를 배워 `models\paths.json` 저장. numpy만 사용 | `yolo-venv\Scripts\python.exe ...\tools\learn_paths.py` |
| `make_yolo_dataset.py` | 시연 이미지 · 라벨을 YOLO 학습 폴더(`yolo_dataset`)로 정리. 표준 Python만 사용 | [YOLO_실습.md](YOLO_실습.md) 4절 |

## 라이선스 메모

YOLO(Ultralytics, AGPL-3.0)는 교육생이 직접 설치해 공식 `yolo` 명령으로 학습한다.
이 익스텐션과 도구는 `ultralytics`를 import하지 않고, ONNX로 내보낸 모델만 Isaac Sim 내장 onnxruntime으로 읽는다.
