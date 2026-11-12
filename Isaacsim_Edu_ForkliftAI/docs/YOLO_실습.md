# 실습: YOLO 직접 설치하고 "AI의 눈" 학습시키기 (Windows)

Lesson 2 지게차 AI의 한 부분이다. 녹화한 시연 이미지로 YOLO를 학습시켜, 포크 카메라 영상에서
**파렛트(pallet)** 와 **2층 선반(rack_level2)** 을 찾는 모델을 만든다.

- YOLO는 **물체를 찾는 일만** 한다. 운전하는 법은 `learn_paths.py`(경로 학습)가 따로 배운다.
- 모든 명령은 **명령 프롬프트(cmd)** 에서 실행한다. PowerShell은 가상환경 활성화가 막힐 수 있다.
- 인터넷 연결이 필요하다 (패키지 약 3 GB, 기본 모델 `yolo11n.pt` 자동 다운로드).

> **라이선스:** YOLO(Ultralytics)는 AGPL-3.0 오픈소스다. 그래서 교육 자료와 VM에는 들어 있지 않고,
> **각자 직접 설치해서** 공식 명령으로 학습한다. Isaac Sim 익스텐션은 YOLO 코드를 쓰지 않고,
> 여러분이 마지막에 내보낸 **ONNX 파일만** 읽는다.

---

## 0. 준비 확인 (1분)

```
python --version
nvidia-smi
```

- `Python 3.10` ~ `3.12` 이면 된다.
- `nvidia-smi` 표에 GPU(`NVIDIA A10`)가 보여야 한다.

## 1. 가상환경 만들기 (1분)

YOLO 전용 Python 공간을 따로 만든다. Isaac Sim의 Python과 섞이지 않는다.

```
cd C:\isaacsimProjects\forklift_ai
python -m venv yolo-venv
yolo-venv\Scripts\activate.bat
```

프롬프트 앞에 `(yolo-venv)` 가 붙으면 성공.

## 2. PyTorch(GPU 버전) 먼저 설치 (약 3분)

> ⚠ **순서가 중요하다.** Windows에서 `pip install ultralytics` 만 하면 **GPU가 아닌 CPU 버전 PyTorch**가 깔려
> 학습이 수십 배 느려진다. GPU 버전을 먼저 설치한다.

```
python -m pip install --upgrade pip
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128
python -c "import torch; print(torch.cuda.is_available(), torch.cuda.get_device_name(0))"
```

`True NVIDIA A10` 이 나와야 다음으로 간다. `False` 면 강사를 부른다.

## 3. YOLO 설치 (약 1분)

```
pip install ultralytics onnx onnxslim onnxruntime
yolo checks
```

onnx 관련 패키지는 7절 내보내기에 쓴다. 미리 깔아 두면 내보낼 때 자동 설치 · 재시작 안내가 뜨지 않는다.

## 4. 학습 데이터 만들기 (수 초)

녹화 폴더(`forklift_ai\demos\demo_*`: 기본 시연 15개 + 내가 녹화한 시연)를 YOLO가 읽는 구조로 정리한다.

```
python C:\isaacsimProjects\isaacsim-edu-lv2\Isaacsim_Edu_ForkliftAI\tools\make_yolo_dataset.py
```

출력 예 (시연 20개일 때):
```
시연 20개 (검증용 4개) -> 학습 이미지 2713장, 검증 이미지 685장
박스: pallet 14919개, rack_level2 7445개
```

- 정답 박스(라벨)는 녹화할 때 시뮬레이터가 **자동으로** 붙여 두었다. 직접 라벨링할 필요가 없다.
- **검증용**은 학습에 쓰지 않고 남겨 둔 시연이다. 모델이 "처음 보는 운전 장면"에서도 잘 찾는지 채점하는 데 쓴다.

## 5. 학습 (이 PC 기준 50 epoch 약 15분)

```
yolo detect train data=C:\isaacsimProjects\forklift_ai\yolo_dataset\data.yaml model=yolo11n.pt epochs=50 imgsz=640 batch=16 project=C:\isaacsimProjects\forklift_ai\runs name=my_forklift
```

| 옵션 | 뜻 |
|---|---|
| `model=yolo11n.pt` | 일반 사진으로 미리 학습된 가장 작은 YOLO에서 시작 (전이 학습) |
| `epochs=50` | 학습 데이터를 50번 반복. 시간이 모자라면 `epochs=30` |
| `imgsz=640` | 이미지 크기 |
| `batch=16` | 한 번에 보는 이미지 수 |

- 화면의 `mAP50` 이 학습이 진행될수록 올라가는지 본다 (1.0에 가까울수록 정확).
- 학습하는 동안에는 **Isaac Sim을 꺼 두면** 2~3배 빨라진다 (같은 GPU를 쓴다).

## 6. 결과 보기

`C:\isaacsimProjects\forklift_ai\runs\my_forklift\` 폴더:

| 파일 | 볼 것 |
|---|---|
| `results.png` | 학습 곡선. loss는 내려가고 mAP50은 올라가야 한다 |
| `val_batch0_labels.jpg` / `val_batch0_pred.jpg` | 정답 박스 vs 내 모델이 찾은 박스 |
| `confusion_matrix.png` | 파렛트 · 선반을 서로 헷갈렸는지 |
| `weights\best.pt` | 가장 점수가 좋았던 내 모델 |

새 이미지로 확인:
```
yolo detect predict model=C:\isaacsimProjects\forklift_ai\runs\my_forklift\weights\best.pt source=C:\isaacsimProjects\forklift_ai\yolo_dataset\images\val save=True
```

## 7. Isaac Sim용으로 내보내기 (약 10초)

```
yolo export model=C:\isaacsimProjects\forklift_ai\runs\my_forklift\weights\best.pt format=onnx
```

`best.onnx` 가 생긴다. 지게차 AI 창은 `runs` 폴더에서 **가장 최근의 `best.onnx`를 자동으로** 고른다.
다음 "AI 운전 시작"부터 쓰이므로 Isaac Sim을 다시 켤 필요는 없다. 창의 **파렛트 거리 판단**을 **YOLO 사용**으로 두고 운전시킨다.

---

## 강사용: 측정값 (개발 PC, 노트북 RTX 5070 Ti Laptop, Windows 11)

| 단계 | 결과 |
|---|---|
| 2~3. 설치 (2026-11-09) | PyTorch 2.11.0+cu128, ultralytics 8.4.208, 합계 3분 28초 |
| 4. 데이터 (2026-11-11, 시연 20개) | 학습 2,713장 / 검증 685장 |
| 5. 학습 (2026-11-11, 시연 20개) | epoch당 약 16초 (Isaac Sim을 같이 켜 두었을 때 28~50초). 81 epoch에서 자동 종료, 27분 |
| 정확도 mAP50 | 15 epoch 0.938 → 30 epoch 0.959 → 50 epoch 0.962 → 최고 0.976 (71 epoch) |
| 7. 내보내기 | 3~10초, `best.onnx` 10.2 MB |

- **50 epoch는 약 13~15분**, 30 epoch는 약 8~9분이다 (Isaac Sim을 끈 상태 기준).
- 30 epoch와 50 epoch의 mAP50 차이는 0.003이라, 시간이 빠듯하면 30 epoch로 줄여도 된다.
- **A10 VM에서는 아직 재 보지 않았다.** 교육 전에 VM에서 한 번 재 볼 것.
- 기본 시연 15개 + 교육생 5개 = 20개 기준이다.
