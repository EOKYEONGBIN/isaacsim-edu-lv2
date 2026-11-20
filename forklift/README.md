# isaacsim-edu-lv2 — AZWELL Isaac Sim 교육 Lv2 (지게차 AI)

Isaac Sim 6.2.0 익스텐션 묶음. 교육생이 지게차를 직접 운전해 시연을 녹화하고, 그 시연으로 AI를 학습시켜
AI가 파렛트를 2층 선반에 올리게 만드는 실습이다.

```
Lesson 1  지게차 시뮬레이션   키보드로 지게차 운전 익히기
Lesson 2  지게차 AI           ① 시연 녹화 → ② 경로 학습 + YOLO 학습 → ③ AI 운전
```

## 폴더

이 `forklift` 폴더가 곧 Isaac Sim의 **익스텐션 검색 경로**다. 저장소는 교육 VM의 `E:\omniverse\content\isaacsim-edu-lv2`에 두므로,
검색 경로는 `E:\omniverse\content\isaacsim-edu-lv2\forklift`이다. 저장소 맨 위의 `ExtensionRoot.config`에 적힌 폴더들을
`AZ Edu Lv2 익스텐션.exe`가 검색 경로로 등록한다.

익스텐션은 경로를 고정해 두지 않고 **저장소 폴더(`isaacsim-edu-lv2`)의 바로 위 폴더**(교육 VM: `E:\omniverse\content`)를 기준으로 장면 · 데이터를 찾는다.
그래서 아래 세 폴더가 같은 폴더 안에 나란히 있으면 어느 드라이브 · 폴더에 두어도 된다 (개발 PC는 `C:\isaacsimProjects`).
이 문서의 경로는 교육 VM 기준이다.

| 폴더 | 내용 |
|---|---|
| `Isaacsim_Edu_Lv2/` | 런처. Azwell 메뉴의 "AZ Edu Lv2" 창, 레슨 버튼 |
| `Isaacsim_Edu_Lv2_Lesson1/` | Lesson 1 지게차 시뮬레이션 |
| `Isaacsim_Edu_ForkliftAI/` | Lesson 2 지게차 AI. `tools/`에 학습 스크립트, `docs/YOLO_실습.md`에 교육생 YOLO 교재 |
| `docs/` | 강사용 문서 |

익스텐션 밖에 필요한 것 (저장소에 없음):

| 경로 | 내용 | 받는 곳 |
|---|---|---|
| `E:\omniverse\content\iw_hub_warehouse_navigation\` | 창고 장면 (`iw_hub_warehouse_navigation.usd`, 지게차 AI용 `iw_hub_warehouse_forklift_ai.usd`) | 강사가 복사 (Lv1과 같은 장면 폴더) |
| `E:\omniverse\content\forklift_ai\demos\` | 기본 시연 15개 | 이 저장소 GitHub Release의 zip |

학습된 모델(`forklift_ai\models\paths.json`, `forklift_ai\runs\...\best.onnx`)은 배포하지 않는다. 교육생이 직접 만든다.

## 문서

| 문서 | 읽는 사람 |
|---|---|
| [docs/01_VM_사전세팅.md](docs/01_VM_사전세팅.md) | 강사 · VM 담당자: VM 요구 사항, 설치, YOLO 라이선스 방침 |
| [docs/02_교육_워크플로우.md](docs/02_교육_워크플로우.md) | 강사: 교육 당일 진행 순서와 시간 |
| [docs/03_모방학습_원리.md](docs/03_모방학습_원리.md) | 강사 · 발표 자료: AI가 운전을 배우는 원리, 어떤 스크립트를 어떻게 바꿨는지 |
| [Isaacsim_Edu_ForkliftAI/docs/YOLO_실습.md](Isaacsim_Edu_ForkliftAI/docs/YOLO_실습.md) | 교육생: YOLO 직접 설치 · 학습 · ONNX 내보내기 |
| [docs/WEB_CLAUDE_발표자료_브리프.md](docs/WEB_CLAUDE_발표자료_브리프.md) | 발표 자료(Figma) 제작용 브리프 |
