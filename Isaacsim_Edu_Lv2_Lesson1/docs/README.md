# Isaac Sim Edu Lv2 - Lesson 1 (지게차 시뮬레이션)

AZWELL Isaac Sim 교육 Lv2의 Lesson 1. Lv1 `Isaacsim_Edu_Lv1_Lesson3`(지게차 운전)을 복사한 것이다.

- 켜면 `E:\omniverse\content\iw_hub_warehouse_navigation\iw_hub_warehouse_navigation.usd` 장면과
  "지게차 시뮬레이션" 창을 연다.
- Play 후 방향키로 운전(위/아래 = 전진/후진, 좌/우 = 회전), Left Shift = 포크 올림, Left Ctrl = 포크 내림.
- 파렛트는 포크와의 PhysX 접촉·마찰만으로 들어 올린다 (스크립트로 붙이지 않음).
- Lesson 2(지게차 AI, `Isaacsim_Edu_ForkliftAI`)가 이 익스텐션의 조작 노드와 운전 상태를 그대로 쓴다.
  지게차 AI가 켜져 있으면 이 익스텐션은 장면·창을 열지 않고 조작만 제공한다.

`Isaacsim_Edu_Lv2` 런처의 버튼으로 켜는 것을 전제로 한다.

## Lv1 Lesson 3과의 관계

- 익스텐션 이름, 창 제목(`AZ Edu Lv2 - Lesson 1`), 설정 키(`/persistent/exts/Isaacsim_Edu_Lv2/...`)만 바꿨다.
- Action Graph 노드 타입 이름(`Isaacsim_Edu_Lv1_Lesson3.Forklift*`)은 **그대로** 두었다. 장면 파일이 이 이름으로
  노드를 참조하기 때문이다. 그래서 Lv1 Lesson 3과 동시에 켜지면 안 되며, 켜질 때 Lv1 레슨을 모두 끈다.
