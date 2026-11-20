# Changelog

## [0.4.1] - 2026-11-13
- AI start check: dropped the rule that refused any start behind the target rack. Only a start already touching a
  rack, pillar or the pallet is refused; from behind the rack the planner decides (it goes if a learned route gets
  round, otherwise '경로 없음').
- AI drive time limit 90 s -> 180 s.
- AI run table: '경로' column became '모델&경로' (AI model name · demo #); `ai_runs.csv` gained a `model` column.
- Tooltips drawn on their own dark background (`set_tooltip_fn`): plain `tooltip=` text sat on Kit's pale yellow
  tooltip window in the panel's light text colour and looked empty. Long tooltips wrap.

## [0.4.0] - 2026-11-12
- Panel: long notes became tooltips (hover); an 'AI 운전 옵션' card with ON / OFF toggles for
  'YOLO로 파렛트 거리 재기' (was the 'YOLO 사용 / 없음' combo) and '자리 고쳐 잡기'; a folder button next to
  'AI 모델' opens the chosen model's folder; with no model the list shows a disabled '학습된 모델 없음'.
- '위치 자유' now applies to recordings too: recordings and AI drives start where the forklift was put by hand,
  and Stop / 'AI 멈춤' bring it back there (turning the option off goes back to the sliders' pose).
- Demonstration table: new 회전 column. AI run table: '눈' column renamed YOLO (ON / OFF); '눈' wording
  replaced with YOLO in the panel and status texts.
- Fixed: the demo summary and the learned-model lines stayed '-' when the panel tab was first drawn late.

## [0.3.0] - 2026-11-12
- 'AI 모델' list above the AI start / stop buttons: pick which learned AI model to drive with. Offered:
  `models/paths.json` ('기본') and every `models/<name>/paths.json`; each pairs with a YOLO model --
  `models/<name>/best.onnx`, else the YOLO run of the same name (`runs/<name>/weights/best.onnx`), else the newest
  run. The newest model is chosen at first; the list follows new / removed models while Isaac Sim runs.
- `tools/learn_paths.py --name <name>` saves to `models/<name>/paths.json` (without it: `models/paths.json`, as before).

## [0.2.0] - 2026-11-12
- Moved to the isaacsim-edu-lv2 repository as Lesson 2 of the new `Isaacsim_Edu_Lv2` launcher (was Lesson 5 of the
  Lv1 launcher). Depends on `Isaacsim_Edu_Lv2_Lesson1` (copy of Lv1's Lesson 3) instead of `Isaacsim_Edu_Lv1_Lesson3`;
  window "AZ Edu Lv2 - Lesson 2", docks into "AZ Edu Lv2", active-lesson setting under `Isaacsim_Edu_Lv2`.
- '녹화 시작' also stays off after an AI run while its paused end scene is kept, until Stop / 'AI 멈춤' resets it
  (it was already off while the AI drove); `_on_start` ignores the request while the AI drives.
- Everything built 2026-11-09..11 and never logged here: path learning + rehearsal (replacing the failed key-level
  behavior cloning), the YOLO distance table, bent routes (warp), stepwise repositioning ('자리 고쳐 잡기'),
  '위치 자유', start rotation, start overlap check, AI run table, panel layout. See docs/03_모방학습_원리.md.

## [0.1.1] - 2026-11-09
- Step 2 material: `tools/make_yolo_dataset.py` (demos -> YOLO train/val split by whole demonstrations,
  data.yaml; standard library only) and the trainee guide `docs/YOLO_실습.md` (Windows: venv, CUDA PyTorch first,
  ultralytics, train, predict, ONNX export).
- Verified on the laptop (RTX 5070 Ti, Windows 11) with 8 scripted verification drives: install 3 min 28 s,
  YOLO11n 30 epochs 3 min 36 s, mAP50 0.954 (0.94 already at epoch 6), ONNX export 9 s.

## [0.1.0] - 2026-11-09
- Step 1, demonstration recording on top of Lesson 3 (Lesson 3 code untouched): keys, forklift state and pallet
  pose at 15 Hz, AI-camera images at 5 Hz with automatic YOLO labels (pallet, rack_level2) from Replicator tight
  2D boxes, success check (pallet resting on the 2nd rack level), optional random start pose in the aisle.
- AI camera between the forks, riding with the carriage (the driver view is blocked by the mast and dashboard).
- Verified headless on Isaac Sim 6.2.0 with scripted key presses driving the real Lesson 3 action graph:
  150 samples / 10 s, 46 images, every image labeled.
