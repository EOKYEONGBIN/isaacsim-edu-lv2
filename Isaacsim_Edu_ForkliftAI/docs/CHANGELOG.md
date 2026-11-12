# Changelog

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
