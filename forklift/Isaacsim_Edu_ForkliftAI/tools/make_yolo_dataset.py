"""녹화한 지게차 시연으로 YOLO 검출 데이터셋을 만듭니다.

    python make_yolo_dataset.py                      (기본값 사용)
    python make_yolo_dataset.py --demos D:\\demos --out D:\\yolo_dataset --val 0.2

Forklift AI 녹화기가 저장한 demo_*/images/*.png + demo_*/labels/*.txt를 읽어 다음을 만듭니다.

    <out>/images/train, images/val, labels/train, labels/val, data.yaml

검증 이미지는 임의 프레임이 아니라 시연 단위로 떼어 냅니다. 한 주행의 인접 프레임은 거의 같아서,
본 적 없는 주행으로 검증해야 하기 때문입니다. 시연이 하나뿐이면 5프레임마다 하나를 검증용으로 씁니다.
표준 라이브러리만 사용합니다.
"""

import argparse
import json
import random
import shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]  # isaacsim-edu-lv2를 담고 있는 폴더 (교육용 VM: E:\omniverse\content)
DEFAULT_DEMOS = ROOT / "forklift_ai" / "demos"
DEFAULT_OUT = ROOT / "forklift_ai" / "yolo_dataset"
CLASSES = ["pallet", "rack_level2"]


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--demos", type=Path, default=DEFAULT_DEMOS, help="folder with demo_* recordings")
    ap.add_argument("--out", type=Path, default=DEFAULT_OUT, help="dataset folder to create (replaced)")
    ap.add_argument("--val", type=float, default=0.2, help="share of demonstrations used for validation")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    demos = sorted(d for d in args.demos.glob("demo_*") if (d / "images").is_dir())
    if not demos:
        raise SystemExit(f"녹화 폴더(demo_*)가 없습니다: {args.demos}")

    if args.out.exists():
        shutil.rmtree(args.out)
    for split in ("train", "val"):
        (args.out / "images" / split).mkdir(parents=True)
        (args.out / "labels" / split).mkdir(parents=True)

    rng = random.Random(args.seed)
    order = demos[:]
    rng.shuffle(order)
    n_val = max(1, round(len(order) * args.val)) if len(order) > 1 else 0
    val_demos = set(order[:n_val])

    counts = {"train": 0, "val": 0}
    boxes = {c: 0 for c in CLASSES}
    for demo in demos:
        for i, img in enumerate(sorted((demo / "images").glob("*.png"))):
            label = demo / "labels" / (img.stem + ".txt")
            if not label.exists():
                continue
            split = "val" if (demo in val_demos or (n_val == 0 and i % 5 == 0)) else "train"
            name = f"{demo.name}_{img.stem}"
            shutil.copyfile(img, args.out / "images" / split / f"{name}.png")
            shutil.copyfile(label, args.out / "labels" / split / f"{name}.txt")
            counts[split] += 1
            for line in label.read_text(encoding="utf-8").splitlines():
                if line.strip():
                    boxes[CLASSES[int(line.split()[0])]] += 1

    (args.out / "data.yaml").write_text(
        f"path: {args.out.as_posix()}\n"
        "train: images/train\n"
        "val: images/val\n"
        "names:\n" + "".join(f"  {i}: {c}\n" for i, c in enumerate(CLASSES)),
        encoding="utf-8",
    )
    (args.out / "summary.json").write_text(json.dumps({
        "demos": [d.name for d in demos], "val_demos": sorted(d.name for d in val_demos),
        "images": counts, "boxes": boxes}, ensure_ascii=False, indent=1), encoding="utf-8")
    print(f"시연 {len(demos)}개 (검증용 {len(val_demos)}개) -> 학습 이미지 {counts['train']}장, 검증 이미지 {counts['val']}장")
    print(f"박스: pallet {boxes['pallet']}개, rack_level2 {boxes['rack_level2']}개")
    print(f"data.yaml: {args.out / 'data.yaml'}")


if __name__ == "__main__":
    main()
