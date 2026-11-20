"""Turn recorded forklift demonstrations into a YOLO detection dataset.

    python make_yolo_dataset.py                      (defaults below)
    python make_yolo_dataset.py --demos D:\\demos --out D:\\yolo_dataset --val 0.2

Reads demo_*/images/*.png + demo_*/labels/*.txt written by the Forklift AI recorder and creates

    <out>/images/train, images/val, labels/train, labels/val, data.yaml

Validation images come from whole demonstrations (not random frames), so the model is checked on
drives it never saw -- neighbouring frames of one drive look almost the same. With a single
demonstration, every 5th frame is used instead. Standard library only.
"""

import argparse
import json
import random
import shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]  # the folder that holds isaacsim-edu-lv2 (training VM: E:\omniverse\content)
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
