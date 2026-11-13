"""Learn driving routes from the recorded demonstrations.

    python learn_paths.py                  (defaults below; numpy only, e.g. inside yolo-venv)
    python learn_paths.py --name my_forklift   (-> models\\my_forklift\\paths.json: an 'AI 모델' of its own)
    python learn_paths.py --demos D:\\demos

From every successful demonstration it keeps the route (forward / reverse segments, tiny
back-and-forth corrections removed) and the fork height along it, then checks each route by
driving it in the head from its own start. Isaac Sim reads the result (models\\paths.json).
"""

import argparse
import csv
import importlib.util
import json
import time
from pathlib import Path

HERE = Path(__file__).resolve()
ROOT = HERE.parents[3]  # the folder that holds isaacsim-edu-lv2 (training VM: E:\omniverse\content)
spec = importlib.util.spec_from_file_location("paths", HERE.parents[1] / "Isaacsim_Edu_ForkliftAI" / "paths.py")
P = importlib.util.module_from_spec(spec)
spec.loader.exec_module(P)
spec = importlib.util.spec_from_file_location("vision", HERE.parents[1] / "Isaacsim_Edu_ForkliftAI" / "vision.py")
V = importlib.util.module_from_spec(spec)
spec.loader.exec_module(V)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--demos", type=Path, default=ROOT / "forklift_ai" / "demos")
    ap.add_argument("--name", help="AI model name: saved as models\\<name>\\paths.json and offered in the panel's "
                                   "'AI 모델' list (a YOLO run of the same name is paired with it)")
    ap.add_argument("--out", type=Path, help="where to save (default models\\paths.json, or models\\<name>\\paths.json)")
    args = ap.parse_args()
    if args.out is None:
        models = ROOT / "forklift_ai" / "models"
        args.out = models / args.name / "paths.json" if args.name else models / "paths.json"

    t0 = time.time()
    demos = []
    for d in sorted(args.demos.glob("demo_*")):
        try:
            meta = json.loads((d / "meta.json").read_text(encoding="utf-8"))
            rows = list(csv.DictReader(open(d / "steps.csv", encoding="utf-8")))
        except (OSError, ValueError):
            continue
        if meta.get("success"):
            demos.append((d.name, rows, meta))
    if not demos:
        raise SystemExit(f"성공한 시연이 없습니다: {args.demos}")

    paths = P.learn(demos)
    print(f"성공한 시연 {len(demos)}개에서 경로를 배웠습니다.")
    print(" 시연                  구간(전진F/후진B, 길이)                       자기 출발점에서 연습 → 포크 끝 오차")
    for d in paths["demos"]:
        segs = " ".join(f"{'F' if s['dir'] > 0 else 'B'}{(len(s['x']) - 1) * P.RESAMPLE_M:.1f}" for s in d["segments"])
        miss = min((m for m in (P.rehearse(d, *d["start"], la) for la in P.LOOKAHEADS_M) if m is not None), default=None)
        result = f"{miss * 100:.0f} cm" if miss is not None else "끝까지 못 감"
        if d["insert_segment"] is None:
            result = "파렛트에 넣는 구간을 못 찾음 → AI가 쓰지 않음"
        elif not d["usable"]:
            result += f" · 파렛트에 {d['approach_deg']:.0f}° 비스듬히 들어감 → AI가 쓰지 않음"
        else:
            result += f" · 들어간 각도 {d['approach_deg']:.0f}°"
        print(f" {d['name']}  {segs:45s} {result}")
    paths["demos"] = [d for d in paths["demos"] if d["insert_segment"] is not None]
    n_ok = sum(d["usable"] for d in paths["demos"])
    print(f"AI가 쓰는 경로: {n_ok}개 (파렛트 구멍 방향에서 {P.MAX_APPROACH_DEG:.0f}° 이내로 들어간 시연)")
    s = P.summary(paths)
    print(f"\n들어 올린 높이 평균 {s['carry_lift_m'][0]} m · 내려놓은 뒤 포크 높이 평균 {s['final_lift_m'][0]} m · "
          f"경로 길이 평균 {s['path_length_m'][0]} m")
    if paths["seat_ahead_m"] is not None:
        print(f"포크가 끝까지 들어갔을 때 파렛트 중심까지 거리: {paths['seat_ahead_m']} m (이 거리가 되면 멈추고 들어 올립니다)")
    # what the YOLO box says about the distance: from the boxes the recorder saved with each image
    paths["vision"], n = V.learn_table([args.demos / name for name, _, _ in demos], P.pallet_ahead)
    if paths["vision"]:
        far = paths["vision"]["far"]
        print(f"AI의 눈 거리표: 이미지 {n}장에서 학습 · 파렛트 박스 아래 끝이 화면 {far[0][0]:.2f}→{far[0][-1]:.2f} 위치면 "
              f"{far[1][0]:.1f}→{far[1][-1]:.1f} m (그보다 가까우면 지게차가 움직인 거리로 계산)")
    else:
        print(f"AI의 눈 거리표를 만들 이미지가 부족합니다 ({n}장). AI 운전은 시뮬레이터 정답 거리로만 됩니다.")
    args.out.parent.mkdir(parents=True, exist_ok=True)
    P.save(paths, args.out)
    print(f"저장: {args.out}  ({time.time() - t0:.1f}초)")


if __name__ == "__main__":
    main()
