# 통로 사진 자동 생성 (Replicator) + 정답 라벨 기록 + (선택) Cosmos 판단 정확도 표
#
#   import data_gen, asyncio
#   # 1) 사진만 만들기: 무작위 장애물 배치로 N장, 정답 라벨과 함께 저장
#   out = await data_gen.run(n=24, seed=0)
#   # 2) 만든 사진을 Cosmos 에 보내 정확도 표까지 (지금 aisle_judge 의 프롬프트 칸 글 + 장애물별 판단으로)
#   out = await data_gen.run(n=24, seed=0, judge=True)
#   # 3) 이미 만든 사진 폴더를 다시 판단만 하기 (사진은 그대로, 프롬프트·장애물별 판단만 바꿔 비교)
#   out = await data_gen.judge_folder(r"datasets/aisle_random_...")
#
# 만드는 것 (datasets/aisle_random_<시각>/)
#   images/img_0001.png ...          통로 입구에서 본 앞 카메라 사진 (Cosmos 가 보는 것과 같은 카메라·해상도)
#   images_labeled/img_0001.png ...  같은 사진에 물체마다 박스와 이름을 그려 넣은 것 (눈으로 확인용)
#   labels.csv                       박스 좌표 중심의 한 파일 (아래 열). 물체 하나 = 한 줄. 물체가 없는 사진(비움)은 label 이 빈 한 줄
#       image, aisle_no, state, truth_passable, label, x_min, y_min, x_max, y_max   (좌표는 사진 픽셀)
#   results_custom.csv               (judge=True 일 때) 사진마다 Cosmos 의 판단·이유·응답 시간
#   summary.md                       (judge=True 일 때) 상태별 정확도 표 + 시험에 쓴 프롬프트와 장애물별 판단
#
# 박스는 Replicator 의 instance_id_segmentation 어노테이터(물체 번호 이미지)에서 계산한다:
# 우리가 놓은 물체(쓰러진 박스·젖은 바닥 표지판·웅덩이)의 번호가 칠해진 픽셀의 최소~최대 좌표가 그 물체의 박스다.
#
# 무작위로 바꾸는 것 (같은 seed 면 같은 사진이 나온다 → 프롬프트 비교가 공정하다)
#   - 통로(1~4)와 장애물 상태(비움 / 박스로 막힘 / 액체 흘림), 박스·웅덩이 위치(seed 로 달라짐)
#   - 카메라 자리: 로봇을 통로 입구에서 좌우 ±0.35 m, 앞뒤 ±0.4 m, 방향 ±6° 흔든다
#   - 조명 세기: 장면의 모든 조명을 같은 배율(0.6~1.4)로 흔든다 (끝나면 원래 값으로 복구)
#
# 사진 촬영은 capture.py (Replicator 의 렌더 프로덕트 + RGB 어노테이터) 를 그대로 쓴다.
# 민감한 정보(서버 주소·계정 등)는 어떤 파일에도 기록하지 않는다.

import asyncio
import builtins
import csv
import os
import random
import time

import omni.kit.app
import omni.timeline
import omni.usd

import amr_scene
import aisle_judge
import capture
import incidents

ROOT = os.path.dirname(os.path.abspath(__file__))
STATES = incidents.STATES                      # 장애물 표(incidents.py)에서 가져온다
DX, DY, DYAW = 0.35, 0.4, 6.0
LIGHT_RANGE = (0.6, 1.4)


def status():
    """진행 상황 (다른 곳에서 읽어 갈 수 있다): builtins._datagen"""
    return getattr(builtins, "_datagen", {})


def _set_status(**kw):
    s = status()
    s.update(kw)
    builtins._datagen = s


async def _frames(n):
    for _ in range(n):
        await omni.kit.app.get_app().next_update_async()


async def _ensure_stopped():
    tl = omni.timeline.get_timeline_interface()
    if not tl.is_stopped():
        tl.stop()
        await _frames(5)


def _lights():
    """장면의 조명들과 원래 세기를 모은다 (조명 세기 흔들기용)."""
    from pxr import UsdLux
    stage = omni.usd.get_context().get_stage()
    out = []
    for p in stage.Traverse():
        try:
            if p.HasAPI(UsdLux.LightAPI) or p.IsA(UsdLux.BoundableLightBase) or p.IsA(UsdLux.NonboundableLightBase):
                a = UsdLux.LightAPI(p).GetIntensityAttr()
                v = a.Get()
                if v is not None:
                    out.append((a, float(v)))
        except Exception:
            pass
    return out


def _scale_lights(lights, k):
    for a, v in lights:
        a.Set(v * k)


async def _refresh():
    """장면을 바꾼 뒤 잠깐 Play 했다 멈춘다. 멈춘 상태의 Replicator 사진은 바뀐 장면을 반영하지 않고 이전 화면을 그대로 돌려줄 때가
    있었다 (확인: 로봇 자리를 시작점 → 통로 입구로 옮겨도 사진이 완전히 같았다). 한 번 Play 했다 멈추면 최신 화면이 나온다."""
    tl = omni.timeline.get_timeline_interface()
    tl.play()
    await _frames(8)
    await _ensure_stopped()


async def _grab_front():
    """앞 카메라 사진 한 장 (표시물 숨김은 호출하는 쪽에서 한 번에 한다). 값이 비어 나오면 Play 했다 멈춘 뒤 다시 찍는다."""
    rgb = await capture._grab(amr_scene.FRONT_CAM, capture.RESOLUTION, 6)
    if rgb is None:
        await _refresh()
        rgb = await capture._grab(amr_scene.FRONT_CAM, capture.RESOLUTION, 6)
    return rgb


# 박스는 Replicator 의 instance_id_segmentation 어노테이터로 얻는다: 사진과 같은 크기의 '물체 번호 이미지'(픽셀마다 그 자리의
# 물체 번호)와 '번호 → 물체 경로' 표를 준다. 우리가 놓은 물체(/World/Incidents/...)의 번호가 칠해진 픽셀의 최소~최대 좌표가 박스다.
# (의미 이름표 + bounding_box_2d_tight 도 시도했지만, 창고 에셋이 이미 쓰는 이름표 수백 개만 돌려주고 우리가 중간에 붙인 이름표는
#  인식하지 못했다. 이 방식은 이름표가 필요 없고, 다른 물체에 가려진 부분은 자동으로 빠진다.)
LABEL_KEYS = ["image", "aisle_no", "state", "truth_passable", "label", "x_min", "y_min", "x_max", "y_max"]
COSMOS_KEYS = ["image", "aisle_no", "state", "truth_passable", "decision", "action", "seen", "obstacle", "reason", "confidence", "sec", "error"]
BOX_COLORS = dict(incidents.box_colors(), person=(255, 80, 200))      # 장애물 색은 incidents.py 표에서, 작업자(person)만 여기서


def _class_of(name):
    """/World/Incidents/<통로>/<이름>/... 의 <이름> 으로 박스 이름을 정한다. 우리 물체가 아니면 None."""
    return incidents.class_of(name)


def _object_of(path):
    """물체 경로 → (물체 키, 박스 이름). 우리가 놓은 장애물(/World/Incidents/...)과 걸어 다니는 작업자(/World/Workers/W<n>)만 해당,
    창고 선반·바닥 등은 None. 작업자의 충돌체(W<n>_col)는 눈에 보이지 않는 물체라 뺀다."""
    import dynamic_events
    inc = incidents.ROOT_PATH + "/"
    wk = dynamic_events.WORKER_ROOT + "/"
    if path.startswith(inc):
        parts = path[len(inc):].split("/")                   # [통로, 이름, ...]
        if len(parts) >= 2 and _class_of(parts[1]):
            return f"{parts[0]}/{parts[1]}", _class_of(parts[1])
    elif path.startswith(wk):
        parts = path[len(wk):].split("/")                    # [W<n>, ...]
        if parts and parts[0].startswith("W") and not parts[0].endswith("_col"):
            return f"worker/{parts[0]}", "person"
    return None


def _seg_annotator(v):
    """앞 카메라 렌더 프로덕트에 '물체 번호 이미지' 어노테이터를 붙인다 (한 번만, 끝나면 _detach_seg)."""
    import omni.replicator.core as rep
    s = getattr(builtins, "_datagen_seg", None)
    if s and s["rp"] is v["rp"]:
        return s["ann"]
    ann = rep.AnnotatorRegistry.get_annotator("instance_id_segmentation", init_params={"colorize": False})
    ann.attach([v["rp"]])
    builtins._datagen_seg = {"rp": v["rp"], "ann": ann}
    return ann


def _detach_seg():
    s = getattr(builtins, "_datagen_seg", None)
    if s:
        try:
            s["ann"].detach()
        except Exception:
            pass
    builtins._datagen_seg = None


def _read_boxes(ann, w, h):
    """방금 찍은 사진의 박스 목록 [(label, x_min, y_min, x_max, y_max), ...]. 물체 하나가 메시 여러 개여도 하나로 합친다."""
    import numpy as np
    d = ann.get_data()
    if not d or "data" not in d:
        return []
    seg = np.asarray(d["data"])
    if seg.ndim == 3:
        seg = seg[..., 0]
    by_obj = {}                                              # 물체 키 → (박스 이름, 번호들)
    for sid, path in (d.get("info") or {}).get("idToLabels", {}).items():
        obj = _object_of(str(path))
        if obj:                                              # 창고 선반·바닥 등은 뺀다
            by_obj.setdefault(obj[0], (obj[1], []))[1].append(int(sid))
    out = []
    for key, (cls, ids) in sorted(by_obj.items()):
        ys, xs = np.nonzero(np.isin(seg, ids))
        if len(xs) == 0:
            continue                                         # 사진에 안 보임 (가려졌거나 화면 밖)
        x0, y0, x1, y1 = int(xs.min()), int(ys.min()), min(int(xs.max()), w - 1), min(int(ys.max()), h - 1)
        if x1 > x0 and y1 > y0:
            out.append((cls, x0, y0, x1, y1))
    return out


def _draw_boxes(pil, boxes):
    """사진에 박스와 이름(영문)을 그린 복사본."""
    from PIL import ImageDraw
    im = pil.copy()
    dr = ImageDraw.Draw(im)
    for cls, x0, y0, x1, y1 in boxes:
        c = BOX_COLORS.get(cls, (255, 0, 255))
        dr.rectangle([x0, y0, x1, y1], outline=c, width=3)
        tw = 7 * len(cls) + 8
        dr.rectangle([x0, max(0, y0 - 16), x0 + tw, max(0, y0 - 16) + 16], fill=c)
        dr.text((x0 + 4, max(0, y0 - 16) + 2), cls, fill=(0, 0, 0))
    return im


def _write_csv(path, keys, rows):
    with open(path, "w", encoding="utf-8-sig", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k) for k in keys})


def _write_labels(out_dir, rows):
    _write_csv(os.path.join(out_dir, "labels.csv"), LABEL_KEYS, rows)


class DriveRecorder:
    """주행 중 통로 입구에서 Cosmos 에 보낸 사진을 저장한다 (확장의 '데이터 저장' 이 켜져 있을 때).
    datasets/aisle_drive_<시각>/ 에 주행 한 번이 폴더 하나:
      images/               Cosmos 에 보낸 원본 사진
      images_labeled/       같은 사진에 박스와 이름을 그려 넣은 것
      labels.csv            박스 좌표 (정확도 시험과 같은 형식, 물체 하나 = 한 줄)
      cosmos.csv            사진마다 Cosmos 의 응답 (판단·행동·장애물·이유·확신·응답 시간)
    서버 주소·계정 같은 민감한 정보는 기록하지 않는다. 한 장 저장할 때마다 CSV 를 다시 써서 주행이 중간에 끝나도 남는다."""

    def __init__(self, out_dir=None):
        self.dir = out_dir or os.path.join(ROOT, "datasets", f"aisle_drive_{time.strftime('%Y%m%d_%H%M%S')}")
        os.makedirs(os.path.join(self.dir, "images"), exist_ok=True)
        os.makedirs(os.path.join(self.dir, "images_labeled"), exist_ok=True)
        self.n = 0
        self.label_rows, self.cosmos_rows = [], []

    def add(self, aisle, state, rgb, boxes, res):
        """사진 한 장 기록. state = 그 순간의 정답 상태(incidents.OBSTACLES 의 키, 또는 worker), boxes = _read_boxes 결과, res = Cosmos 응답."""
        from PIL import Image
        self.n += 1
        name = f"img_{self.n:04d}_aisle{amr_scene.aisle_no(aisle)}.png"
        pil = Image.fromarray(rgb).convert("RGB")
        pil.save(os.path.join(self.dir, "images", name))
        _draw_boxes(pil, boxes or []).save(os.path.join(self.dir, "images_labeled", name))
        base = {"image": f"images/{name}", "aisle_no": amr_scene.aisle_no(aisle), "state": state,
                "truth_passable": incidents.passable(state)}
        if boxes:
            self.label_rows += [dict(base, label=c, x_min=x0, y_min=y0, x_max=x1, y_max=y1) for c, x0, y0, x1, y1 in boxes]
        else:
            self.label_rows.append(dict(base, label="", x_min="", y_min="", x_max="", y_max=""))
        self.cosmos_rows.append(dict(base, decision=res.get("decision"), action=res.get("action"),
                                     seen="; ".join(res.get("seen") or []),
                                     obstacle=(res.get("obstacle") or ""), reason=(res.get("reason") or ""),
                                     confidence=res.get("confidence"), sec=res.get("sec"), error=res.get("error")))
        _write_labels(self.dir, self.label_rows)
        _write_csv(os.path.join(self.dir, "cosmos.csv"), COSMOS_KEYS, self.cosmos_rows)


async def generate(n=24, seed=0, out_dir=None, progress=None, should_stop=None):
    """무작위 장애물 배치로 사진 n 장과 정답 라벨을 만든다. 만든 폴더 경로를 돌려준다."""
    from PIL import Image
    rnd = random.Random(seed)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    out_dir = out_dir or os.path.join(ROOT, "datasets", f"aisle_random_{stamp}")
    os.makedirs(os.path.join(out_dir, "images"), exist_ok=True)
    os.makedirs(os.path.join(out_dir, "images_labeled"), exist_ok=True)
    await _ensure_stopped()
    capture.prepare(amr_scene.FRONT_CAM)                 # 렌더 프로덕트는 멈춘 상태에서 한 번만 만든다
    seg = _seg_annotator(capture._ensure(amr_scene.FRONT_CAM, capture.RESOLUTION))   # '물체 번호 이미지' 어노테이터도 같은 렌더 프로덕트에
    await _frames(10)
    lights = _lights()
    rows = []
    amr_scene.set_goal_marker_visible(False)             # 목적지·안전 영역·번호 원 표시는 사진에 나오면 안 된다 (Cosmos 가 장애물로 볼 수 있음)
    await _frames(5)
    try:
        for i in range(n):
            if should_stop and should_stop():             # 중지 요청: 지금까지 만든 사진과 라벨만 남기고 끝낸다
                break
            aisle = rnd.choice(list(amr_scene.AISLES))
            state = rnd.choice(STATES)
            layout_seed = seed * 100000 + i
            dx, dy, dyaw = rnd.uniform(-DX, DX), rnd.uniform(-DY, DY), rnd.uniform(-DYAW, DYAW)
            k = rnd.uniform(*LIGHT_RANGE)
            incidents.clear_all()
            incidents.set_aisle(aisle, state, seed=layout_seed)
            amr_scene.set_robot_pose(amr_scene.AISLES[aisle] + dx, amr_scene.ENTRY_Y + dy, 90.0 + dyaw)
            _scale_lights(lights, k)
            await _frames(5)
            # Play 한 채로 찍는다: 사진(RGB)과 박스(물체 번호 이미지)가 같은 프레임에서 나온다. 예전처럼 Play 했다 멈춘 뒤 찍으면
            # RGB 는 새 화면인데 물체 번호 이미지는 Play 중의 옛 화면을 돌려줘 박스가 사진보다 약 25 px 위로 어긋났다 (측정).
            # 실제 주행에서 Cosmos 사진도 Play 중에 찍으므로 조건이 더 비슷하기도 하다.
            omni.timeline.get_timeline_interface().play()
            await _frames(10)
            rgb = await _grab_front()
            if rgb is None:
                await _ensure_stopped()
                raise RuntimeError("카메라 사진을 가져오지 못했습니다")
            boxes = _read_boxes(seg, rgb.shape[1], rgb.shape[0])      # 멈추기 전에 읽는다
            await _ensure_stopped()
            name = f"img_{i + 1:04d}.png"
            pil = Image.fromarray(rgb).convert("RGB")
            pil.save(os.path.join(out_dir, "images", name))
            _draw_boxes(pil, boxes).save(os.path.join(out_dir, "images_labeled", name))
            base = {"image": f"images/{name}", "aisle_no": amr_scene.aisle_no(aisle), "state": state,
                    "truth_passable": incidents.passable(state)}
            if boxes:
                rows += [dict(base, label=c, x_min=x0, y_min=y0, x_max=x1, y_max=y1) for c, x0, y0, x1, y1 in boxes]
            else:
                rows.append(dict(base, label="", x_min="", y_min="", x_max="", y_max=""))       # 물체 없음(비움)도 사진은 한 줄 남긴다
            _set_status(stage="generate", done=i + 1, total=n, out_dir=out_dir)
            if progress:
                progress(i + 1, n)
    finally:
        _detach_seg()                                     # 주행에는 필요 없는 어노테이터를 뗀다
        amr_scene.set_goal_marker_visible(True)           # 표시물 다시 보이게
        _scale_lights(lights, 1.0)                        # 조명을 원래 세기로
        incidents.clear_all()
        amr_scene.set_robot_pose(*amr_scene.START, 90.0)
    _write_labels(out_dir, rows)
    return out_dir


def _load_rows(folder):
    """labels.csv 를 사진 단위로 묶어 읽는다 (물체마다 한 줄이라 사진 하나가 여러 줄일 수 있다). 사진마다 한 항목."""
    letter = {amr_scene.aisle_no(a): a for a in amr_scene.AISLES}
    seen, rows = set(), []
    with open(os.path.join(folder, "labels.csv"), encoding="utf-8-sig", newline="") as f:
        for r in csv.DictReader(f):
            if r["image"] in seen:
                continue
            seen.add(r["image"])
            rows.append({"image": r["image"], "aisle_no": r["aisle_no"], "aisle": letter.get(str(r["aisle_no"])),
                         "state": r["state"],
                         "truth_passable": incidents.passable(r["state"]) if r["state"] in incidents.OBSTACLES
                         else str(r["truth_passable"]).strip().lower() == "true"})
    return rows


async def judge_folder(folder, progress=None, should_stop=None):
    """폴더의 사진을 Cosmos 에 보내 판단하고 정확도 표를 만든다. 지금의 프롬프트 칸 글과 '장애물별 판단' 으로 한 번 돌리고
    results_custom.csv 와 summary.md 를 쓴다. 정답(truth_passable)은 사진을 만든 때가 아니라 지금의 장애물별 판단으로 다시 계산한다."""
    import base64
    import io
    from PIL import Image
    import amr_config
    folder = folder if os.path.isabs(folder) else os.path.join(ROOT, folder)
    rows = _load_rows(folder)
    loop = asyncio.get_event_loop()
    key = "custom"
    summary = {}
    res_rows = []
    for i, r in enumerate(rows):
        if should_stop and should_stop():
            break
        pil = Image.open(os.path.join(folder, r["image"])).convert("RGB")
        buf = io.BytesIO()
        if amr_config.IMAGE_FORMAT == "jpeg":
            pil.save(buf, format="JPEG", quality=amr_config.JPEG_QUALITY)
        else:
            pil.save(buf, format="PNG")
        b64 = base64.b64encode(buf.getvalue()).decode("ascii")
        res = await loop.run_in_executor(None, aisle_judge.judge, b64, r["aisle"])
        res_rows.append({"image": r["image"], "aisle_no": r["aisle_no"], "state": r["state"], "truth_passable": r["truth_passable"],
                         "decision": res["decision"], "action": res.get("action"), "seen": "; ".join(res.get("seen") or [])[:60],
                         "obstacle": (res.get("obstacle") or "")[:40],
                         "reason": (res.get("reason") or "")[:80], "sec": res.get("sec"), "error": res.get("error")})
        _set_status(stage="judge", prompt=key, done=i + 1, total=len(rows))
        if progress:
            progress(key, i + 1, len(rows))
    if res_rows:
        with open(os.path.join(folder, f"results_{key}.csv"), "w", encoding="utf-8-sig", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(res_rows[0].keys()))
            w.writeheader()
            w.writerows(res_rows)
        summary[key] = _score(res_rows)
    _write_summary(folder, rows, summary)
    return summary


def _score(res_rows):
    ok = [r for r in res_rows if r["decision"] is not None and not r["error"]]
    by = {}
    for st in STATES:
        sub = [r for r in ok if r["state"] == st]
        by[st] = {"n": len([r for r in res_rows if r["state"] == st]), "correct": sum(1 for r in sub if r["decision"] == r["truth_passable"])}
    ko = incidents.SHORT
    wrong = [f"통로{r['aisle_no']} {ko.get(r['state'], r['state'])}" for r in ok if r["decision"] != r["truth_passable"]]
    return {"wrong": wrong, "n": len(res_rows), "judged": len(ok), "correct": sum(1 for r in ok if r["decision"] == r["truth_passable"]),
            "danger": sum(1 for r in ok if (not r["truth_passable"]) and r["decision"]),            # 막힌 통로를 통과로 본 것
            "false_block": sum(1 for r in ok if r["truth_passable"] and not r["decision"]),         # 빈 통로를 막힘으로 본 것
            "errors": len(res_rows) - len(ok),
            "avg_sec": round(sum((r["sec"] or 0) for r in res_rows) / max(1, len(res_rows)), 1), "by_state": by}


def _write_summary(folder, rows, summary):
    ko = incidents.SHORT
    lines = [f"# 통로 사진 판단 정확도 ({len(rows)}장)", "",
             "| 시험 | 정확도 | 위험한 오판(막힌 통로를 통과) | 빈 통로를 막힘으로 본 것 | 오류 | 평균 응답(초) |", "|---|---|---|---|---|---|"]
    for k, s in summary.items():
        lines.append(f"| 현재 프롬프트 | {s['correct']} / {s['n']} | {s['danger']} | {s['false_block']} | {s['errors']} | {s['avg_sec']} |")
    lines += ["", "## 상태별 정답 수", "", "| 시험 | " + " | ".join(ko[x] for x in STATES) + " |", "|---|" + "---|" * len(STATES)]
    for k, s in summary.items():
        lines.append(f"| 현재 프롬프트 | " + " | ".join(f"{s['by_state'][x]['correct']} / {s['by_state'][x]['n']}" for x in STATES) + " |")
    # 시험에 쓴 장애물별 판단과 프롬프트 (나중에 결과를 비교할 수 있게 남긴다)
    mode_ko = {"block": "막힘", "pass": "통과"}
    lines += ["", "## 장애물별 판단 (정답 기준)", ""] + [
        f"- {incidents.OBSTACLES[k]['name']}: {mode_ko[incidents.get_rule(k)]}" for k in incidents.rule_keys()]
    lines += ["", "## 시험에 쓴 프롬프트", "", "```", aisle_judge.effective_system(), "```"]
    with open(os.path.join(folder, "summary.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


async def run(n=24, seed=0, out_dir=None, judge=False, progress=None):
    """사진 생성(+선택: Cosmos 판단). 결과: {"folder": 폴더, "summary": 정확도 요약 또는 None}"""
    folder = await generate(n=n, seed=seed, out_dir=out_dir, progress=progress)
    summary = None
    if judge:
        summary = await judge_folder(folder)
    _set_status(stage="done", out_dir=folder)
    return {"folder": folder, "summary": summary}
