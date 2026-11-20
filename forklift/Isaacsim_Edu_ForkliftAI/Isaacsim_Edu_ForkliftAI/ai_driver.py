"""AI driver: follows a learned route by pressing Lesson 1's control keys instead of the keyboard.

Lesson 1's control nodes ask `forklift_omnigraph_nodes._is_key_down(stage, key_node)` every frame.
While the AI drives, that function answers with the path follower's decision for the six driving
keys (camera cycling still follows the real keyboard), so the forklift moves exactly as if a person
held those keys. Nothing in Lesson 1 is modified on disk.

With a YOLO model, how far the pallet is (to know when the forks are all the way in) comes from the
fork camera instead of the simulator: see PalletEyes. The simulator's pallet pose is then only used
to score the run (did it end up on the shelf?).
"""

from __future__ import annotations

import math

import numpy as np
import Isaacsim_Edu_Lv2_Lesson1.forklift_omnigraph_nodes as fon

from . import paths as P
from . import vision as V
from .recorder import (FORKS_CLEAR_M, IMAGE_SIZE, ensure_ai_camera, forklift_state, pallet_pose, pallet_tilt_deg,
                       placed_on_shelf, shelf_footprint)

KEY_OF_PATH = {
    fon.KEY_FORWARD_PATH: "forward", fon.KEY_BACKWARD_PATH: "backward",
    fon.KEY_LEFT_PATH: "left", fon.KEY_RIGHT_PATH: "right",
    fon.KEY_LIFT_UP_PATH: "lift_up", fon.KEY_LIFT_DOWN_PATH: "lift_down",
}
TIMEOUT_S = 180.0
SETTLE_S = 1.5  # after the forks are out, give the pallet this long to come to rest before judging
BACK_OUT_MAX_M = 1.5  # if the learned route ends with the forks still in the pallet, back straight out this much more
LIFTED_M = 0.2  # forks this high: the pallet should be riding on them
MAX_PALLET_TILT_DEG = 10.0  # leaning more than this on lifted forks = hanging off the fork tips


def start_overlap(stage, x, y, yaw_deg, lift):
    """Why the forklift cannot start at this pose because it already overlaps something (Korean, for the
    panel), or None. Racks, pillars and other fixed objects: Lesson 1's own test, which asks the physics
    scene -- only while the simulation runs. The pallet (a moving body that test lets through): its outline."""
    if fon._pose_blocked(stage, x, y, math.radians(yaw_deg), lift):
        what = "선반·기둥 같은 다른 물체"
    else:
        px, py, _pz, pyaw = pallet_pose(stage)
        if not P.hits_pallet(x, y, yaw_deg, (px, py, pyaw), straight=True):
            return None
        what = "파렛트"
    return f"지게차가 {what}와 겹쳐 있어 시작할 수 없습니다. 출발 위치를 옮겨 주세요."


class PalletEyes:
    """YOLO on the fork camera -> where the pallet stands, while driving into it.

    While the pallet's bottom edge is in view (more than ~2.4 m away) each detection gives a distance
    (learned table), turned into a floor position with the forklift's own pose; the median of the
    latest ones is kept. Closer than that the bottom edge leaves the image, and the distance is
    counted down from the forklift's own motion. If the pallet slides away (pushed), its bottom edge
    shows again: a measurement that disagrees with the kept position by more than MOVED_M means the
    pallet moved, so the old measurements are dropped and it is measured afresh.
    """

    EVERY_S = 0.2  # 5 detections per second
    KEEP = 5
    MAX_LIFT_M = 0.15  # the table was learned with the forks (and the camera) down
    MOVED_M = 0.25  # the distance table's own spread is about +-0.2 m at 2.5-3 m

    def __init__(self, model_path, table):
        self.detector = V.Detector(model_path)
        self.table = table
        self._rp = self._rgb = None
        self.reset()

    def reset(self):
        self._spots = []
        self._t_last = None
        self.spot = None
        self.last_box = None
        self.last_estimate = None
        self.detections = 0
        self.history = []  # (sim time, box bottom or None, distance measured or None) for diagnosis

    def attach(self, stage):
        import omni.replicator.core as rep

        self._rp = rep.create.render_product(ensure_ai_camera(stage), IMAGE_SIZE)
        self._rgb = rep.AnnotatorRegistry.get_annotator("rgb")
        self._rgb.attach(self._rp)

    def detach(self):
        try:
            if self._rgb is not None:
                self._rgb.detach()
            if self._rp is not None:
                self._rp.destroy()
        except Exception:  # noqa: BLE001
            pass
        self._rp = self._rgb = None

    def look(self, sim_time, x, y, yaw_deg, lift):
        if self._rgb is None or lift > self.MAX_LIFT_M:
            return
        if self._t_last is not None and sim_time - self._t_last < self.EVERY_S - 1e-6:
            return
        self._t_last = sim_time
        rgb = self._rgb.get_data()
        if rgb is None or not hasattr(rgb, "shape") or rgb.size == 0:
            return
        box = V.target_pallet(self.detector.detect(np.asarray(rgb)[..., :3]))
        self.detections += 1
        self.last_box = box
        bottom = None if box is None else round(box[2] + box[4] / 2, 3)
        if box is None or bottom >= V.CLIPPED_BOTTOM:
            self.history.append((round(sim_time, 2), bottom, None))
            return  # too close to measure: keep counting down from the last measurement
        est = V.estimate_ahead(box, self.table)
        self.history.append((round(sim_time, 2), bottom, None if est is None else round(est, 2)))
        if est is None:
            return
        self.last_estimate = est
        yr = math.radians(yaw_deg)
        new = (x + math.sin(yr) * est, y - math.cos(yr) * est)
        if self.spot is not None and math.dist(new, self.spot) > self.MOVED_M:
            self._spots = []  # the pallet moved (pushed along by the forks): measure it afresh
        self._spots = (self._spots + [new])[-self.KEEP:]
        self.spot = tuple(float(v) for v in np.median(np.array(self._spots), axis=0))

    def ahead(self, x, y, yaw_deg):
        if self.spot is None:
            return None
        return P.pallet_ahead(x, y, yaw_deg, *self.spot)[0]


class AIDriver:
    def __init__(self, paths_file, model_path=None, blind=False, reposition=False):
        """model_path: the YOLO model (the AI's eyes); without it the simulator's true pallet distance is
        used -- or, if `blind`, none at all (the forks go in as far as the person's route and lift there).
        A model that cannot be used (no distance table in paths.json) counts as blind too."""
        self.paths = P.load(paths_file)
        use_eyes = model_path and self.paths.get("vision")
        self.eyes = PalletEyes(model_path, self.paths["vision"]) if use_eyes else None
        self.blind = blind or (model_path is not None and self.eyes is None)
        self.seat_check = None  # (true distance, distance the AI used) when it lifted the pallet
        self._k_seen = None
        self._pose = None
        self.active = False
        self.keys = {k: False for k in P.KEYS}
        self.follower = None
        self.plan_text = ""
        self.move = None  # a short repositioning move driven before the route (see plan)
        self.move_text = ""
        self._repositions = 0
        self._max_repositions = P.APPROACH_MAX_STEPS if reposition else 0  # the panel's '자리 고쳐 잡기'
        self._orig = None
        self._t_last = None
        self._t0 = None
        self._t_done = None
        self.result = None
        self.tilt_deg = None

    def plan(self, stage, x, y, yaw_deg):
        """Rehearse every learned route from this pose and keep the best one that does not run into
        anything (Lesson 1's own collision test, plus the pallet, which that test lets the forklift
        push). False if none gets through. Needs the simulation running: the collision test asks the
        physics scene. With '자리 고쳐 잡기' it only takes a quick look (the nearest demonstrations); no route ->
        drive a short move toward where the demonstrations started (paths.approach_move) and look again from
        where it really ended, step by step; a full look only once no move gets it any closer."""
        px, py, _pz, pyaw = pallet_pose(stage)
        pallet = (px, py, pyaw)

        def blocked(old, new):
            return fon._refuse_pose(stage, (old[0], old[1], math.radians(old[2]), old[3]),
                                    (new[0], new[1], math.radians(new[2]), new[3]))

        if self._max_repositions:  # think briefly first
            demo, la, miss, tried, rejected = P.plan(self.paths, x, y, yaw_deg, blocked, pallet=pallet,
                                                     nearest=P.QUICK_DEMOS, lookaheads=P.QUICK_LOOKAHEADS_M)
        else:
            demo, la, miss, tried, rejected = P.plan(self.paths, x, y, yaw_deg, blocked, pallet=pallet)
        if demo is None and self._max_repositions:
            self.follower = None
            if self._repositions < self._max_repositions:
                found = P.approach_move(self.paths, x, y, yaw_deg, blocked, pallet, collides=self._collides(stage))
                if found is not None:  # move a little toward where the demonstrations started, then think again
                    self._repositions += 1
                    self.move = P.MoveDriver(found[0])
                    self.move_text = P.describe_move(found[0])
                    self.plan_text = (f"바로 갈 수 있는 경로가 없어 녹화 출발점 쪽으로 조금 움직입니다 "
                                      f"({self._repositions}/{self._max_repositions}번째: {self.move_text}). 움직인 뒤 다시 생각합니다.")
                    return True
            # no move gets it any closer: one full look from here before giving up
            demo, la, miss, tried, rejected = P.plan(self.paths, x, y, yaw_deg, blocked, pallet=pallet)
        hit = f", 부딪히는 {rejected}가지 제외" if rejected else ""
        if demo is None:
            self.follower = None
            if not self._max_repositions:
                tried_move = " ('자리 고쳐 잡기'를 켜면 조금씩 움직여 가며 다시 찾아봅니다.)"
            elif self._repositions:
                tried_move = f" 녹화 출발점 쪽으로 {self._repositions}번 움직여 봤지만 더 가까워질 수 없어 멈췄습니다."
            else:
                tried_move = " 녹화 출발점 쪽으로 더 가까워지는 짧은 움직임도 없습니다."
            self.plan_text = (f"연습한 {len(tried)}가지 방법 모두 파렛트 구멍에 정확히 닿지 못하거나 부딪혔습니다{hit}.{tried_move} "
                              "이 출발 위치에서 갈 수 있는 학습된 경로가 없으니, 출발 위치를 바꾸거나 "
                              "이 근처에서 출발하는 시연을 더 녹화하세요.")
            return False
        self.move = None
        self.follower = P.PathFollower(demo, la, self.paths.get("seat_ahead_m"))
        self.plan_text = (f"{len(tried)}가지 방법을 머릿속으로 연습{hit} → '{demo['name']}' 경로 선택 "
                          f"(예상 포크 끝 오차 {miss * 100:.0f} cm)")
        return True

    @staticmethod
    def _collides(stage):
        """Lesson 1's own test: would the forklift at this pose (x, y, yaw_deg, lift) touch a rack or pillar?"""
        return lambda x, y, yaw_deg, lift: fon._pose_blocked(stage, x, y, math.radians(yaw_deg), lift)

    def check_start(self, stage, x, y, yaw_deg, lift):
        """Why the AI cannot set off from here (Korean, for the panel), or None: touching a rack, pillar or the
        pallet already. A start behind the target rack is left to plan(): it goes if a learned route gets round."""
        return start_overlap(stage, x, y, yaw_deg, lift)

    def start(self, sim_time):
        if self._orig is None:
            self._orig = fon._is_key_down
            fon._is_key_down = self._is_key_down
        self.keys = {k: False for k in P.KEYS}
        self._t_last = None
        self._t0 = sim_time
        self._t_done = None
        self._back_from = None
        self.result = None
        self.tilt_deg = None
        self.seat_check = None
        self._k_seen = None
        self.active = True

    def attach_eyes(self, stage):
        if self.eyes is not None:
            self.eyes.attach(stage)

    def stop(self):
        if self.eyes is not None:
            self.eyes.detach()
        self.active = False
        self.keys = {k: False for k in P.KEYS}
        if self._orig is not None:
            fon._is_key_down = self._orig
            self._orig = None

    def _is_key_down(self, stage, node_path):
        key = KEY_OF_PATH.get(node_path)
        if self.active and key is not None:
            return self.keys[key]
        return self._orig(stage, node_path) if self._orig else False

    def update(self, stage, sim_time):
        """Decide keys every frame. Returns 'success' / 'stuck' / 'missed' / 'timeout' when over, else None."""
        if not self.active:
            return None
        dt = 0.0 if self._t_last is None else max(sim_time - self._t_last, 0.0)
        self._t_last = sim_time
        s = forklift_state(stage)
        if self.move is not None:  # repositioning first (no route from where it started)
            self.keys = self.move.step(s["x"], s["y"], fon._current_speed_mps)
            if self.move.done:
                self.move = None
                self.keys = {k: False for k in P.KEYS}
                if not self.plan(stage, s["x"], s["y"], s["yaw_deg"]):  # from where the move really ended
                    self.result = "no_route"
            if self.result is None and sim_time - self._t0 > TIMEOUT_S:
                self.result = "timeout"
            return self.result
        px, py, pz, _ = pallet_pose(stage)
        true_ahead, side = P.pallet_ahead(s["x"], s["y"], s["yaw_deg"], px, py)
        f = self.follower
        inserting = f.k == f.demo.get("insert_segment") and f.mode == "drive"
        if self.eyes is not None:
            if f.k != self._k_seen and inserting:
                self.eyes.reset()  # a fresh look at the pallet for the drive in
            if inserting:
                self.eyes.look(sim_time, s["x"], s["y"], s["yaw_deg"], s["lift_m"])
            ahead = self.eyes.ahead(s["x"], s["y"], s["yaw_deg"])
        else:
            ahead = None if self.blind else (true_ahead if abs(side) < 0.4 else None)
        self._k_seen = f.k
        self._pose = (s["x"], s["y"], s["yaw_deg"])
        k_before, mode_before = f.k, f.mode
        self.keys = f.step(s["x"], s["y"], s["yaw_deg"], fon._current_speed_mps,
                           fon._current_turn_rate_deg_per_sec, s["lift_m"], dt, pallet_ahead_m=ahead)
        if (self.seat_check is None and k_before == f.demo.get("insert_segment")
                and mode_before == "drive" and f.mode == "settle"):
            self.seat_check = (round(true_ahead, 3), None if ahead is None else round(ahead, 3))
        # Referee, not the AI's senses (like the on-shelf judgement below): once the forks have lifted it, the
        # pallet must ride level on them. Hanging off the fork tips it would be dragged into the rack -- stop.
        if (not f.done and f.k >= (f.demo.get("insert_segment") or 0) and s["lift_m"] > LIFTED_M):
            tilt = pallet_tilt_deg(stage)
            if tilt > MAX_PALLET_TILT_DEG:
                self.tilt_deg = tilt
                self.result = "tilted"
                return self.result
        # Judged only once the whole learned route is driven, the forks are completely out of the pallet
        # and the pallet has had a moment to settle -- never in the middle of placing it.
        forks_out = math.hypot(px - s["x"], py - s["y"]) > FORKS_CLEAR_M
        if f.done:
            if self._back_from is None:
                self._back_from = (s["x"], s["y"])
            backed = math.hypot(s["x"] - self._back_from[0], s["y"] - self._back_from[1])
            if not forks_out and backed < BACK_OUT_MAX_M:
                self.keys = {k: k == "backward" for k in P.KEYS}  # forks still in: straight back
                self._t_done = None
            elif self._t_done is None:
                self._t_done = sim_time
            elif sim_time - self._t_done >= SETTLE_S:
                on_shelf = placed_on_shelf((px, py, pz, 0.0), shelf_footprint(stage))
                self.result = "success" if on_shelf and forks_out else "missed"
        if self.result is None and f.failed:
            self.result = "not_inserted" if f.failed == "not_inserted" else "stuck"
        elif self.result is None and sim_time - self._t0 > TIMEOUT_S:
            self.result = "timeout"
        return self.result

    @property
    def progress_text(self):
        if self.move is not None:
            return f"자리 고쳐 잡는 중 ({self.move_text})"
        f = self.follower
        if f is None:
            return ""
        if f.mode == "back_off":
            return f"구간 {f.k + 1}/{len(f.segs)} (포크가 덜 들어가 뒤로 뺐다가 다시 넣는 중)"
        return f"구간 {f.k + 1}/{len(f.segs)} ({'전진' if f.segs[f.k]['dir'] > 0 else '후진'})"

    @property
    def eyes_text(self):
        e = self.eyes
        if e is None:
            return "YOLO 거리 재기 끔: 파렛트 거리를 재지 않고 녹화 경로 끝에서 들어 올림" if self.blind else "파렛트 거리: 시뮬레이터 정답"
        if e.spot is None or self._pose is None:
            return "YOLO: 파렛트 찾는 중"
        return f"YOLO: 파렛트 {e.ahead(*self._pose):.2f} m 앞"
