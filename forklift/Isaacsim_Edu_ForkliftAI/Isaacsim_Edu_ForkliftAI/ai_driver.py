"""AI 운전자: 키보드 대신 Lesson 1의 조작 키를 눌러 학습된 경로를 따라간다.

Lesson 1의 제어 노드는 매 프레임 `forklift_omnigraph_nodes._is_key_down(stage, key_node)`를 호출한다.
AI 운전 중에는 이 함수가 주행 키 6개에 대해 경로 추종기의 판단을 돌려준다(카메라 전환은 실제 키보드).
그래서 사람이 키를 누른 것과 똑같이 움직이며, Lesson 1 파일은 수정하지 않는다.

YOLO 모델이 있으면 파렛트까지 거리(포크가 끝까지 들어갔는지)를 시뮬레이터 대신 포크 카메라로 잰다(`PalletEyes`).
이때 시뮬레이터의 파렛트 위치는 채점(선반에 올라갔는가)에만 쓴다.
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
SETTLE_S = 1.5  # 포크가 빠진 뒤 파렛트가 멈출 때까지 기다렸다가 판정
BACK_OUT_MAX_M = 1.5  # 학습 경로가 끝나도 포크가 파렛트 안에 있으면 최대 이만큼 더 똑바로 후진
LIFTED_M = 0.2  # 포크가 이 높이 이상이면 파렛트가 포크에 실려 있어야 함
MAX_PALLET_TILT_DEG = 10.0  # 들린 포크 위에서 이보다 더 기울면 포크 끝에 걸쳐 있는 것


def start_overlap(stage, x, y, yaw_deg, lift):
    """이 자세에서 이미 무언가와 겹쳐 출발할 수 없는 이유(패널용 한국어 문구), 없으면 None.
    선반·기둥 등 고정 물체는 Lesson 1의 충돌 검사로 판단(물리 scene에 묻기 때문에 시뮬레이션 중에만 동작).
    그 검사가 통과시키는 파렛트(움직이는 물체)는 외곽선으로 판단한다."""
    if fon._pose_blocked(stage, x, y, math.radians(yaw_deg), lift):
        what = "선반·기둥 같은 다른 물체"
    else:
        px, py, _pz, pyaw = pallet_pose(stage)
        if not P.hits_pallet(x, y, yaw_deg, (px, py, pyaw), straight=True):
            return None
        what = "파렛트"
    return f"지게차가 {what}와 겹쳐 있어 시작할 수 없습니다. 출발 위치를 옮겨 주세요."


class PalletEyes:
    """파렛트로 들어가는 동안 포크 카메라의 YOLO로 파렛트 위치를 잰다.

    파렛트 아랫변이 보이는 동안(약 2.4 m 이상) 검출마다 거리표로 거리를 구해 지게차 자세로 바닥 위치로 바꾸고,
    최근 값들의 중앙값을 유지한다. 더 가까우면 아랫변이 화면 밖으로 나가므로 지게차 자신의 이동량으로 거리를 줄여 나간다.
    파렛트가 밀려나면 아랫변이 다시 보이는데, 유지 중인 위치와 `MOVED_M` 넘게 차이 나면 파렛트가 움직인 것으로 보고
    이전 측정을 버린 뒤 새로 잰다.
    """

    EVERY_S = 0.2  # 초당 5회 검출
    KEEP = 5
    MAX_LIFT_M = 0.15  # 거리표는 포크(와 카메라)를 내린 상태로 학습했음
    MOVED_M = 0.25  # 거리표 자체의 오차가 2.5-3 m에서 약 +-0.2 m

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
        self.history = []  # 진단용 (sim time, 박스 아랫변 또는 None, 측정 거리 또는 None)

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
            return  # 너무 가까워 측정 불가: 마지막 측정값에서 계속 줄여 나감
        est = V.estimate_ahead(box, self.table)
        self.history.append((round(sim_time, 2), bottom, None if est is None else round(est, 2)))
        if est is None:
            return
        self.last_estimate = est
        yr = math.radians(yaw_deg)
        new = (x + math.sin(yr) * est, y - math.cos(yr) * est)
        if self.spot is not None and math.dist(new, self.spot) > self.MOVED_M:
            self._spots = []  # 파렛트가 움직임(포크에 밀림): 새로 측정
        self._spots = (self._spots + [new])[-self.KEEP:]
        self.spot = tuple(float(v) for v in np.median(np.array(self._spots), axis=0))

    def ahead(self, x, y, yaw_deg):
        if self.spot is None:
            return None
        return P.pallet_ahead(x, y, yaw_deg, *self.spot)[0]


class AIDriver:
    def __init__(self, paths_file, model_path=None, blind=False, reposition=False):
        """model_path: YOLO 모델(AI의 눈). 없으면 시뮬레이터의 실제 파렛트 거리를 쓰고, `blind`이면 거리를 쓰지 않는다
        (사람 경로만큼 포크를 넣고 거기서 들어 올림). 쓸 수 없는 모델(`paths.json`에 거리표 없음)도 blind로 취급."""
        self.paths = P.load(paths_file)
        use_eyes = model_path and self.paths.get("vision")
        self.eyes = PalletEyes(model_path, self.paths["vision"]) if use_eyes else None
        self.blind = blind or (model_path is not None and self.eyes is None)
        self.seat_check = None  # 파렛트를 들어 올린 순간의 (실제 거리, AI가 쓴 거리)
        self._k_seen = None
        self._pose = None
        self.active = False
        self.keys = {k: False for k in P.KEYS}
        self.follower = None
        self.plan_text = ""
        self.move = None  # 경로 전에 운전하는 짧은 자리 이동 (plan 참고)
        self.move_text = ""
        self._repositions = 0
        self._max_repositions = P.APPROACH_MAX_STEPS if reposition else 0  # 패널의 '자리 고쳐 잡기'
        self._orig = None
        self._t_last = None
        self._t0 = None
        self._t_done = None
        self.result = None
        self.tilt_deg = None

    def plan(self, stage, x, y, yaw_deg):
        """이 자세에서 학습된 경로를 모두 리허설해, 아무것도 들이받지 않는 경로 중 가장 좋은 것을 고른다
        (Lesson 1의 충돌 검사 + 그 검사가 밀고 가도록 허용하는 파렛트). 통과하는 경로가 없으면 False.
        충돌 검사가 물리 scene에 묻기 때문에 시뮬레이션 실행 중이어야 한다.
        '자리 고쳐 잡기'를 켜면 가까운 시연만 빠르게 본다. 경로가 없으면 시연 출발점 쪽으로 짧게 이동하고
        (`paths.approach_move`) 실제로 멈춘 곳에서 다시 본다. 더 가까워지는 이동이 없을 때만 전체를 탐색한다."""
        px, py, _pz, pyaw = pallet_pose(stage)
        pallet = (px, py, pyaw)

        def blocked(old, new):
            return fon._refuse_pose(stage, (old[0], old[1], math.radians(old[2]), old[3]),
                                    (new[0], new[1], math.radians(new[2]), new[3]))

        if self._max_repositions:  # 먼저 짧게 탐색
            demo, la, miss, tried, rejected = P.plan(self.paths, x, y, yaw_deg, blocked, pallet=pallet,
                                                     nearest=P.QUICK_DEMOS, lookaheads=P.QUICK_LOOKAHEADS_M)
        else:
            demo, la, miss, tried, rejected = P.plan(self.paths, x, y, yaw_deg, blocked, pallet=pallet)
        if demo is None and self._max_repositions:
            self.follower = None
            if self._repositions < self._max_repositions:
                found = P.approach_move(self.paths, x, y, yaw_deg, blocked, pallet, collides=self._collides(stage))
                if found is not None:  # 시연 출발점 쪽으로 조금 이동한 뒤 다시 탐색
                    self._repositions += 1
                    self.move = P.MoveDriver(found[0])
                    self.move_text = P.describe_move(found[0])
                    self.plan_text = (f"바로 갈 수 있는 경로가 없어 녹화 출발점 쪽으로 조금 움직입니다 "
                                      f"({self._repositions}/{self._max_repositions}번째: {self.move_text}). 움직인 뒤 다시 생각합니다.")
                    return True
            # 더 가까워지는 이동이 없음: 포기 전에 여기서 한 번 전체 탐색
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
        """Lesson 1의 충돌 검사: 이 자세(x, y, yaw_deg, lift)의 지게차가 선반·기둥에 닿는가?"""
        return lambda x, y, yaw_deg, lift: fon._pose_blocked(stage, x, y, math.radians(yaw_deg), lift)

    def check_start(self, stage, x, y, yaw_deg, lift):
        """AI가 여기서 출발할 수 없는 이유(패널용 한국어), 없으면 None: 이미 선반·기둥·파렛트에 닿아 있는 경우.
        목표 선반 뒤쪽 출발은 plan()에 맡긴다: 돌아갈 수 있는 학습 경로가 있으면 출발."""
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
        """매 프레임 누를 키를 정한다. 끝나면 'success' / 'stuck' / 'missed' / 'timeout' 등, 아니면 None."""
        if not self.active:
            return None
        dt = 0.0 if self._t_last is None else max(sim_time - self._t_last, 0.0)
        self._t_last = sim_time
        s = forklift_state(stage)
        if self.move is not None:  # 자리 이동 먼저 (출발 위치에서 갈 경로가 없었음)
            self.keys = self.move.step(s["x"], s["y"], fon._current_speed_mps)
            if self.move.done:
                self.move = None
                self.keys = {k: False for k in P.KEYS}
                if not self.plan(stage, s["x"], s["y"], s["yaw_deg"]):  # 이동이 실제로 끝난 위치에서
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
                self.eyes.reset()  # 파렛트로 들어가기 전에 새로 측정
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
        # AI의 감각이 아닌 심판 판정(아래 선반 판정과 같음): 들어 올린 뒤 파렛트는 포크 위에 수평으로 실려야 한다.
        # 포크 끝에 걸쳐 있으면 선반 쪽으로 끌려가므로 멈춘다.
        if (not f.done and f.k >= (f.demo.get("insert_segment") or 0) and s["lift_m"] > LIFTED_M):
            tilt = pallet_tilt_deg(stage)
            if tilt > MAX_PALLET_TILT_DEG:
                self.tilt_deg = tilt
                self.result = "tilted"
                return self.result
        # 학습 경로를 끝까지 운전하고, 포크가 파렛트에서 완전히 빠지고, 파렛트가 잠시 안정된 뒤에만 판정
        # (내려놓는 도중에는 판정하지 않음).
        forks_out = math.hypot(px - s["x"], py - s["y"]) > FORKS_CLEAR_M
        if f.done:
            if self._back_from is None:
                self._back_from = (s["x"], s["y"])
            backed = math.hypot(s["x"] - self._back_from[0], s["y"] - self._back_from[1])
            if not forks_out and backed < BACK_OUT_MAX_M:
                self.keys = {k: k == "backward" for k in P.KEYS}  # 포크가 아직 안에 있음: 똑바로 후진
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
