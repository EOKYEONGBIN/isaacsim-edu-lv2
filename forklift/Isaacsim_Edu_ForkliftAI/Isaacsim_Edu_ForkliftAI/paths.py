"""시연에서 주행 경로를 학습하고, Lesson 1의 키 조작으로 그 경로를 따라간다.

순수 Python + numpy (Kit 없음). tools/learn_paths.py와 AI 운전자가 함께 쓴다.

성공한 시연마다 학습하는 것: 전진/후진 구간으로 나눈 주행 경로(짧은 앞뒤 보정은 제거)와
경로를 따라간 포크 높이(파렛트를 들거나 내린 정지 지점 포함).
출발 전 AI는 현재 위치에서 학습된 경로를 모두 머릿속으로 주행해 보고(지게차 운동 규칙 그대로, 경로당 1초 미만),
포크가 파렛트 구멍에 가장 정확히 들어가는 경로를 골라 사람과 같은 6개 키를 누르는 pure pursuit 제어기로 따라간다.
"""

from __future__ import annotations

import json
import math

import numpy as np

KEYS = ["forward", "backward", "left", "right", "lift_up", "lift_down"]

# Lesson 1 지게차(forklift_omnigraph_nodes): 회전 속도 = 30 deg/s * steer/30 * speed/1.2
MAX_STEER_DEG = 30.0
CURV_PER_STEER_DEG = math.radians(30.0) / 30.0 / 1.2  # [1/m per deg]: 최대 0.436 1/m -> 회전 반경 2.3 m
MAX_CURV = CURV_PER_STEER_DEG * MAX_STEER_DEG

MOVING_MPS = 0.03  # 이보다 느리면 정지로 간주
MIN_SEGMENT_M = 0.30  # 이보다 짧은 앞뒤 보정은 버림
LIFT_STOP_M = 0.10  # 정지 중 포크가 이만큼 움직이면 별도 단계 (예: 파렛트 들기)
RESAMPLE_M = 0.05
MAX_APPROACH_DEG = 35.0  # 이보다 비뚤게 들어가면 포크가 미끄러져 들어가지 않고 파렛트를 밀어냄


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


# ---------------------------------------------------------------- 학습


def demo_segments(rows):
    """시연 하나의 전진/후진 구간 (rows = steps.csv 행 dict).

    [{"dir": +1/-1, "x": [...], "y": [...], "yaw": [...] (deg), "lift": [...],
      "lift_after": 다음 구간 시작 전 맞출 포크 높이}] 반환.
    """
    t = np.array([float(r["t"]) for r in rows])
    x = np.array([float(r["x"]) for r in rows])
    y = np.array([float(r["y"]) for r in rows])
    yaw = np.array([float(r["yaw_deg"]) for r in rows])
    v = np.array([float(r["speed_mps"]) for r in rows])
    lift = np.array([float(r["lift_m"]) for r in rows])
    sign = np.where(np.abs(v) < MOVING_MPS, 0, np.sign(v)).astype(int)

    # 방향이 같은 연속 주행 구간. 정지 중 포크를 올리거나 내렸으면 거기서도 끊는다
    # ("진입, 정지, 들기, 계속 전진"은 둘 다 전진이지만 두 단계)
    runs, i, hard = [], 0, False
    while i < len(sign):
        if sign[i] == 0:
            i += 1
            continue
        j, stop_at, lifted = i, None, False
        while j + 1 < len(sign) and sign[j + 1] in (0, sign[i]):
            if sign[j + 1] == 0:
                stop_at = j + 1 if stop_at is None else stop_at
            elif stop_at is not None:
                if abs(lift[j + 1] - lift[stop_at]) > LIFT_STOP_M:
                    lifted = True
                    break
                stop_at = None
            j += 1
        k = j
        while k > i and sign[k] == 0:  # 끝부분 정지는 주행 구간이 아니라 정지에 속함
            k -= 1
        runs.append([sign[i], i, k, hard])
        hard = lifted
        i = j + 1

    def length(a, b):
        return float(np.sum(np.hypot(np.diff(x[a:b + 1]), np.diff(y[a:b + 1]))))

    # 짧은 보정은 버리고(직전의 들기 정지는 유지), 같은 방향이 된 이웃 구간을 합친다
    # -- 단, 사이에 포크를 올리거나 내렸으면 합치지 않음
    kept, carry = [], False
    for r in runs:
        if length(r[1], r[2]) < MIN_SEGMENT_M:
            carry = carry or r[3]
            continue
        kept.append([r[0], r[1], r[2], r[3] or carry])
        carry = False
    merged = []
    for r in kept:
        if merged and merged[-1][0] == r[0] and not r[3]:
            merged[-1][2] = r[2]
        else:
            merged.append(r[:3])

    segs = []
    for n, (d, a, b) in enumerate(merged):
        nxt = merged[n + 1][1] if n + 1 < len(merged) else len(rows) - 1
        # 다음으로 넘어가기 전 맞출 포크 높이: 사람이 다음 구간을 시작할 때(또는 맨 끝)의 높이
        lift_after = float(lift[nxt])
        px, py, pyaw, plift = _resample(x[a:b + 1], y[a:b + 1], yaw[a:b + 1], lift[a:b + 1])
        segs.append({"dir": int(d), "x": px, "y": py, "yaw": pyaw, "lift": plift, "lift_after": lift_after,
                     "t0": float(t[a]), "t1": float(t[b])})
    return segs


def _resample(x, y, yaw, lift):
    s = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))])
    if s[-1] < 1e-6:
        return [float(x[0])], [float(y[0])], [float(yaw[0])], [float(lift[0])]
    su = np.linspace(0.0, s[-1], max(2, int(math.ceil(s[-1] / RESAMPLE_M)) + 1))
    yaw_u = np.degrees(np.unwrap(np.radians(yaw)))
    return ([round(float(v), 4) for v in np.interp(su, s, x)], [round(float(v), 4) for v in np.interp(su, s, y)],
            [round(float(v), 3) for v in np.interp(su, s, yaw_u)], [round(float(v), 4) for v in np.interp(su, s, lift)])


def insertion(segs, rows):
    """포크가 파렛트에 닿은 지점: 파렛트가 처음 움직인 순간 (사람은 들기 전에 조금 미는 경우가 많음).
    (구간 인덱스, 그때 지게차의 [x, y, yaw_deg]) 또는 (None, None) 반환."""
    p0 = [float(rows[0][k]) for k in ("pallet_x", "pallet_y", "pallet_z")]
    for r in rows:
        moved = math.dist(p0, [float(r[k]) for k in ("pallet_x", "pallet_y", "pallet_z")])
        if moved > 0.02:
            t_contact = float(r["t"])
            pose = [float(r["x"]), float(r["y"]), float(r["yaw_deg"])]
            break
    else:
        return None, None
    started = [k for k, s in enumerate(segs) if s["t0"] <= t_contact]
    if not started or segs[started[-1]]["dir"] < 0:
        return None, None  # 후진하다 닿은 경우?
    return started[-1], pose


def pallet_ahead(x, y, yaw_deg, px, py):
    """지게차 좌표계에서의 파렛트 중심: (전방 m, 왼쪽 m)."""
    yr = math.radians(yaw_deg)
    fx, fy = math.sin(yr), -math.cos(yr)
    dx, dy = px - x, py - y
    return dx * fx + dy * fy, -dx * fy + dy * fx


# 리허설의 파렛트 충돌 검사용 외곽 (지게차 좌표계: 원점 기준 전방 시작/끝 m, 반폭).
# Lesson 1 충돌 검사는 지게차가 파렛트를 미는 것을 허용하므로 파렛트로 돌진하는 경로도 통과해 버린다.
# 그래서 차체는 서 있는 파렛트에 닿으면 안 되고, 포크(두 갈래를 한 외곽으로)는 FORK_SWEEP_DEG 이상 비뚤게
# 닿으면 안 된다 (사람 시연은 최대 ~27 deg). 새 출발점에 맞게 변형한 경로는 변형이 파렛트 직전에 끝나므로
# 더 엄격하게: 포크가 곧고 중앙에 맞아야 함 (FORK_SIDE_TOL_M, FORK_ANGLE_TOL_DEG).
BODY_OUTLINE = (-1.26, 1.30, 0.58)
FORKS_OUTLINE = (1.30, 2.45, 0.45)
PALLET_HALF = (0.50, 0.61)  # 포크 진입 방향, 그 직각 방향
FORK_SWEEP_DEG = 30.0
FORK_SIDE_TOL_M = 0.10
FORK_ANGLE_TOL_DEG = 6.0


def _outline(x, y, yaw_deg, ahead0, ahead1, half_width):
    yr = math.radians(yaw_deg)
    fx, fy, lx, ly = math.sin(yr), -math.cos(yr), math.cos(yr), math.sin(yr)
    return np.array([(x + fx * a + lx * w, y + fy * a + ly * w) for a in (ahead0, ahead1) for w in (-half_width, half_width)])


def _overlap(c1, c2):
    """두 직사각형(꼭짓점 4개씩)이 겹치는가? 분리축 검사."""
    for c in (c1, c2):
        for e in (c[1] - c[0], c[2] - c[0]):
            axis = np.array([-e[1], e[0]])
            p1, p2 = c1 @ axis, c2 @ axis
            if p1.max() < p2.min() or p2.max() < p1.min():
                return False
    return True


def hits_pallet(x, y, yaw_deg, pallet, straight=False):
    """이 자세의 지게차가 서 있는 파렛트(x, y, yaw_deg)에 부딪히는가? 차체가 닿거나 포크가 FORK_SWEEP_DEG 이상
    비뚤게 닿으면 True. `straight`이면 조금이라도 중앙에서 벗어나거나 비뚤어도 True."""
    px, py, pyaw = pallet
    box = _outline(px, py, pyaw, -PALLET_HALF[0], PALLET_HALF[0], PALLET_HALF[1])
    if _overlap(_outline(x, y, yaw_deg, *BODY_OUTLINE), box):
        return True
    if _overlap(_outline(x, y, yaw_deg, *FORKS_OUTLINE), box):
        askew = abs(yaw_deg - pyaw) % 180.0
        askew = min(askew, 180.0 - askew)
        if not straight:
            return askew >= FORK_SWEEP_DEG
        return abs(pallet_ahead(x, y, yaw_deg, px, py)[1]) >= FORK_SIDE_TOL_M or askew >= FORK_ANGLE_TOL_DEG
    return False


def seated_pallet_ahead(rows):
    """사람이 파렛트를 들기 시작할 때, 즉 포크가 끝까지 들어갔을 때(파렛트가 포크 캐리지에 닿음) 파렛트의 전방 거리."""
    z0 = float(rows[0]["pallet_z"])
    r = next((r for r in rows if float(r["pallet_z"]) > z0 + 0.05), None)
    if r is None:
        return None
    return pallet_ahead(float(r["x"]), float(r["y"]), float(r["yaw_deg"]), float(r["pallet_x"]), float(r["pallet_y"]))[0]


def learn(demos):
    """demos: [(name, rows, meta)] -> `paths.json`으로 저장할 dict."""
    out, seats = [], []
    for name, rows, meta in demos:
        segs = demo_segments(rows)
        if not segs:
            continue
        k, contact = insertion(segs, rows)
        approach = None
        if contact is not None:
            # 파렛트 구멍은 파렛트 로컬 y 방향, Lesson 1 지게차는 yaw 0에서 -y 방향으로 주행
            off = (contact[2] - float(rows[0]["pallet_yaw_deg"])) % 180.0
            approach = round(min(off, 180.0 - off), 1)
        seat = seated_pallet_ahead(rows)
        if seat is not None:
            seats.append(seat)
        s0 = meta["start"]["forklift"]
        out.append({"name": name, "start": [s0["x"], s0["y"], s0["yaw_deg"]],
                    "pallet_start": list(meta["start"]["pallet"])[:2], "insert_segment": k,
                    "contact": contact, "approach_deg": approach,
                    "usable": k is not None and approach is not None and approach <= MAX_APPROACH_DEG,
                    "segments": segs})
    # 포크가 끝까지 들어간 거리: 중앙값 (비뚤게 집은 시연 하나에 흔들리지 않도록)
    seat = float(np.median(seats)) if seats else None
    return {"version": 1, "seat_ahead_m": None if seat is None else round(seat, 3), "demos": out}


def summary(paths):
    """교육생에게 보여줄 수치: AI가 학습한 내용."""
    carry, place_lift, n_segs, length = [], [], [], []
    for d in paths["demos"]:
        lifts = [v for s in d["segments"] for v in s["lift"]] + [s["lift_after"] for s in d["segments"]]
        carry.append(max(lifts))
        place_lift.append(d["segments"][-1]["lift_after"])
        n_segs.append(len(d["segments"]))
        length.append(sum(float(np.sum(np.hypot(np.diff(s["x"]), np.diff(s["y"])))) for s in d["segments"]))
    f = lambda a: (round(float(np.mean(a)), 2), round(float(np.min(a)), 2), round(float(np.max(a)), 2))  # noqa: E731
    return {"demos": len(paths["demos"]), "carry_lift_m": f(carry), "final_lift_m": f(place_lift),
            "segments": f(n_segs), "path_length_m": f(length)}


# ---------------------------------------------------------------- 경로 추종


class PathFollower:
    """학습된 경로를 키 입력으로 바꾼다 (매 프레임 step() 호출)."""

    LOOKAHEAD_M = 0.7
    CRUISE_MPS = 0.6
    STOP_DECEL = 0.5  # 구간 끝 전에 이 정도로 완만하게 감속 [m/s^2]
    END_TOL_M = 0.06
    RATE_BAND_DEG = 0.5  # deg/s
    LIFT_TOL_M = 0.02
    STUCK_S = 8.0

    SEAT_MARGIN_M = 0.03
    CREEP_MAX_M = 1.0  # 시연 끝점을 지나 포크를 끝까지 넣으려고 최대 이만큼 더 전진
    CREEP_PUSH_M = 0.3  # ...단, 이만큼 가도 파렛트가 가까워지지 않으면 멈춤 (밀고만 있는 것)
    CREEP_REMAINING_M = 0.03  # 저속 전진 속도: 정지 지점까지 이만큼 남은 것처럼 (~0.25 m/s)
    # 포크가 파렛트 깊이의 이 비율 이상 들어가야 들기 시작 (덜 들어가면 포크 끝에 걸림).
    # 부족하면 RETRY_BACK_M만큼 똑바로 후진해 한 번 더 넣어 보고, 그래도 안 되면 포기.
    LIFT_MIN_INSERT = 0.8
    PALLET_DEPTH_M = 1.0
    RETRY_BACK_M = 0.6
    RETRIES = 1

    def __init__(self, demo, lookahead_m=None, seat_ahead_m=None):
        self.demo = demo
        self.seat_ahead_m = seat_ahead_m
        if lookahead_m is not None:
            self.LOOKAHEAD_M = lookahead_m
        self.segs = demo["segments"]
        self.k = 0  # 구간
        self.i = 0  # 구간 안의 진행 인덱스
        self.mode = "drive"
        self.done = False
        self.failed = None
        self._best = None
        self._since_progress = 0.0
        self._turning = 0  # +1 CCW 키 누름, -1 CW 키 누름, 0 없음
        self._creep = None  # [x0, y0, 최소 파렛트 거리, 그때까지 주행 거리]
        self._retries = 0
        self._back_from = None  # 재시도를 위해 후진을 시작한 위치

    def step(self, x, y, yaw_deg, speed, turn_rate_deg, lift, dt, pallet_ahead_m=None):
        """이번 프레임에 누를 키: dict key -> bool. `pallet_ahead_m`: 파렛트 전방 거리 (모르면 None).
        파렛트 진입 중에는 포크가 끝까지 들어가는 즉시 멈춘다."""
        keys = {k: False for k in KEYS}
        if self.done or self.failed:
            return keys
        seg = self.segs[self.k]
        sx, sy, sl = np.asarray(seg["x"]), np.asarray(seg["y"]), np.asarray(seg["lift"])
        n = len(sx)

        if self.mode == "back_off":  # 포크가 덜 들어감: 똑바로 후진한 뒤 다시 진입
            if math.hypot(x - self._back_from[0], y - self._back_from[1]) < self.RETRY_BACK_M:
                if speed * seg["dir"] > -0.25:
                    keys["backward" if seg["dir"] > 0 else "forward"] = True
                return keys
            self.mode = "drive"

        if self.mode == "drive":
            # 진행: 이전 위치 앞쪽에서 가장 가까운 점 (경로상 뒤로 돌아가지 않음)
            lo, hi = self.i, min(n, self.i + 40)
            d2 = (sx[lo:hi] - x) ** 2 + (sy[lo:hi] - y) ** 2
            self.i = lo + int(np.argmin(d2))
            remaining = float(np.sum(np.hypot(np.diff(sx[self.i:]), np.diff(sy[self.i:])))) if self.i < n - 1 else 0.0
            end_dist = math.hypot(sx[-1] - x, sy[-1] - y)
            # 끝점을 지났는가? (마지막 스텝 방향으로 투영)
            if n >= 2:
                ex, ey = sx[-1] - sx[-2], sy[-1] - sy[-2]
                past = (x - sx[-1]) * ex + (y - sy[-1]) * ey > 0
            else:
                past = True
            inserting = (self.k == self.demo.get("insert_segment") and pallet_ahead_m is not None
                         and self.seat_ahead_m is not None)
            seated = inserting and pallet_ahead_m <= self.seat_ahead_m + self.SEAT_MARGIN_M
            at_end = (remaining < self.END_TOL_M and end_dist < 0.25) or (self.i >= n - 2 and past)
            # 파렛트가 시연보다 더 밀렸을 수 있음: 시연 끝점에서 포크가 덜 들어갔으면 같은 방향으로 천천히 더 전진
            # (지금 들면 포크 끝에 걸림).
            creeping = at_end and inserting and not seated and self._keep_creeping(x, y, pallet_ahead_m)
            if at_end and not creeping and inserting and not seated and not self._deep_enough(pallet_ahead_m):
                # 끝까지 밀어 넣어도 포크가 LIFT_MIN_INSERT만큼 안 들어감: 들지 않음
                if self._retries >= self.RETRIES:
                    self.failed = "not_inserted"
                    return keys
                self._retries += 1
                self.mode = "back_off"
                self._back_from = (x, y)
                self._creep = None
                keys["backward" if seg["dir"] > 0 else "forward"] = True
                return keys
            if seated or (at_end and not creeping):
                self.mode = "settle"
            else:
                self._drive(keys, seg, sx, sy, x, y, yaw_deg, speed, turn_rate_deg,
                            self.CREEP_REMAINING_M if creeping else remaining)
                self._lift_toward(keys, float(sl[min(self.i + 2, n - 1)]), lift)
                if not creeping:
                    self._watch_progress(remaining, dt)
                return keys

        # settle: 정지 -> 다음 단계용 포크 높이 맞춤 -> 다음 구간 시작
        if abs(speed) > 0.02:
            if seg["dir"] > 0 and speed > 0.15:
                keys["backward"] = True  # 브레이크
            elif seg["dir"] < 0 and speed < -0.15:
                keys["forward"] = True
            return keys
        if not self._lift_toward(keys, seg["lift_after"], lift):
            return keys
        if self.k + 1 < len(self.segs):
            self.k += 1
            self.i = 0
            self.mode = "drive"
            self._turning = 0
            self._creep = None
            self._best = None
            self._since_progress = 0.0
        else:
            self.done = True
        return keys

    def _drive(self, keys, seg, sx, sy, x, y, yaw_deg, speed, turn_rate_deg, remaining):
        d = seg["dir"]
        # lookahead 지점
        j, acc = self.i, 0.0
        while j < len(sx) - 1 and acc < self.LOOKAHEAD_M:
            acc += math.hypot(sx[j + 1] - sx[j], sy[j + 1] - sy[j])
            j += 1
        tx, ty = sx[j], sy[j]
        if acc < self.LOOKAHEAD_M and len(sx) >= 2:  # 끝점 너머는 마지막 방향으로 연장
            ex, ey = sx[-1] - sx[-2], sy[-1] - sy[-2]
            norm = math.hypot(ex, ey) or 1.0
            tx += ex / norm * (self.LOOKAHEAD_M - acc)
            ty += ey / norm * (self.LOOKAHEAD_M - acc)
        yaw = math.radians(yaw_deg)
        fwd = (math.sin(yaw), -math.cos(yaw))  # Lesson 1 전진 방향 = (sin yaw, -cos yaw)
        mx, my = (fwd[0] * d, fwd[1] * d)  # 진행 방향
        lx, ly = -my, mx  # 진행 방향의 왼쪽
        dx, dy = tx - x, ty - y
        ahead, left = dx * mx + dy * my, dx * lx + dy * ly
        ld2 = max(ahead * ahead + left * left, 1e-6)
        curv = 2.0 * left / ld2  # pure pursuit, 진행 방향 기준 CCW가 +
        curv = max(-MAX_CURV, min(MAX_CURV, curv))
        # Lesson 1은 Left/Right를 누르고 있을 때만 회전 (떼면 바퀴가 즉시 중앙 복귀, 회전 멈춤).
        # 진행 방향과 무관하게 heading은 곡률 * 진행 속도로 CCW 회전한다.
        # 그래서 yaw rate로 조향: 현재 회전 속도를 목표 쪽으로 미는 키를 누른다.
        # 떼는 순간 바퀴가 복귀하므로 회전 속도가 목표를 확실히 넘을 때까지 유지(히스테리시스)하고,
        # 경로가 최대 조향을 요구하는 동안은 절대 떼지 않는다.
        v_along = speed * d
        want = math.degrees(curv * max(v_along, 0.05))
        band = self.RATE_BAND_DEG
        full_lock = abs(curv) >= 0.95 * MAX_CURV
        if self._turning > 0:
            keep = want > 0 and (full_lock or turn_rate_deg < want + band)
            self._turning = 1 if keep else 0
        elif self._turning < 0:
            keep = want < 0 and (full_lock or turn_rate_deg > want - band)
            self._turning = -1 if keep else 0
        if self._turning == 0:
            if want > turn_rate_deg + band:
                self._turning = 1
            elif want < turn_rate_deg - band:
                self._turning = -1
        sgn = d if abs(speed) < 0.05 else (1 if speed > 0 else -1)  # 전진 중 Left = CCW 회전
        if self._turning:
            keys["left" if self._turning * sgn > 0 else "right"] = True

        # 속도: 순항, 구간 끝에서 멈추도록 감속, 급회전 중에는 더 느리게
        v_target = min(self.CRUISE_MPS, math.sqrt(2 * self.STOP_DECEL * max(remaining, 0.0)) + 0.08)
        v_target *= 1.0 - 0.4 * abs(curv) / MAX_CURV
        if v_along < v_target - 0.03:
            keys["forward" if d > 0 else "backward"] = True
        elif v_along > v_target + 0.25:
            keys["backward" if d > 0 else "forward"] = True  # 브레이크

    def _deep_enough(self, pallet_ahead_m):
        """포크가 파렛트 깊이의 LIFT_MIN_INSERT 이상 들어갔는가? (seat_ahead_m = 끝까지 들어감)"""
        return pallet_ahead_m <= self.seat_ahead_m + (1.0 - self.LIFT_MIN_INSERT) * self.PALLET_DEPTH_M

    def _keep_creeping(self, x, y, pallet_ahead_m):
        if self._creep is None:
            self._creep = [x, y, pallet_ahead_m, 0.0]
        driven = math.hypot(x - self._creep[0], y - self._creep[1])
        if pallet_ahead_m < self._creep[2] - 0.02:
            self._creep[2], self._creep[3] = pallet_ahead_m, driven
        return driven < self.CREEP_MAX_M and driven - self._creep[3] < self.CREEP_PUSH_M

    def _lift_toward(self, keys, target, lift):
        if lift < target - self.LIFT_TOL_M:
            keys["lift_up"] = True
            return False
        if lift > target + self.LIFT_TOL_M:
            keys["lift_down"] = True
            return False
        return True

    def _watch_progress(self, remaining, dt):
        if self._best is None or remaining < self._best - 0.05:
            self._best = remaining
            self._since_progress = 0.0
        else:
            self._since_progress += dt
            if self._since_progress > self.STUCK_S:
                self.failed = "stuck"


# ---------------------------------------------------------------- 리허설

FORK_REACH_M = 1.0  # 진입점 기준 포크 끝이 파렛트 안으로 들어가 있는 거리
LOOKAHEADS_M = (0.5, 0.7, 1.0)
COLLISION_EVERY = 3  # 충돌 검사 간격 [프레임] (이동 거리 최대 ~6 cm)

# 현재 위치에서 출발하도록 변형한 경로(warp 참고): 출발점이 가장 가까운 시연들로, 파렛트까지 거리의 이 비율에
# 걸쳐 이동량을 서서히 줄인다. 포크가 파렛트에 닿기 직전 구간은 사람이 운전한 그대로 둔다.
WARP_NEAREST = 6
WARP_BLENDS = (0.5, 1.0)
WARP_KEEP_M = 1.5
WARP_MIN_SHIFT_M = 0.05
# 리허설에서 포크 끝이 구멍을 이보다 크게 벗어나면 주행할 가치 없음: 포크가 파렛트 옆을 지나치거나 옆면에
# 부딪힘 (구멍의 여유가 대략 이 정도)
MAX_PLAN_MISS_M = 0.10


class KinematicForklift:
    """Lesson 1의 키 조작 지게차 운동(forklift_omnigraph_nodes)을 씬 없이 재현."""

    def __init__(self, x, y, yaw_deg):
        self.x, self.y, self.yaw = x, y, yaw_deg
        self.speed = self.turn_rate = self.steer = self.lift = 0.0

    @staticmethod
    def _toward(c, t, m):
        return min(c + m, t) if c < t else max(c - m, t)

    def step(self, keys, dt):
        f, b = keys["forward"], keys["backward"]
        d = 1.0 if f and not b else (-1.0 if b and not f else 0.0)
        if d != 0 and self.speed * d < 0:
            self.speed = self._toward(self.speed, 0.0, 5.0 * dt)
        elif d == 0:
            self.speed = self._toward(self.speed, 0.0, 2.5 * dt)
        else:
            self.speed = self._toward(self.speed, d * (1.2 if d > 0 else 0.8), 0.8 * dt)
        yr = math.radians(self.yaw)
        self.x += math.sin(yr) * self.speed * dt
        self.y += -math.cos(yr) * self.speed * dt
        left, right = keys["left"], keys["right"]
        for pressed, other, sign in ((left, right, 1.0), (right, left, -1.0)):
            if not pressed and other:
                continue
            self.steer = self._toward(self.steer, sign * MAX_STEER_DEG if pressed else 0.0, 90.0 * dt)
        if left != right:
            self.turn_rate = self._toward(self.turn_rate, 30.0 * (self.steer / 30.0) * (self.speed / 1.2), 60.0 * dt)
        else:
            self.turn_rate = self._toward(self.turn_rate, 0.0, 90.0 * dt)
        self.yaw += self.turn_rate * dt
        if keys["lift_up"]:
            self.lift = min(1.5, self.lift + 0.3 * min(dt, 1 / 60))
        if keys["lift_down"]:
            self.lift = max(-0.1, self.lift - 0.3 * min(dt, 1 / 60))


def rehearse(demo, x, y, yaw_deg, lookahead_m, dt=1 / 60, t_max=120.0, blocked=None):
    """이 자세에서 `demo`를 머릿속으로 주행 (시뮬레이션 없음). 사람의 포크가 파렛트에 닿은 지점에서 포크 끝의
    옆 방향 오차 [m] 반환, 통과하지 못하면 None.
    dt = Isaac Sim timeline 스텝 (초당 60). `blocked(old_pose, new_pose)` (pose = x, y, yaw_deg, lift)는
    Lesson 1 충돌 검사: 랙이나 기둥에 부딪히는 경로는 거부(None)."""
    k_ins = demo.get("insert_segment")
    if k_ins is None:
        return None
    fk, fol, t = KinematicForklift(x, y, yaw_deg), PathFollower(demo, lookahead_m), 0.0
    miss = None
    n, last = 0, (x, y, yaw_deg, 0.0)
    while t < t_max and not fol.done and not fol.failed:
        fk.step(fol.step(fk.x, fk.y, fk.yaw, fk.speed, fk.turn_rate, fk.lift, dt), dt)
        t += dt
        n += 1
        # 충돌은 파렛트까지 가는 동안만 검사: 목표 선반 근처는 여유가 몇 cm로, 실제 주행과 리허설의 오차보다 작아
        # 검사 결과를 믿을 수 없음
        if blocked is not None and n % COLLISION_EVERY == 0 and fol.k <= k_ins:
            pose = (fk.x, fk.y, fk.yaw, fk.lift)
            if blocked(last, pose):
                return None
            last = pose
        if miss is None and fol.k == k_ins:
            cx, cy, cyaw = demo["contact"]
            yr = math.radians(cyaw)
            ahead = math.sin(yr) * (fk.x - cx) - math.cos(yr) * (fk.y - cy)
            if ahead >= 0.0 or fol.mode == "settle":  # 사람의 포크가 파렛트에 닿은 지점에 도달
                side = abs(math.cos(yr) * (fk.x - cx) + math.sin(yr) * (fk.y - cy))
                miss = side + FORK_REACH_M * abs(math.sin(_wrap(math.radians(fk.yaw - cyaw))))
        if miss is not None and fol.k > k_ins:
            return miss  # 파렛트를 집은 뒤로는 검사하지 않음: 나머지는 사람이 운전한 경로 그대로
    return miss if fol.done else None


def warp(demo, x, y, blend_share):
    """시연 경로를 (x, y)에서 출발하도록 변형. 모든 점을 출발점 차이만큼 옮기되 경로를 따라 cosine으로 서서히
    줄여서, 사람이 운전한 모양을 유지하고 포크가 파렛트에 닿기 전 마지막 WARP_KEEP_M은 녹화 그대로 둔다.
    이미 여기서 출발하거나 이동량을 줄일 거리가 부족하면 None. (구간의 yaw는 그대로; 추종은 x, y만 사용)"""
    k_ins, contact = demo.get("insert_segment"), demo.get("contact")
    if k_ins is None or contact is None:
        return None
    dx, dy = x - demo["start"][0], y - demo["start"][1]
    if math.hypot(dx, dy) < WARP_MIN_SHIFT_M:
        return None
    s_contact = 0.0  # 포크가 파렛트에 닿은 지점까지의 경로상 거리
    for k, seg in enumerate(demo["segments"]):
        sx, sy = np.asarray(seg["x"]), np.asarray(seg["y"])
        ds = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(sx), np.diff(sy)))])
        if k == k_ins:
            s_contact += float(ds[int(np.argmin(np.hypot(sx - contact[0], sy - contact[1])))])
            break
        s_contact += float(ds[-1])
    blend = (s_contact - WARP_KEEP_M) * blend_share
    if blend < 1.0:
        return None
    segs, s = [], 0.0
    for seg in demo["segments"]:
        sx, sy = np.asarray(seg["x"]), np.asarray(seg["y"])
        ds = s + np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(sx), np.diff(sy)))])
        w = np.where(ds < blend, 0.5 * (1.0 + np.cos(np.pi * ds / blend)), 0.0)
        segs.append({**seg, "x": [round(float(v), 4) for v in sx + w * dx],
                     "y": [round(float(v), 4) for v in sy + w * dy]})
        s = float(ds[-1])
    return {**demo, "name": f"{demo['name']} (변형)", "start": [x, y, demo["start"][2]], "segments": segs,
            "warped": True}


def plan(paths, x, y, yaw_deg, blocked=None, pallet=None, nearest=None, lookaheads=LOOKAHEADS_M):
    """이 자세에서 모든 시연을 (조향 설정 몇 가지로) 리허설해, 포크가 파렛트 구멍에 가장 정확히 들어가고
    아무것에도 부딪히지 않는 경로를 고른다. 충돌 검사(`blocked` = Lesson 1 충돌 검사, `pallet` (x, y, yaw_deg) =
    서 있는 파렛트)는 느리므로 정확한 순서대로 확인한다. 사람 경로가 모두 실패할 때만 출발점이 가까운 시연을
    현재 위치에서 출발하도록 변형(warp)해 같은 방식으로 시도한다. 변형은 파렛트 직전에 끝나므로 변형 경로는
    포크도 곧게 들어가야 한다 (hits_pallet(straight=True)).

    `nearest` / `lookaheads`: 출발점이 가까운 시연 / 더 적은 조향 설정으로 좁혀 빠르게 확인 (reposition 참고).

    (demo, lookahead_m, miss_m, tried, rejected) 반환. 통과한 경로가 없으면 demo는 None.
    """
    usable = [d for d in paths["demos"] if d.get("usable", True)]
    if nearest is not None:
        usable = sorted(usable, key=lambda d: math.hypot(d["start"][0] - x, d["start"][1] - y))[:nearest]
    tried, rejected = [], 0

    def first_clear(candidates, straight):
        nonlocal rejected
        def check(old, new):
            return ((blocked is not None and blocked(old, new))
                    or (pallet is not None and hits_pallet(new[0], new[1], new[2], pallet, straight=straight)))

        rehearsed = [(rehearse(demo, x, y, yaw_deg, la), demo, la) for demo in candidates for la in lookaheads]
        tried.extend(rehearsed)
        ranked = sorted((c for c in rehearsed if c[0] is not None and c[0] <= MAX_PLAN_MISS_M), key=lambda c: c[0])
        for miss, demo, la in ranked:
            if (blocked is None and pallet is None) or rehearse(demo, x, y, yaw_deg, la, blocked=check) is not None:
                return demo, la, miss
            rejected += 1
        return None

    found = first_clear(usable, straight=False)
    if found is None:
        closest = sorted(usable, key=lambda d: math.hypot(d["start"][0] - x, d["start"][1] - y))[:WARP_NEAREST]
        bent = [b for d in closest for share in WARP_BLENDS if (b := warp(d, x, y, share)) is not None]
        found = first_clear(bent, straight=True)
    if found is None:
        return None, None, None, tried, rejected
    return (*found, tried, rejected)


# ---------------------------------------------------------------- 자리 고쳐 잡기

# '자리 고쳐 잡기': 잠깐 생각하고, 조금 움직이고, 다시 생각한다. 현재 위치에서 빠르게 확인(가까운 시연
# QUICK_DEMOS개, 조향 설정 1개) -> 경로가 없으면 시연 출발점(위치와 방향, start_gap)에 가장 가까워지는 짧은 이동
# (APPROACH_MOVES)을 한 뒤 다시 확인. 최대 APPROACH_MAX_STEPS회.
# 이동 = 스텝 목록 (방향 +1 / -1, 누르는 조향 키 또는 None, m).
APPROACH_MOVES = (
    [(+1, None, 0.7)], [(-1, None, 0.7)], [(+1, None, 2.0)], [(+1, None, 3.0)], [(+1, None, 5.0)],
    [(+1, "left", 1.0)], [(+1, "right", 1.0)], [(-1, "left", 1.0)], [(-1, "right", 1.0)],
    [(+1, "left", 2.0)], [(+1, "right", 2.0)], [(-1, "left", 2.0)], [(-1, "right", 2.0)],
    [(+1, "left", 3.6)], [(+1, "right", 3.6)], [(-1, "left", 3.6)], [(-1, "right", 3.6)],  # ~90도 회전
)
QUICK_DEMOS = 4
QUICK_LOOKAHEADS_M = (0.7,)
APPROACH_MAX_STEPS = 16
APPROACH_MIN_GAIN_M = 0.05  # 이동 한 번에 최소 이만큼 가까워져야 함 (start_gap)
GAP_M_PER_RAD = 1.0  # start_gap: 방향 차이 1 rad를 이만큼의 m로 환산 -- 가까울 때 기준:
GAP_NEAR_M = 1.5  # 멀수록 점점 덜 반영 (멀 때는 자유롭게 방향 전환, 도착해서 정렬)
REVERSE_COST = 0.3  # 후진 1 m당 이만큼의 gap 페널티 (가능하면 전진)
OPEN_M = 0.8  # 이동 끝이 랙/기둥과 이보다 가까우면 NEAR_WALL_COST만큼 불리: 최단 경로라고 벽으로
NEAR_WALL_COST = 1.0  # 향하면 그 뒤 회전할 공간이 없음
# 자리 고쳐 잡기 이동은 랙/기둥과 이 거리를 유지 (실제 주행은 리허설보다 조금 더 미끄러짐)
# -- 이미 그만큼 가까이 서 있으면 실제 접촉만 검사 (아니면 빠져나올 수 없음)
REPOSITION_MARGIN_M = 0.25
REPOSITION_SPEED_MPS = 0.5


class MoveDriver:
    """Lesson 1 키로 짧은 이동 하나를 주행: 스텝마다 방향 키(와 조향 키)를 거리만큼 누른 뒤 떼고 관성으로 정지."""

    def __init__(self, steps):
        self.steps = list(steps)
        self.k = 0
        self.moved = 0.0
        self.last = None
        self.coasting = False
        self.done = False

    def step(self, x, y, speed):
        keys = {k: False for k in KEYS}
        if self.done:
            return keys
        if self.last is not None:
            self.moved += math.hypot(x - self.last[0], y - self.last[1])
        self.last = (x, y)
        d, turn, dist = self.steps[self.k]
        if not self.coasting and self.moved < dist:
            if abs(speed) < REPOSITION_SPEED_MPS:
                keys["forward" if d > 0 else "backward"] = True
            if turn:
                keys[turn] = True
            return keys
        self.coasting = True
        if abs(speed) > 0.02:
            return keys
        self.k += 1
        self.moved = 0.0
        self.coasting = False
        self.done = self.k >= len(self.steps)
        return keys


def rehearse_move(steps, x, y, yaw_deg, blocked=None, pallet=None, dt=1 / 60, t_max=20.0, collides=None, margin=0.0):
    """짧은 이동을 머릿속으로 주행: 끝 자세 (x, y, yaw_deg), 무언가에 부딪히면 None.
    `collides(x, y, yaw_deg, lift)` (Lesson 1 자세 검사)는 해당 자세와, `margin`이 있으면 그만큼 앞/뒤/좌/우로
    옮긴 자세도 검사한다 -- 즉 모든 것과 그 거리를 유지."""
    fk, mv = KinematicForklift(x, y, yaw_deg), MoveDriver(steps)
    t, n, last = 0.0, 0, (x, y, yaw_deg, 0.0)
    shifts = [(0.0, 0.0)] + ([(margin, 0.0), (-margin, 0.0), (0.0, margin), (0.0, -margin)] if margin > 0 else [])
    while not mv.done and t < t_max:
        fk.step(mv.step(fk.x, fk.y, fk.speed), dt)
        t += dt
        n += 1
        if n % COLLISION_EVERY == 0:
            pose = (fk.x, fk.y, fk.yaw, fk.lift)
            if (blocked is not None and blocked(last, pose)) or (pallet is not None and hits_pallet(fk.x, fk.y, fk.yaw, pallet)):
                return None
            if collides is not None:
                yr = math.radians(fk.yaw)
                fx, fy, lx, ly = math.sin(yr), -math.cos(yr), math.cos(yr), math.sin(yr)
                if any(collides(fk.x + fx * a + lx * s, fk.y + fy * a + ly * s, fk.yaw, fk.lift) for a, s in shifts):
                    return None
            last = pose
    return (fk.x, fk.y, fk.yaw) if mv.done else None


def start_gap(paths, x, y, yaw_deg):
    """이 자세와 가장 가까운 시연 출발점 사이의 거리 [m]. 방향 차이는 1 rad당 GAP_M_PER_RAD m로 환산."""
    gaps = []
    for d in paths["demos"]:
        if not d.get("usable", True):
            continue
        dist = math.hypot(d["start"][0] - x, d["start"][1] - y)
        weight = GAP_M_PER_RAD * min(1.0, GAP_NEAR_M / max(dist, 1e-3))
        gaps.append(dist + weight * abs(_wrap(math.radians(yaw_deg - d["start"][2]))))
    return min(gaps) if gaps else float("inf")


def approach_move(paths, x, y, yaw_deg, blocked=None, pallet=None, collides=None):
    """시연 출발점에 가장 가까워지는 짧은 이동 (APPROACH_MOVES 중 리허설에서 충돌 없는 것): (steps, 끝 자세).
    APPROACH_MIN_GAIN_M 이상 가까워지는 이동이 없으면 None. `collides` (Lesson 1 자세 검사)가 있으면
    모든 것과 REPOSITION_MARGIN_M을 유지 (rehearse_move 참고)."""
    now = start_gap(paths, x, y, yaw_deg)
    margin = REPOSITION_MARGIN_M
    if collides is not None:
        yr = math.radians(yaw_deg)
        fx, fy, lx, ly = math.sin(yr), -math.cos(yr), math.cos(yr), math.sin(yr)
        if any(collides(x + fx * a + lx * s, y + fy * a + ly * s, yaw_deg, 0.0)
               for a, s in ((margin, 0.0), (-margin, 0.0), (0.0, margin), (0.0, -margin))):
            margin = 0.0  # 이미 그만큼 가까움: 빠져나가는 동안 닿지만 않으면 됨
    best = None
    for steps in APPROACH_MOVES:
        end = rehearse_move(steps, x, y, yaw_deg, blocked=None if collides is not None else blocked, pallet=pallet,
                            collides=collides, margin=margin)
        if end is None:
            continue
        gap = start_gap(paths, *end)
        score = gap + REVERSE_COST * sum(dist for d, _turn, dist in steps if d < 0)
        if collides is not None:
            ex, ey, eyaw = end
            yr = math.radians(eyaw)
            fx, fy, lx, ly = math.sin(yr), -math.cos(yr), math.cos(yr), math.sin(yr)
            if any(collides(ex + fx * a + lx * s, ey + fy * a + ly * s, eyaw, 0.0)
                   for a, s in ((OPEN_M, 0.0), (-OPEN_M, 0.0), (0.0, OPEN_M), (0.0, -OPEN_M))):
                score += NEAR_WALL_COST
        if gap < now - APPROACH_MIN_GAIN_M and (best is None or score < best[0]):
            best = (score, steps, end)
    return None if best is None else (best[1], best[2])


def describe_move(steps):
    """패널 표시용 한국어 문구, 예: '후진 1.5 m (왼쪽으로 꺾어서) → 후진 1.5 m (오른쪽으로 꺾어서)'."""
    turn = {None: "", "left": " (왼쪽으로 꺾어서)", "right": " (오른쪽으로 꺾어서)"}
    return " → ".join(f"{'전진' if d > 0 else '후진'} {dist:.1f} m{turn[t]}" for d, t, dist in steps)


def save(paths, path):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(paths, f)


def load(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)
