"""Learning driving paths from demonstrations, and following them with Lesson 1's keys.

Pure Python + numpy (no Kit), shared by tools/learn_paths.py and the AI driver.

What is learned from each successful demonstration: the route the forklift took, split into
forward / reverse segments (tiny back-and-forth corrections removed), and the fork height
along it -- including the stops where the person lifted or lowered the pallet. Before it moves,
the AI rehearses every learned route from where the forklift is now (with the forklift's own
motion rules, in a fraction of a second each), keeps the one whose forks would meet the pallet
pocket most accurately, and follows it with a pure-pursuit controller that presses the same six
keys a person would.
"""

from __future__ import annotations

import json
import math

import numpy as np

KEYS = ["forward", "backward", "left", "right", "lift_up", "lift_down"]

# Lesson 1 forklift (forklift_omnigraph_nodes): turn rate = 30 deg/s * steer/30 * speed/1.2
MAX_STEER_DEG = 30.0
CURV_PER_STEER_DEG = math.radians(30.0) / 30.0 / 1.2  # [1/m per deg]: max 0.436 1/m -> 2.3 m radius
MAX_CURV = CURV_PER_STEER_DEG * MAX_STEER_DEG

MOVING_MPS = 0.03  # slower than this counts as standing still
MIN_SEGMENT_M = 0.30  # shorter back-and-forth corrections are dropped
LIFT_STOP_M = 0.10  # forks moved this much while standing still = a separate step (e.g. lifting the pallet)
RESAMPLE_M = 0.05
MAX_APPROACH_DEG = 35.0  # forks entering the pallet more crooked than this shove it around instead of sliding in


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


# ---------------------------------------------------------------- learning


def demo_segments(rows):
    """Forward/reverse segments of one demonstration (rows = steps.csv dicts).

    Returns [{"dir": +1/-1, "x": [...], "y": [...], "yaw": [...] (deg), "lift": [...],
              "lift_after": fork height to reach before the next segment starts}].
    """
    t = np.array([float(r["t"]) for r in rows])
    x = np.array([float(r["x"]) for r in rows])
    y = np.array([float(r["y"]) for r in rows])
    yaw = np.array([float(r["yaw_deg"]) for r in rows])
    v = np.array([float(r["speed_mps"]) for r in rows])
    lift = np.array([float(r["lift_m"]) for r in rows])
    sign = np.where(np.abs(v) < MOVING_MPS, 0, np.sign(v)).astype(int)

    # raw runs of constant non-zero direction; a stop where the forks went up or down also ends a run
    # ("drive in, stop, lift, drive on" is two steps even though both go forward)
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
        while k > i and sign[k] == 0:  # trailing standstill belongs to the stop, not the run
            k -= 1
        runs.append([sign[i], i, k, hard])
        hard = lifted
        i = j + 1

    def length(a, b):
        return float(np.sum(np.hypot(np.diff(x[a:b + 1]), np.diff(y[a:b + 1]))))

    # drop tiny corrections (a lift stop just before one still counts), then merge neighbours that now
    # drive the same way -- unless the forks went up or down in between
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
        # fork height wanted before moving on: what the person had when they started the next segment
        # (or at the very end)
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
    """Where the forks met the pallet: the first moment the pallet moved (people often push it a little
    before lifting). Returns (segment index, [x, y, yaw_deg] of the forklift then) or (None, None)."""
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
        return None, None  # backed into it?
    return started[-1], pose


def pallet_ahead(x, y, yaw_deg, px, py):
    """Pallet centre in the forklift's frame: (metres ahead, metres to the left)."""
    yr = math.radians(yaw_deg)
    fx, fy = math.sin(yr), -math.cos(yr)
    dx, dy = px - x, py - y
    return dx * fx + dy * fy, -dx * fy + dy * fx


# Outlines for the rehearsal's pallet check (forklift frame: from / to metres ahead of its origin, half width).
# Lesson 1's own collision test lets the forklift push the pallet, so a route that drives into it would look
# fine there: the body must never touch the standing pallet, and the forks (both tines as one outline) must
# not swing across it -- reaching it more than FORK_SWEEP_DEG askew (the people's own drives reach it at up to
# ~27 deg). Routes bent to a new start are held to more: forks straight and centred (FORK_SIDE_TOL_M,
# FORK_ANGLE_TOL_DEG), since their bend ends shortly before the pallet.
BODY_OUTLINE = (-1.26, 1.30, 0.58)
FORKS_OUTLINE = (1.30, 2.45, 0.45)
PALLET_HALF = (0.50, 0.61)  # along the pallet's fork-entry axis, across it
FORK_SWEEP_DEG = 30.0
FORK_SIDE_TOL_M = 0.10
FORK_ANGLE_TOL_DEG = 6.0


def _outline(x, y, yaw_deg, ahead0, ahead1, half_width):
    yr = math.radians(yaw_deg)
    fx, fy, lx, ly = math.sin(yr), -math.cos(yr), math.cos(yr), math.sin(yr)
    return np.array([(x + fx * a + lx * w, y + fy * a + ly * w) for a in (ahead0, ahead1) for w in (-half_width, half_width)])


def _overlap(c1, c2):
    """Do two rectangles (4 corners each) overlap? Separating-axis test."""
    for c in (c1, c2):
        for e in (c[1] - c[0], c[2] - c[0]):
            axis = np.array([-e[1], e[0]])
            p1, p2 = c1 @ axis, c2 @ axis
            if p1.max() < p2.min() or p2.max() < p1.min():
                return False
    return True


def hits_pallet(x, y, yaw_deg, pallet, straight=False):
    """Would the forklift at this pose bump the standing pallet (x, y, yaw_deg)? The body touching it, or the
    forks reaching it swung more than FORK_SWEEP_DEG -- with `straight`, off-centre or askew at all."""
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
    """How far ahead of the forklift the pallet was when the person started lifting it, i.e. with the
    forks all the way in (the pallet rests against the fork carriage)."""
    z0 = float(rows[0]["pallet_z"])
    r = next((r for r in rows if float(r["pallet_z"]) > z0 + 0.05), None)
    if r is None:
        return None
    return pallet_ahead(float(r["x"]), float(r["y"]), float(r["yaw_deg"]), float(r["pallet_x"]), float(r["pallet_y"]))[0]


def learn(demos):
    """demos: [(name, rows, meta)] -> dict saved as paths.json."""
    out, seats = [], []
    for name, rows, meta in demos:
        segs = demo_segments(rows)
        if not segs:
            continue
        k, contact = insertion(segs, rows)
        approach = None
        if contact is not None:
            # pockets run along the pallet's local y; Lesson 1's forklift at yaw 0 drives along -y
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
    # forks fully in: the typical distance (median, so one crooked pickup does not move it)
    seat = float(np.median(seats)) if seats else None
    return {"version": 1, "seat_ahead_m": None if seat is None else round(seat, 3), "demos": out}


def summary(paths):
    """Numbers to show trainees: what the AI learned."""
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


# ---------------------------------------------------------------- following


class PathFollower:
    """Turns a learned route into key presses (call step() every frame)."""

    LOOKAHEAD_M = 0.7
    CRUISE_MPS = 0.6
    STOP_DECEL = 0.5  # plan to slow down this gently before a segment end [m/s^2]
    END_TOL_M = 0.06
    RATE_BAND_DEG = 0.5  # deg/s
    LIFT_TOL_M = 0.02
    STUCK_S = 8.0

    SEAT_MARGIN_M = 0.03
    CREEP_MAX_M = 1.0  # past the demo's end, drive on at most this far to get the forks all the way in
    CREEP_PUSH_M = 0.3  # ...but stop if the pallet came no closer over this much driving (just shoving it)
    CREEP_REMAINING_M = 0.03  # creeping speed: as if this close to a stop (~0.25 m/s)
    # Lift only once the forks are at least this share of the pallet's depth in: less leaves it hanging on the
    # fork tips. Short of that, back straight out RETRY_BACK_M and push in once more, then give up.
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
        self.k = 0  # segment
        self.i = 0  # progress index inside the segment
        self.mode = "drive"
        self.done = False
        self.failed = None
        self._best = None
        self._since_progress = 0.0
        self._turning = 0  # +1 holding the CCW key, -1 the CW key, 0 none
        self._creep = None  # [x0, y0, closest pallet distance, distance driven when it was closest]
        self._retries = 0
        self._back_from = None  # where backing out for another try started

    def step(self, x, y, yaw_deg, speed, turn_rate_deg, lift, dt, pallet_ahead_m=None):
        """Keys to hold this frame: dict key -> bool. `pallet_ahead_m`: how far ahead the pallet is
        (None if unknown); on the way into the pallet the AI stops as soon as the forks are all the way in."""
        keys = {k: False for k in KEYS}
        if self.done or self.failed:
            return keys
        seg = self.segs[self.k]
        sx, sy, sl = np.asarray(seg["x"]), np.asarray(seg["y"]), np.asarray(seg["lift"])
        n = len(sx)

        if self.mode == "back_off":  # forks not far enough in: straight back out, then in again
            if math.hypot(x - self._back_from[0], y - self._back_from[1]) < self.RETRY_BACK_M:
                if speed * seg["dir"] > -0.25:
                    keys["backward" if seg["dir"] > 0 else "forward"] = True
                return keys
            self.mode = "drive"

        if self.mode == "drive":
            # progress: nearest point ahead of where we were (never go back along the route)
            lo, hi = self.i, min(n, self.i + 40)
            d2 = (sx[lo:hi] - x) ** 2 + (sy[lo:hi] - y) ** 2
            self.i = lo + int(np.argmin(d2))
            remaining = float(np.sum(np.hypot(np.diff(sx[self.i:]), np.diff(sy[self.i:])))) if self.i < n - 1 else 0.0
            end_dist = math.hypot(sx[-1] - x, sy[-1] - y)
            # passed the end? (projection of the last step)
            if n >= 2:
                ex, ey = sx[-1] - sx[-2], sy[-1] - sy[-2]
                past = (x - sx[-1]) * ex + (y - sy[-1]) * ey > 0
            else:
                past = True
            inserting = (self.k == self.demo.get("insert_segment") and pallet_ahead_m is not None
                         and self.seat_ahead_m is not None)
            seated = inserting and pallet_ahead_m <= self.seat_ahead_m + self.SEAT_MARGIN_M
            at_end = (remaining < self.END_TOL_M and end_dist < 0.25) or (self.i >= n - 2 and past)
            # The pallet may have slid further than in the demo: forks not all the way in at the demo's
            # end point, so creep on along the same line (lifting now would leave it on the fork tips).
            creeping = at_end and inserting and not seated and self._keep_creeping(x, y, pallet_ahead_m)
            if at_end and not creeping and inserting and not seated and not self._deep_enough(pallet_ahead_m):
                # pushed in as far as it goes and the forks are still not LIFT_MIN_INSERT in: do not lift
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

        # settle: stop, set the fork height for what comes next, then start the next segment
        if abs(speed) > 0.02:
            if seg["dir"] > 0 and speed > 0.15:
                keys["backward"] = True  # brake
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
        # lookahead point
        j, acc = self.i, 0.0
        while j < len(sx) - 1 and acc < self.LOOKAHEAD_M:
            acc += math.hypot(sx[j + 1] - sx[j], sy[j + 1] - sy[j])
            j += 1
        tx, ty = sx[j], sy[j]
        if acc < self.LOOKAHEAD_M and len(sx) >= 2:  # extend past the end along the last direction
            ex, ey = sx[-1] - sx[-2], sy[-1] - sy[-2]
            norm = math.hypot(ex, ey) or 1.0
            tx += ex / norm * (self.LOOKAHEAD_M - acc)
            ty += ey / norm * (self.LOOKAHEAD_M - acc)
        yaw = math.radians(yaw_deg)
        fwd = (math.sin(yaw), -math.cos(yaw))  # Lesson 1 drives toward (sin yaw, -cos yaw)
        mx, my = (fwd[0] * d, fwd[1] * d)  # direction of motion
        lx, ly = -my, mx  # left of the motion direction
        dx, dy = tx - x, ty - y
        ahead, left = dx * mx + dy * my, dx * lx + dy * ly
        ld2 = max(ahead * ahead + left * left, 1e-6)
        curv = 2.0 * left / ld2  # pure pursuit, CCW positive along the motion
        curv = max(-MAX_CURV, min(MAX_CURV, curv))
        # Lesson 1 only turns while Left/Right is held (released -> wheels recentre, turning stops), and
        # the heading turns CCW at curvature * speed along the motion for either direction of travel.
        # So steer by yaw rate: hold the key that pushes the current turn rate toward the wanted one.
        # Releasing the key recentres the wheels at once, so keep holding it until the turn rate is
        # clearly past the wanted one (hysteresis), and never let go while the route needs full lock.
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
        sgn = d if abs(speed) < 0.05 else (1 if speed > 0 else -1)  # Left turns CCW when moving forward
        if self._turning:
            keys["left" if self._turning * sgn > 0 else "right"] = True

        # speed: cruise, slow down to stop at the segment end, slower while turning hard
        v_target = min(self.CRUISE_MPS, math.sqrt(2 * self.STOP_DECEL * max(remaining, 0.0)) + 0.08)
        v_target *= 1.0 - 0.4 * abs(curv) / MAX_CURV
        if v_along < v_target - 0.03:
            keys["forward" if d > 0 else "backward"] = True
        elif v_along > v_target + 0.25:
            keys["backward" if d > 0 else "forward"] = True  # brake

    def _deep_enough(self, pallet_ahead_m):
        """Forks at least LIFT_MIN_INSERT of the pallet's depth in (seat_ahead_m = all the way in)?"""
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


# ---------------------------------------------------------------- rehearsal

FORK_REACH_M = 1.0  # how far ahead of the insertion point the fork tips are, inside the pallet
LOOKAHEADS_M = (0.5, 0.7, 1.0)
COLLISION_EVERY = 3  # frames between collision checks (at most ~6 cm of travel)

# Routes bent to start where the forklift is (see warp): the demonstrations that started nearest, with the
# shift faded out over these shares of the way to the pallet; the last stretch before the forks met it stays
# exactly as the person drove it.
WARP_NEAREST = 6
WARP_BLENDS = (0.5, 1.0)
WARP_KEEP_M = 1.5
WARP_MIN_SHIFT_M = 0.05
# A route whose rehearsed fork tips would miss the pocket by more than this is not worth driving: the
# forks would run past or into the side of the pallet (the pockets leave about this much play)
MAX_PLAN_MISS_M = 0.10


class KinematicForklift:
    """Lesson 1's key-driven forklift motion (forklift_omnigraph_nodes), without the scene."""

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
    """Drive `demo` from this pose in the head (no simulation): sideways miss of the fork tips where
    the person's forks met the pallet [m], or None if the drive would not get through.
    dt = Isaac Sim's timeline step (60 per second). `blocked(old_pose, new_pose)` (poses = x, y, yaw_deg,
    lift), if given, is Lesson 1's own collision test: a route that would run into a rack or pillar
    is rejected (None)."""
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
        # collisions only matter on the way to the pallet: near the target shelf the clearances are a few cm,
        # less than how far the real drive drifts from the rehearsal, so the test there is unreliable
        if blocked is not None and n % COLLISION_EVERY == 0 and fol.k <= k_ins:
            pose = (fk.x, fk.y, fk.yaw, fk.lift)
            if blocked(last, pose):
                return None
            last = pose
        if miss is None and fol.k == k_ins:
            cx, cy, cyaw = demo["contact"]
            yr = math.radians(cyaw)
            ahead = math.sin(yr) * (fk.x - cx) - math.cos(yr) * (fk.y - cy)
            if ahead >= 0.0 or fol.mode == "settle":  # reached the point where the person's forks met the pallet
                side = abs(math.cos(yr) * (fk.x - cx) + math.sin(yr) * (fk.y - cy))
                miss = side + FORK_REACH_M * abs(math.sin(_wrap(math.radians(fk.yaw - cyaw))))
        if miss is not None and fol.k > k_ins:
            return miss  # past the pallet pickup nothing more is checked: the rest is the person's own route
    return miss if fol.done else None


def warp(demo, x, y, blend_share):
    """The demonstration's route bent to start at (x, y): every point moves by the start offset, fading
    out smoothly (cosine) along the way, so the route keeps the person's shape and the last WARP_KEEP_M
    before the forks met the pallet is exactly as recorded. None if it starts here already or there is
    no room to fade the shift out. (The segments' recorded yaw is left as is; following uses x, y only.)"""
    k_ins, contact = demo.get("insert_segment"), demo.get("contact")
    if k_ins is None or contact is None:
        return None
    dx, dy = x - demo["start"][0], y - demo["start"][1]
    if math.hypot(dx, dy) < WARP_MIN_SHIFT_M:
        return None
    s_contact = 0.0  # distance along the route to where the forks met the pallet
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
    """Rehearse every demonstration (with a few steering settings) from this pose and keep the one whose
    forks would meet the pallet pocket most accurately and that does not run into anything: `blocked`
    (Lesson 1's collision test) and, with `pallet` (x, y, yaw_deg), the standing pallet -- checked from the
    most accurate down, since those tests are slower. Only if none of the people's routes gets through are
    the nearest-starting ones bent to start right here (warp) and tried the same way; a bent route must
    also bring its forks in straight (hits_pallet(straight=True)), since the bend ends shortly before the pallet.

    `nearest` / `lookaheads` narrow it to the demonstrations that started nearest / fewer steering settings
    (a quicker look, see reposition).

    Returns (demo, lookahead_m, miss_m, tried, rejected) -- demo is None if nothing got through.
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


# ---------------------------------------------------------------- repositioning

# '자리 고쳐 잡기': think briefly, move a little, think again. From where the forklift stands, a quick look
# (QUICK_DEMOS nearest demonstrations, one steering setting); no route -> the short move (APPROACH_MOVES) that
# brings it closest to where the people's demonstrations started (position and heading, start_gap), then
# look again from there, up to APPROACH_MAX_STEPS times.
# A move is a list of steps (direction +1 / -1, steering key held or None, metres).
APPROACH_MOVES = (
    [(+1, None, 0.7)], [(-1, None, 0.7)], [(+1, None, 2.0)], [(+1, None, 3.0)], [(+1, None, 5.0)],
    [(+1, "left", 1.0)], [(+1, "right", 1.0)], [(-1, "left", 1.0)], [(-1, "right", 1.0)],
    [(+1, "left", 2.0)], [(+1, "right", 2.0)], [(-1, "left", 2.0)], [(-1, "right", 2.0)],
    [(+1, "left", 3.6)], [(+1, "right", 3.6)], [(-1, "left", 3.6)], [(-1, "right", 3.6)],  # ~quarter turns
)
QUICK_DEMOS = 4
QUICK_LOOKAHEADS_M = (0.7,)
APPROACH_MAX_STEPS = 16
APPROACH_MIN_GAIN_M = 0.05  # a move must bring the forklift at least this much closer (start_gap)
GAP_M_PER_RAD = 1.0  # start_gap: heading difference counted as this many metres per radian -- when near:
GAP_NEAR_M = 1.5  # farther away it counts less and less (turn toward the start freely, line up once there)
REVERSE_COST = 0.3  # a move's reversing counts as this many metres of gap per metre (drive forward if it works)
OPEN_M = 0.8  # a move ending closer than this to a rack or pillar counts NEAR_WALL_COST metres worse: heading
NEAR_WALL_COST = 1.0  # into a wall because it is the shortest way leaves no room to turn afterwards
# A repositioning move must keep this far from racks and pillars (the real drive slides a little further than the
# rehearsal) -- unless the forklift already stands that close, then only real contact counts (else no way out)
REPOSITION_MARGIN_M = 0.25
REPOSITION_SPEED_MPS = 0.5


class MoveDriver:
    """Drives one short move with Lesson 1's keys: each step holds its direction key (and steering key) until it
    has covered its distance, then lets go and coasts to a stop."""

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
    """Drive a short move in the head: the pose (x, y, yaw_deg) it ends at, or None if it runs into anything.
    `collides(x, y, yaw_deg, lift)` (Lesson 1's pose test) is asked for the pose and, with `margin`, for it
    shifted that far forward / back / left / right -- keeping the move that far from everything."""
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
    """How far this pose is from the nearest place a person's demonstration started: metres, with the heading
    difference counted GAP_M_PER_RAD metres per radian."""
    gaps = []
    for d in paths["demos"]:
        if not d.get("usable", True):
            continue
        dist = math.hypot(d["start"][0] - x, d["start"][1] - y)
        weight = GAP_M_PER_RAD * min(1.0, GAP_NEAR_M / max(dist, 1e-3))
        gaps.append(dist + weight * abs(_wrap(math.radians(yaw_deg - d["start"][2]))))
    return min(gaps) if gaps else float("inf")


def approach_move(paths, x, y, yaw_deg, blocked=None, pallet=None, collides=None):
    """The short move (APPROACH_MOVES, rehearsed, nothing hit) that brings the forklift closest to where the
    demonstrations started: (steps, end pose), or None if none gets it APPROACH_MIN_GAIN_M closer. With
    `collides` (Lesson 1's pose test) the move keeps REPOSITION_MARGIN_M from everything (see rehearse_move)."""
    now = start_gap(paths, x, y, yaw_deg)
    margin = REPOSITION_MARGIN_M
    if collides is not None:
        yr = math.radians(yaw_deg)
        fx, fy, lx, ly = math.sin(yr), -math.cos(yr), math.cos(yr), math.sin(yr)
        if any(collides(x + fx * a + lx * s, y + fy * a + ly * s, yaw_deg, 0.0)
               for a, s in ((margin, 0.0), (-margin, 0.0), (0.0, margin), (0.0, -margin))):
            margin = 0.0  # already that close: just do not touch anything on the way out
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
    """Korean for the panel, e.g. '후진 1.5 m (왼쪽으로 꺾어서) → 후진 1.5 m (오른쪽으로 꺾어서)'."""
    turn = {None: "", "left": " (왼쪽으로 꺾어서)", "right": " (오른쪽으로 꺾어서)"}
    return " → ".join(f"{'전진' if d > 0 else '후진'} {dist:.1f} m{turn[t]}" for d, t, dist in steps)


def save(paths, path):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(paths, f)


def load(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)
