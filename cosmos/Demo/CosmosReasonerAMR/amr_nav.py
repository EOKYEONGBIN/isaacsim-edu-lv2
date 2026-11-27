# AMR 경로 계산 (전역 경로: 통로 그래프 + A*)
#
#   import lidar_drive
#   nav = lidar_drive.LidarNavigator()      # 이 클래스를 물려받아 실제 주행(바퀴 구동 + 라이다)을 한다
#   path = nav.plan("START", "GOAL")        # 막힌 통로 간선은 제외하고 A* 로 최단 경로
#
# - 경로: 통로 그래프에서 A* 로 최단 경로 (막힌 통로 간선은 제외)
# - 이동은 lidar_drive.LidarNavigator 가 한다. (예전에는 여기에 물리 없이 위치를 직접 옮기는 단순 이동도 있었는데,
#   쓰지 않아 뺐다)
# - self.blocked / self.judged: 통로 입구에서 판단한 결과를 기억해 같은 통로를 다시 묻지 않는다.

import heapq
import math

import amr_scene


class Navigator:
    def __init__(self):
        self.nodes, self.edges = amr_scene.graph()
        self.blocked = set()          # 막힘으로 판단된 통로 이름
        self.judged = {}              # 통로 → 판단 결과 (이번 주행에서 이미 본 통로는 다시 묻지 않음)
        self.node = "START"
        self.x, self.y = self.nodes["START"]
        self.yaw = 90.0
        self.log = []                 # (사건, 내용)
        self.stop_requested = False
        amr_scene.set_robot_pose(self.x, self.y, self.yaw)

    # ---------- 경로 ----------
    def _neighbors(self, n):
        for a, b, aisle in self.edges:
            if aisle and aisle in self.blocked:
                continue
            if a == n:
                yield b, aisle
            elif b == n:
                yield a, aisle

    def _dist(self, a, b):
        (x1, y1), (x2, y2) = self.nodes[a], self.nodes[b]
        return math.hypot(x2 - x1, y2 - y1)

    def plan(self, start, goal):
        """A* 최단 경로. 경로가 없으면 None.

        A* = 다익스트라 + '목표까지 남은 거리 추정(h)'. 큐에서 꺼내는 순서를 (지금까지 온 거리 g + h) 로 정해
        목표 쪽 지점부터 확정해 간다. h 는 목표까지의 직선거리 — 실제 남은 거리보다 길 수 없으므로
        다익스트라와 똑같은 최단 경로를 찾으면서 덜 탐색한다. (예전에는 다익스트라였고, 결과 경로는 같다)
        self.expanded 에 이번 계산에서 확정한 지점 수를 남긴다.

        비용 = (총 거리, 첫 통로 입구까지의 거리). 총 거리가 같으면 '가장 가까운 통로부터' 들어가는 경로를 고른다.
        (이 창고는 통로 길이가 모두 같고 복도가 나란해서 어느 통로로 가든 GOAL 까지 거리가 같다.
         예전에는 동점일 때 내부 순서로 골라 A 가 막히면 바로 옆 B 가 아니라 D 로 갔다.
         통로 입구에서 판단하는 방식이라 가까운 통로부터 확인하는 편이 덜 돌아간다.)
        상태 = (노드, 통로에 이미 들어갔는지) — 들어간 뒤로는 두 번째 비용이 늘지 않는다."""
        def key(c):
            return (round(c[0], 3), round(c[1], 3))      # 부동소수 오차로 동점이 깨지지 않게

        gx, gy = self.nodes[goal]

        def h(n):                                         # 목표까지 직선거리 (휴리스틱)
            x, y = self.nodes[n]
            return math.hypot(gx - x, gy - y)

        s0 = (start, False)
        best = {s0: (0.0, 0.0)}
        prev = {}
        q = [(key((h(start), 0.0)), 0.0, 0.0, start, False)]
        self.expanded = 0
        while q:
            _, d, pre, n, entered = heapq.heappop(q)
            st = (n, entered)
            if key((d, pre)) > key(best.get(st, (1e9, 1e9))):
                continue                                   # 더 좋은 값으로 이미 확정된 지점
            self.expanded += 1
            if n == goal:
                path = [n]
                while st in prev:
                    st = prev[st]
                    path.append(st[0])
                return path[::-1]
            for m, aisle in self._neighbors(n):
                step = self._dist(n, m)
                nd = d + step
                npre = pre if entered else pre + (0.0 if aisle else step)
                ns = (m, entered or bool(aisle))
                if key((nd, npre)) < key(best.get(ns, (1e9, 1e9))):
                    best[ns], prev[ns] = (nd, npre), st
                    heapq.heappush(q, (key((nd + h(m), npre)), nd, npre, m, ns[1]))   # 우선순위 = g + h
        return None

    def aisle_of(self, a, b):
        for e1, e2, aisle in self.edges:
            if {e1, e2} == {a, b}:
                return aisle
        return None
