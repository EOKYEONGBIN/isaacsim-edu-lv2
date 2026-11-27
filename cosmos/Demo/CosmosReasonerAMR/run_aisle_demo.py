# ① 통로 판단 데모 전체 흐름: AMR 이 목적지로 가다가 통로 입구마다 Cosmos 에게 묻고, 막혔으면 우회한다.
#
# Isaac Sim 안에서:
#   import run_aisle_demo, asyncio
#   asyncio.ensure_future(run_aisle_demo.run({"A": "boxes", "B": "spill"}, mode="lidar"))   # mode: lidar / lidar_only
# 진행 상황: builtins._aisle_demo["events"], 기록: logs/aisle_demo.jsonl, 판단 사진: 확장의 '데이터 저장' 을 켜면 datasets/aisle_drive_<시각>/

import asyncio
import builtins
import json
import os
import time

import aisle_judge
import amr_scene
import capture
import dynamic_events
import incidents

ROOT = os.path.dirname(os.path.abspath(__file__))
LOG = os.path.join(ROOT, "logs", "aisle_demo.jsonl")


def _bubble(fn):
    """Cosmos 말풍선(bubble.py) 호출. 말풍선이 안 떠도 주행은 계속된다."""
    try:
        import bubble
        fn(bubble)
    except Exception:
        pass


def _overlays_reset():
    """뷰포트 표시(말풍선 bubble, 경로 띠 path_ribbon)를 새로 만든다. 나중에 만든 화면이 위에 그려지므로
    띠를 먼저, 말풍선을 나중에 만들어 말풍선이 항상 띠 위에 오게 한다."""
    try:
        import bubble
        import path_ribbon
        bubble.stop()
        path_ribbon.stop()
        path_ribbon.ensure_view()
    except Exception:
        pass


def _route(fn):
    """지나온 경로·지점 이름 표시(route_map.py) 호출. 표시가 안 돼도 주행은 계속된다."""
    try:
        import route_map
        return fn(route_map)
    except Exception:
        return None


def _ribbon():
    """경로 띠(path_ribbon.py)를 지운다."""
    try:
        import path_ribbon
        path_ribbon.clear()
    except Exception:
        pass


MODES = ("lidar", "lidar_only")
# (예전 '순간 이동' 방식은 뺐다)
# lidar      = 라이다 주행 (물리로 바퀴 구동) + 통로 입구 Cosmos 판단
# lidar_only = 라이다 주행만 (Cosmos 끔) — '라이다만 있는 AMR' 비교용


ARRIVE_VIEW_DELAY_SEC = 3.0                          # 목적지에 도착한 뒤 위에서 보기로 바꾸기까지 기다리는 시간 (초, 실제 시간)


async def _wait_real(sec, nav=None):
    """실제 시간으로 sec 초 기다린다 (그동안 시뮬레이션은 계속 돈다). 정지를 눌렀으면 바로 끝낸다."""
    import omni.kit.app
    t0 = time.time()
    while time.time() - t0 < sec and not (nav is not None and nav.stop_requested):
        await omni.kit.app.get_app().next_update_async()


async def _stop_and_reset():
    """아이작심 왼쪽 툴바의 정지 버튼을 누른 것처럼 타임라인을 멈추고, AMR 을 출발점(START)으로 옮긴다."""
    import omni.kit.app
    import omni.timeline
    tl = omni.timeline.get_timeline_interface()
    if not tl.is_stopped():
        tl.stop()
        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()
    amr_scene.set_robot_pose(*amr_scene.START, 90.0)


async def run(aisle_states=None, goal="GOAL", rebuild=False, on_event=None, on_judgement=None, on_image=None,
              mode="lidar", dynamic=True, save_data=False, on_block=None):
    """aisle_states: {"A": "boxes", ...} (적지 않은 통로는 비움). mode: MODES 중 하나.
    dynamic: 주행 중 걸어 다니는 작업자 3명(dynamic_events.py)을 켠다 (익스텐션의 작업자 토글 값).
    save_data: 켜면 통로 입구에서 Cosmos 에 보낸 사진을 박스 라벨 이미지·Cosmos 응답과 함께 datasets/aisle_drive_<시각>/ 에 저장한다
               (data_gen.DriveRecorder). Cosmos 를 쓰는 모드(lidar)에서만 사진이 있다. 기본 꺼짐.
    on_event(kind, text) / on_judgement(row) / on_image(aisle, rgb) / on_block(aisle, 이유): UI(익스텐션)용 콜백, 없어도 된다.
    on_block 은 통로 안에서 라이다에 막혔을 때 Cosmos 가 말해 준 '앞의 장면' 을 알린다."""
    ui_event = on_event
    status = {"events": [], "judgements": [], "done": False, "ok": None, "error": None, "data_dir": None}
    builtins._aisle_demo = status
    loop = asyncio.get_event_loop()
    rec = seg = None                                # 데이터 저장용 (save_data 가 켜졌을 때만)
    try:
        _route(lambda r: r.ensure_view())           # 지점 이름·지나온 경로 화면은 가장 아래층 (띠·말풍선보다 먼저 만든다)
        _route(lambda r: r.start_trail())           # 지난 주행의 지나온 경로를 지우고 이번 주행의 경로를 기록하기 시작한다
        _overlays_reset()                           # 플레이를 누르자마자 지난 주행의 말풍선·경로 띠를 치운다 (준비하는 동안 옛 말풍선이 남아 보였다).
                                                    # 띠 화면을 먼저 만들어 두면 말풍선이 항상 띠 위에 그려진다
        if rebuild:
            capture.reset()
            amr_scene.build()
        if mode not in MODES:
            raise ValueError(f"mode 는 {MODES} 중 하나: {mode!r}")
        import lidar_drive
        import omni.kit.app
        lidar_drive.prepare()                       # 타임라인 정지 + 로봇을 START 로 (장애물 충돌체는 정지 상태에서 만든다)
        for _ in range(5):                          # stop 이 실제로 처리될 때까지
            await omni.kit.app.get_app().next_update_async()
        capture.reset()                             # Play 중에 만들어진 렌더 프로덕트가 남아 있으면 정리 (핸들로)
        capture.prepare(amr_scene.FRONT_CAM)        # 카메라 렌더 프로덕트도 Play 전에 만든다
        if save_data and mode != "lidar_only":
            # 박스 라벨용 어노테이터를 Play 전에 붙여 둔다. 주행 중에는 카메라 렌더 갱신이 꺼져 있다가 사진 찍을 때만 켜지므로
            # (capture.set_gating) 이 어노테이터도 평소에는 비용이 없다.
            import data_gen
            rec = data_gen.DriveRecorder()
            seg = data_gen._seg_annotator(capture._ensure(amr_scene.FRONT_CAM, capture.RESOLUTION))
            status["data_dir"] = rec.dir
            _route(lambda r: r.prepare_map())       # 도착했을 때 위에서 본 지도 사진을 찍을 렌더 프로덕트도 Play 전에 만들어 둔다
        incidents.clear_all()
        for a, s in (aisle_states or {}).items():
            incidents.set_aisle(a, s)
        # 새로 놓은 물체가 렌더러에 들어갈 때까지 몇 프레임 기다린다
        # (놓자마자 같은 프레임에 Play 하면 콘·표지판 등이 화면에 안 그려진 적이 있다)
        for _ in range(10):
            await omni.kit.app.get_app().next_update_async()
        truth = incidents.truth()
        status["eff_state"] = {}                    # 채점용: 통로별로 판단 순간의 정답 (작업자가 통로에 있었으면 'worker')
        amr_scene.move_goal_marker(goal)            # 목적지 표시를 이 목적지로 옮기고 빨강으로 (도착하면 초록)
        nav = lidar_drive.LidarNavigator()
        builtins._aisle_nav = nav
        # 라이다가 통로 안에서 막힘을 감지하면 말풍선에 바로 '라이다 막힘 + 안전상의 문제로 돌아갑니다' 를 띄운다.
        # (Cosmos 에게 이유를 물어 답이 오면 같은 말풍선에 이유가 더해진다. Cosmos 를 쓰지 않는 모드에서는 이 문구만 보인다)
        nav.on_lidar_block = lambda aisle: _bubble(
            lambda b: b.show(False, f"통로{amr_scene.aisle_no(aisle)} : 라이다 막힘", "안전상의 문제로 돌아갑니다"))

        def on_event(kind, text):
            if kind == "arrived":
                amr_scene.set_goal_state("arrived")  # 도착 → 목적지 표시가 초록으로 (위에서 보기로 바꾸는 것은 3초 뒤: drive_to 가 끝난 다음)
            status["events"].append((time.strftime("%H:%M:%S"), kind, text))
            if kind in ("arrived", "stuck", "stopped"):
                _ribbon()                               # 도착·정지하면 경로 띠를 지운다
            if ui_event:
                ui_event(kind, text)

        async def on_checkpoint(aisle, lidar_blocked=False):
            """통로 앞 판단. lidar_blocked=True 는 '입구에서는 통과였는데 안에서 라이다에 막힘': 로봇은 라이다를 따라 돌아가고,
            Cosmos 에게는 지금 보이는 모습의 '이유' 만 묻는다 (채점·기록에는 넣지 않는다)."""
            no = amr_scene.aisle_no(aisle)
            rgb = await capture.grab_async(amr_scene.FRONT_CAM, steps=6)
            if rgb is None:
                on_event("error", f"통로{amr_scene.aisle_no(aisle)}: 카메라 이미지를 가져오지 못해 안전하게 '막힘' 으로 처리")
                return False, None
            if lidar_blocked:
                eff = status["eff_state"].get(aisle) or truth[aisle]        # 입구에서 정한 값을 그대로 둔다 (채점은 입구 판단 한 번)
            else:
                eff = dynamic_events.effective_state(aisle, truth[aisle])      # 메인 스레드에서 (USD 를 읽음)
                status["eff_state"][aisle] = eff
            boxes = None
            if rec is not None and not lidar_blocked:                                           # 사진을 찍은 바로 지금 같은 프레임의 박스 (판단을 기다리는 동안 장면이 바뀌므로 먼저 읽는다)
                try:
                    boxes = data_gen._read_boxes(seg, rgb.shape[1], rgb.shape[0])
                except Exception as ex:
                    on_event("error", f"박스 라벨을 읽지 못했습니다: {ex}")
            b64 = capture.to_png_b64(rgb)               # Cosmos 에 보낼 데이터만 만든다 (파일 저장은 '데이터 저장' 체크가 켜졌을 때 DriveRecorder 가 한다)
            if on_image:
                on_image(aisle, rgb)
            on_event("ask", f"통로{no} " + ("라이다 막힘 → 지금 보이는 모습의 이유를 Cosmos 에 물음" if lidar_blocked else "이미지를 Cosmos 에 보냄"))
            if lidar_blocked:
                # 라이다가 막힘을 감지해서 돌아간다. Cosmos 에게는 통과 여부가 아니라 '앞의 장면' 만 묻는다 (aisle_judge.explain_block).
                # '라이다 막힘' 말풍선은 그대로 두고, 이유가 오면 이유와 안전 안내를 더한다
                res = await loop.run_in_executor(None, aisle_judge.explain_block, b64)
                why = (res["reason"] or "이유를 받지 못했습니다") if not res["error"] else "Cosmos 응답을 받지 못했습니다"
                on_event("cosmos", f"통로{no} 라이다 막힘 · Cosmos 가 본 장면: {why}")
                _bubble(lambda b: b.show(False, f"통로{no} : 라이다 막힘", f"안전상의 문제로 돌아갑니다\n{why}"))
                if on_block:
                    on_block(aisle, why, res.get("raw"))
                return False, None
            _bubble(lambda b: b.thinking(f"통로{no}"))
            res = await loop.run_in_executor(None, aisle_judge.judge, b64, aisle)
            row = {"aisle": aisle, "state": eff, "truth_passable": incidents.passable(eff),
                   "decision": res["decision"], "obstacle": res["obstacle"], "reason": res["reason"],
                   "confidence": res["confidence"], "sec": res["sec"], "error": res["error"]}
            status["judgements"].append(row)
            if rec is not None:
                try:
                    rec.add(aisle, eff, rgb, boxes, res)                  # 원본·박스 그린 사진, 박스 좌표, Cosmos 응답을 저장
                except Exception as ex:
                    on_event("error", f"데이터를 저장하지 못했습니다: {ex}")
            if on_judgement:
                on_judgement(row)
            on_event("cosmos", f"통로{amr_scene.aisle_no(aisle)}: {'통과 가능' if res['decision'] else '통과 불가'} "
                               f"({res['obstacle'] or '-'}) {res['reason']}")
            # 말풍선: 제목 "통로1 : Pass / Reroute" (최종 판단 decision 을 따른다) + Cosmos 가 준 이유 (판단 결과 칸의 '이유' 와 같은 글).
            # Cosmos 가 본 물체(seen)는 판단 결과 칸의 '본 것' 에만 보여 준다
            _bubble(lambda b: b.show(res["decision"],
                                     f"통로{amr_scene.aisle_no(aisle)} : {'Pass' if res['decision'] else 'Reroute'}",
                                     res["reason"] or ""))
            # (통과 여부, 막힌 이유): 위험 요소가 사람뿐이면 'person' → 로봇은 지나가지 않고 그 자리에서 기다렸다 다시 판단한다
            return res["decision"], ("person" if aisle_judge.only_person_block(res) else None)

        if dynamic:
            # 돌발 상황: 걸어 다니는 작업자 (타임라인이 멈춘 지금 놓고, Play 되면 움직인다)
            n = dynamic_events.start(on_event=on_event)
            on_event("info", f"작업자 {n}명이 창고를 걸어 다닙니다")
            for _ in range(25):                     # 작업자 에셋이 렌더러에 들어갈 때까지
                await omni.kit.app.get_app().next_update_async()

        capture.set_gating(True)                    # 주행 중에는 Cosmos 카메라 렌더 갱신을 끄고, 사진 찍을 때만 켠다
        ok = await nav.drive_to(goal, on_checkpoint=None if mode == "lidar_only" else on_checkpoint,
                                on_event=on_event)
        status["ok"] = ok
        if ok:
            # 도착한 뒤 바로 바꾸지 않고 3초(실제 시간) 뒤에 위에서 보기로 바꿔 지나온 경로와 지점 이름을 보여 준다
            await _wait_real(ARRIVE_VIEW_DELAY_SEC, nav)
            _route(lambda r: r.show_top_view())
            on_event("info", f"도착 {ARRIVE_VIEW_DELAY_SEC:g}초 뒤 위에서 보기로 전환")
        if rec is not None:                          # 데이터 저장: 위에서 본 지도에 지나온 경로와 지점 이름을 그려 함께 저장한다 (도착하지 못하고 끝나도 지나온 데까지)
            try:
                import route_map
                if len(route_map.trail()) >= 2:
                    map_path = await route_map.save_map(rec.dir)
                    on_event("info", "지나온 경로 지도 저장: " + (os.path.join(os.path.basename(rec.dir), "route_map.png") if map_path
                                                         else "위에서 본 사진을 찍지 못해 저장하지 못했습니다"))
            except Exception as ex:
                on_event("error", f"경로 지도를 저장하지 못했습니다: {ex}")
        os.makedirs(os.path.dirname(LOG), exist_ok=True)
        with open(LOG, "a", encoding="utf-8") as f:
            f.write(json.dumps({"time": time.strftime("%Y-%m-%d %H:%M:%S"), "mode": mode, "aisles": truth, "arrived": ok,
                                "judgements": status["judgements"],
                                "events": status["events"]}, ensure_ascii=False) + "\n")
    except Exception:
        import traceback
        status["error"] = traceback.format_exc()
    finally:
        _ribbon()
        _bubble(lambda b: b.hide())                 # 플레이가 끝나면(도착·정지·실패) Cosmos 말풍선도 숨긴다
        _route(lambda r: r.stop_trail())            # 경로 기록을 멈춘다 (지나온 경로는 남겨 둬서 위에서 보기에서 계속 보인다)
        _route(lambda r: r.release_map())           # 지도 사진용 렌더 프로덕트는 쓰고 나서 치운다
        if seg is not None:
            try:
                data_gen._detach_seg()              # 박스 어노테이터는 주행이 끝나면 뗀다
            except Exception:
                pass
        capture.set_gating(False)                   # 주행이 끝나면 다시 항상 켠다 (멈춘 상태 사진·정확도 시험용)
        dynamic_events.stop()                       # 작업자를 숨긴다
        try:
            await _stop_and_reset()                 # 플레이가 끝나면(도착·정지·실패) 정지 버튼을 누른 것처럼 멈추고 AMR 을 출발점으로
        except Exception:
            pass
    status["done"] = True
    return status
