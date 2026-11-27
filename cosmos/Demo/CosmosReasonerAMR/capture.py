# 카메라 이미지 캡처 (Isaac Sim 안에서 사용)
#
#   import capture
#   rgb = await capture.grab_async("/World/AMR/FrontCamera")        # (H, W, 3) uint8
#   b64 = capture.to_png_b64(rgb)                                    # Cosmos 에 보낼 base64 (save="경로" 를 주면 원본 PNG 도 저장)
#
# - 카메라마다 Replicator 렌더 프로덕트를 한 번만 만들어 builtins 에 보관한다
#   (같은 카메라에 여러 번 만들면 GPU 메모리를 계속 쓴다).
# - 장면을 새로 만들면 예전 렌더 프로덕트는 무효가 되므로 reset() 으로 지운다.
# - 타임라인이 멈춰 있어도 orchestrator.step 으로 한 프레임을 렌더해서 가져온다.

import base64
import builtins
import io
import os

import numpy as np

RESOLUTION = (1280, 720)         # Cosmos 입력용. 640x360 에서는 바닥 박스를 놓치는 일이 많았다 (박스만 막힘 4건 중 1건만 감지)
ROOT = os.path.dirname(os.path.abspath(__file__))


def _cache():
    c = getattr(builtins, "_amr_capture", None)
    if c is None:
        c = {}
        builtins._amr_capture = c
    return c


# Cosmos 카메라(1280x720)는 렌더 프로덕트라서 평소에는 Play 내내 매 프레임 렌더링된다. 사진은 통로 입구에서 한 번만 필요하므로,
# 주행 중에는 렌더 갱신을 끄고(set_updates_enabled(False)) 사진을 찍을 때만 잠깐 켠다 (시험: 프레임 약 +16%,
# 켜고 끄기를 5번 반복해도 사진이 모두 나왔다). 렌더 프로덕트를 지우거나 새로 만드는 것이 아니라 '갱신만' 끄는 것이다.
_gate = {"on": False}


def set_gating(on):
    """True: 주행 중 카메라 렌더 갱신을 끈다 (사진 찍을 때만 켠다). False: 다시 항상 켠다. 타임라인이 멈춘 상태에서 부를 것."""
    _gate["on"] = bool(on)
    for v in _cache().values():
        try:
            v["rp"].hydra_texture.set_updates_enabled(not _gate["on"])
        except Exception:
            pass


def _set_updates(v, on):
    try:
        v["rp"].hydra_texture.set_updates_enabled(bool(on))
    except Exception:
        pass


def reset():
    """새 장면을 만든 뒤 호출: 예전 렌더 프로덕트 정리."""
    c = _cache()
    for v in c.values():
        try:
            v["ann"].detach()
            v["rp"].destroy()
        except Exception:
            pass
    c.clear()


def drop(camera, resolution=RESOLUTION):
    """한 카메라의 렌더 프로덕트만 정리한다 (reset 은 전부 지운다). 주행 뒤에도 남겨 두면 평소처럼 매 프레임 렌더링되므로,
    잠깐만 쓴 카메라(위에서 본 지도 사진 등)는 쓰고 나서 이것으로 치운다. 핸들로 destroy 하므로 안전하다 (RemovePrim 은 하지 않는다)."""
    v = _cache().pop((camera, tuple(resolution)), None)
    if v:
        try:
            v["ann"].detach()
            v["rp"].destroy()
        except Exception:
            pass


def _ensure(camera, resolution=RESOLUTION):
    import omni.replicator.core as rep
    c = _cache()
    key = (camera, tuple(resolution))
    if key not in c:
        rp = rep.create.render_product(camera, resolution)
        ann = rep.AnnotatorRegistry.get_annotator("rgb")
        ann.attach([rp])
        c[key] = {"rp": rp, "ann": ann}
    return c[key]


def prepare(camera, resolution=RESOLUTION):
    """렌더 프로덕트를 미리 만든다. 타임라인이 멈춘 상태에서 부를 것
    (Play 중에 새로 만든 렌더 프로덕트는 값이 비어 나온 적이 있다 — 라이다 주행 중 캡처에서 확인)."""
    _ensure(camera, resolution)


async def grab_async(camera, resolution=RESOLUTION, steps=3):
    """최신 프레임 (H, W, 3) uint8. 렌더가 준비되지 않았으면 None.

    - Play 중: 프레임만 기다리면 새 화면이 들어온다.
    - 멈춘 상태: Replicator 는 기본값(/omni/replicator/captureOnPlay = True)이면 Play 중에만 찍는다.
      이 값을 False 로 바꾸고 Replicator 에게 한 프레임씩 렌더를 시킨다.
      (예전에 찍으려고 잠깐 Play/Stop 을 했더니, 지운 물체가 화면에 남는 등 렌더가 장면과 어긋났다)"""
    # 목적지 표시(빛나는 화살표·링)는 보기용이라 Cosmos 사진에는 안 나오게 찍는 동안 숨긴다
    # (통로 끝에 빛나는 물체가 보이면 장애물로 볼 수 있다)
    try:
        import amr_scene
        amr_scene.set_goal_marker_visible(False)
    except Exception:
        amr_scene = None
    try:
        import rtx_lidar
        rtx_lidar.pause_debug_draw(True)             # 라이다 점 표시(뷰포트용)는 사진을 찍는 동안 숨긴다
    except Exception:
        rtx_lidar = None
    try:
        return await _grab(camera, resolution, steps)
    finally:
        if amr_scene:
            amr_scene.set_goal_marker_visible(True)
        if rtx_lidar:
            rtx_lidar.pause_debug_draw(False)


async def _grab(camera, resolution, steps):
    import carb.settings
    import omni.kit.app
    import omni.replicator.core as rep
    import omni.timeline
    app = omni.kit.app.get_app()
    settings = carb.settings.get_settings()
    v = _ensure(camera, resolution)
    playing = omni.timeline.get_timeline_interface().is_playing()
    gated = playing and _gate["on"]
    if gated:
        _set_updates(v, True)                     # 사진 찍는 동안만 렌더 갱신을 켠다 (끈 채로 찍으면 옛 화면이 나올 수 있다)
    if playing:
        # 멈춘 상태 캡처가 captureOnPlay 를 False 로 바꿔 두면 Play 중에도 annotator 가 비어 나온다
        # (라이다 주행에서 '카메라 이미지를 가져오지 못해' 발생) → Play 중에는 반드시 True 로 되돌린다
        settings.set("/omni/replicator/captureOnPlay", True)
        for _ in range(max(steps, 3) * 2 + (6 if gated else 0)):      # 켠 직후에는 새 프레임이 나올 때까지 조금 더 기다린다
            await app.next_update_async()
    else:
        settings.set("/omni/replicator/captureOnPlay", False)
        for _ in range(max(steps, 3)):
            await rep.orchestrator.step_async(rt_subframes=4, pause_timeline=True)
    data = await _poll(v, app, 60)
    if data is None and playing:
        # 그래도 비어 있으면 Replicator 에게 직접 한 프레임 렌더를 시킨다 (타임라인은 멈추지 않음)
        for _ in range(2):
            await rep.orchestrator.step_async(rt_subframes=4, pause_timeline=False)
        data = await _poll(v, app, 30)
    if not playing:
        settings.set("/omni/replicator/captureOnPlay", True)   # 기본값으로 되돌려 둔다
    if gated:
        _set_updates(v, False)                    # 다시 끈다
    return data


async def _poll(v, app, frames):
    for _ in range(frames):
        data = v["ann"].get_data()
        if data is not None and getattr(data, "size", 0) > 0:
            return np.asarray(data)[:, :, :3].copy()
        await app.next_update_async()
    return None


def to_png_b64(rgb, save=None):
    """Cosmos 에 보낼 base64. 형식은 amr_config.IMAGE_FORMAT (기본 JPEG — 이름은 예전 호환용으로 그대로 둠).
    save 경로가 있으면 기록용으로 원본(PNG)을 저장한다."""
    from PIL import Image
    import amr_config
    pil = Image.fromarray(rgb).convert("RGB")
    if save:
        path = save if os.path.isabs(save) else os.path.join(ROOT, save)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        pil.save(path)
    buf = io.BytesIO()
    if amr_config.IMAGE_FORMAT == "jpeg":
        pil.save(buf, format="JPEG", quality=amr_config.JPEG_QUALITY)
    else:
        pil.save(buf, format="PNG")
    return base64.b64encode(buf.getvalue()).decode("ascii")
