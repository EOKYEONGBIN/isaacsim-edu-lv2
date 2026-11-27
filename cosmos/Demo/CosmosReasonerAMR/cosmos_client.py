# Cosmos3 Reasoner (NIM) 호출 공통 모듈 — ①②③ 모드가 함께 쓴다
#
#   import cosmos_client as cc
#   text = cc.ask(system, user_text, image_b64=png_b64)        # 이미지 1장
#   text = cc.ask(system, user_text, video_path="clip.mp4")    # 영상 (③ CCTV 분석)
#   data = cc.extract_json(text)
#
# 네트워크 호출은 오래 걸리므로 Isaac Sim UI 에서는 별도 스레드(run_in_executor)로 부른다.

import base64
import json
import re
import time
import urllib.error
import urllib.request

import amr_config as config


def _content(text, image_b64=None, video_path=None):
    if not image_b64 and not video_path:
        return text
    parts = []
    if image_b64:
        # base64 앞부분으로 형식 판별: JPEG 는 "/9j/", PNG 는 "iVBOR" 로 시작
        mime = "image/jpeg" if image_b64.startswith("/9j/") else "image/png"
        parts.append({"type": "image_url", "image_url": {"url": f"data:{mime};base64," + image_b64}})
    if video_path:
        with open(video_path, "rb") as f:
            v = base64.b64encode(f.read()).decode("ascii")
        parts.append({"type": "video_url", "video_url": {"url": "data:video/mp4;base64," + v}})
    parts.append({"type": "text", "text": text})
    return parts


def _redact(body):
    """화면에 보여 줄 요청 JSON: 사진(base64) 은 길어서 '<이미지 N KB>' 로 바꾼다 (나머지는 실제로 보낸 그대로)."""
    import copy
    b = copy.deepcopy(body)
    for m in b.get("messages", []):
        if isinstance(m.get("content"), list):
            for part in m["content"]:
                url = (part.get("image_url") or {}).get("url") if part.get("type") == "image_url" else None
                if url and ";base64," in url:
                    head, data = url.split(";base64,", 1)
                    part["image_url"]["url"] = f"{head};base64,<이미지 {len(data) * 3 // 4 // 1024} KB>"
                elif part.get("type") == "video_url":
                    part["video_url"]["url"] = "<영상 데이터>"
    return b


def ask(system, text, image_b64=None, video_path=None, max_tokens=1024, temperature=0.0, extra=None):
    """Reasoner 에 한 번 묻고 답 문자열을 돌려준다. 실패하면 예외."""
    body = {"model": config.NIM_MODEL,
            "messages": [{"role": "system", "content": system},
                         {"role": "user", "content": _content(text, image_b64, video_path)}],
            "max_tokens": max_tokens, "temperature": temperature}
    if video_path:
        body["media_io_kwargs"] = {"video": {"fps": 4.0}}       # NIM 문서 권장 (영상 4 fps)
    if extra:
        body.update(extra)
    ask.last_body = _redact(body)                       # 화면의 Request 칸이 보여 주는 마지막 요청
    req = urllib.request.Request(config.NIM_URL, data=json.dumps(body).encode("utf-8"),
                                 headers={"Content-Type": "application/json"})
    t0 = time.time()
    with urllib.request.urlopen(req, timeout=config.TIMEOUT_SEC) as r:
        data = json.loads(r.read().decode("utf-8"))
    ask.last_sec = round(time.time() - t0, 2)
    return data["choices"][0]["message"]["content"]


ask.last_sec = None
ask.last_body = None


def extract_json(text):
    """답에서 첫 번째 JSON 객체를 꺼낸다 (```json 블록, 앞뒤 설명 허용)."""
    m = re.search(r"```(?:json)?\s*(\{.*?\})\s*```", text, re.S)
    s = m.group(1) if m else None
    if s is None:
        start = text.find("{")
        depth = 0
        for i in range(start, len(text)) if start >= 0 else []:
            depth += {"{": 1, "}": -1}.get(text[i], 0)
            if depth == 0:
                s = text[start:i + 1]
                break
    if not s:
        raise ValueError("no JSON object in reply")
    return json.loads(s)


def server_ok():
    """서버가 응답하는지 빠르게 확인 (/v1/models)."""
    url = config.NIM_URL.split("/v1/")[0] + "/v1/models"
    try:
        with urllib.request.urlopen(url, timeout=5) as r:
            ids = [m["id"] for m in json.loads(r.read().decode("utf-8")).get("data", [])]
        return True, ids
    except (urllib.error.URLError, TimeoutError, OSError, ValueError) as e:
        return False, str(e)
