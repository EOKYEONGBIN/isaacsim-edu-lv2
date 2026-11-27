# ① 통로 판단: AMR 앞 카메라 이미지 → Cosmos 가 통과 가능 여부 판단
#
#   import aisle_judge
#   result = aisle_judge.judge(png_b64, aisle="B")   # 네트워크 호출 (스레드에서 부를 것)
#   result = {"passable": bool, "obstacle": str, "confidence": "high|medium|low", "reason": str,
#             "decision": bool (코드가 내린 최종 판단), "raw": 원문, "sec": 응답 시간, "error": 오류 or None}
#
# 원칙
#   - 장애물 정답 위치는 Cosmos 에 주지 않는다 (주면 판단이 아니라 받아쓰기가 된다). 정답은 채점에만 쓴다.
#   - 대화 기억 없이 매번 새로 판단한다 (이전 통로 상황이 섞이지 않게).
#   - 애매하면 안전 쪽(통과 불가)으로 결정한다.

import re

import amr_config as config
import cosmos_client as cc
import incidents

# ---- Cosmos 프롬프트 = 장애물별 줄 + 고정 답 형식(FORMAT_TAIL) ----------------------------------------------------
# 장애물별 줄(막힘 / 통과)은 incidents.py 의 장애물 표에 있다. 익스텐션의 '장애물별 판단' 버튼이 set_rule() 로 줄을 바꾼다.
# 사용자는 프롬프트 칸의 글(PROMPT_TEXT)을 직접 고칠 수도 있다. FORMAT_TAIL 은 칸에 보이지 않고 보낼 때 맨 뒤에 붙인다
# (고치다가 형식이 깨져 판단이 오류로 나오지 않게). 역할·카메라·차선 설명과 pass/reroute 설명은 일부러 넣지 않는다:
# 예전에는 칸 안의 긴 틀이었는데, 틀이 길면 Cosmos 가 더 보수적으로 봐서 '통과' 로 정한 고깔을 막힘으로 보는 일이 늘었다
# (같은 사진 24장: 고깔 통과 21/24 -> 틀을 빼면 24/24). 틀을 짧게 꼬리에 넣어도 박스 한 장을 놓쳤다 (23/24) -> 아예 뺐다.
# pass / reroute 의 뜻은 장애물 줄 끝의 괄호 (reroute) / (pass) 가 알려 준다.
SHELF_LINE = "Items stored on the shelves and pallets placed under the bottom shelf outside the yellow lines do NOT matter."
FORMAT_TAIL = """Reply with ONLY one JSON object and put "action" first. "seen" lists the objects standing or lying on the lane floor
(not shelves, pillars or floor markings), including harmless ones. "hazards" lists only the things that block the robot.
Each list has at most 3 short items, each different (empty list if none).
Write "seen", "obstacle" and "reason" in Korean Hangul only (no Chinese characters).
{"action": "pass" or "reroute", "seen": ["<차선 바닥의 물체, 짧게>"], "hazards": ["<hazard in the lane>"],
 "obstacle": "<짧은 요약, 없으면 없음>", "confidence": "high" or "medium" or "low", "reason": "<한 문장>"}"""
PROMPT_MAX_CHARS = 2000                      # 프롬프트 칸(장애물 줄 + 직접 쓴 글)의 글자 수 제한


def _rule_line(key, mode=None):
    o = incidents.OBSTACLES[key]
    return o["block_line"] if (mode or incidents.get_rule(key)) == "block" else o["pass_line"]


def build_prompt():
    """장애물별 줄(지금의 막힘/통과 설정) + 선반 문장. 상황 설명과 답 형식(FORMAT_TAIL)은 포함하지 않는다."""
    return "\n".join([_rule_line(k) for k in incidents.rule_keys()] + [SHELF_LINE])


PROMPT_TEXT = build_prompt()                 # 프롬프트 칸의 글 (판단할 때마다 읽는다)


def set_rule(key, mode):
    """장애물 하나를 막힘("block") / 통과("pass") 로 바꾼다: 정답(incidents.RULE)과 프롬프트 글의 그 줄이 함께 바뀐다.
    사용자가 그 줄을 고쳐서 못 찾으면 글에 새 줄을 끼워 넣고 False 를 돌려준다."""
    global PROMPT_TEXT
    old = _rule_line(key)
    incidents.set_rule(key, mode)
    new = _rule_line(key)
    if old == new:
        return True
    lines = PROMPT_TEXT.split("\n")
    found = old in lines
    if found:
        lines[lines.index(old)] = new
    else:
        lines.insert(lines.index(SHELF_LINE) if SHELF_LINE in lines else len(lines), new)
    PROMPT_TEXT = "\n".join(lines)
    return found


def set_prompt_text(text):
    global PROMPT_TEXT
    PROMPT_TEXT = text


def reset_prompt():
    """장애물별 판단과 프롬프트 글을 처음 상태로 되돌린다."""
    global PROMPT_TEXT
    incidents.reset_rules()
    PROMPT_TEXT = build_prompt()


ACTION_RE = re.compile(r'"action"\s*:\s*"(pass|wait|reroute)"')   # 잘린 답에서 action 만 건지기
CJK_RE = re.compile("[\u3000-\u303f\u4e00-\u9fff]")      # 한자와 한자 문장부호(。、) 제거용

NEGATIONS = ("없", "no obj", "no obstacle", "clear of", "is clear", "are clear")

# 같은 말 반복(→ 답이 잘려 JSON 오류)을 막는 생성 옵션. NIM(vLLM) 이 받는 값
GEN_EXTRA = {"repetition_penalty": 1.1}
# '통과' 라고 하기 전에 가운데 먼 쪽을 2배 확대해서 한 번 더 묻는다
# (Nova Carter 카메라는 시야가 넓어 약 10 m 앞 박스가 작게 보여 놓친 적이 있다. 확대하면 4/4 감지,
#  대신 한쪽에만 있는 박스를 막힘으로 보는 경우가 생긴다 → 안전 쪽 실수라 받아들임)
DOUBLE_CHECK = True
CROP_BOX = (0.25, 0.28, 0.75, 0.78)          # (왼, 위, 오른, 아래) 비율


def center_crop_b64(png_b64):
    """가운데 먼 쪽을 잘라 원래 크기로 확대한 PNG base64. PIL 이 없으면 None."""
    try:
        import base64, io
        from PIL import Image
    except ImportError:
        return None
    im = Image.open(io.BytesIO(base64.b64decode(png_b64))).convert("RGB")
    W, H = im.size
    l, t, r, b = CROP_BOX
    c = im.crop((int(W * l), int(H * t), int(W * r), int(H * b))).resize((W, H))
    buf = io.BytesIO()
    if config.IMAGE_FORMAT == "jpeg":
        c.save(buf, "JPEG", quality=config.JPEG_QUALITY)
    else:
        c.save(buf, "PNG")
    return base64.b64encode(buf.getvalue()).decode("ascii")


PERSON_WORDS = ("person", "people", "worker", "human", "pedestrian", "staff", "employee", "man ", "woman",
                "사람", "작업자", "직원", "인부", "근로자", "사원", "남자", "여자")


NON_PERSON_WORDS = ("box", "carton", "pallet", "cone", "wet", "spill", "liquid", "water", "sign", "forklift", "vehicle", "truck",
                    "barrier", "obstacle", "박스", "상자", "팔레트", "콘", "원뿔", "액체", "물", "표지", "지게차", "차량", "장애물")


def only_person_block(res):
    """Cosmos 가 '막힘' 이라고 했는데 위험 요소가 사람뿐인가 (박스·액체 등이 하나라도 섞이면 아님).
    사람은 로봇을 보면 비켜 주므로 이 경우는 통과시킨다.
    hazards 는 'person' 같은 정해진 말이 아니라 '직원 두 명' 처럼 자유 문장으로 올 때가 많아서, 항목마다
    사람 단어가 있고 사람이 아닌 단어는 없는지로 본다 (hazards 가 비면 obstacle 글을 본다)."""
    if res.get("decision") or res.get("error"):
        return False
    items = [str(h).lower() for h in (res.get("hazards") or []) if str(h).strip()]
    if not items and str(res.get("obstacle") or "").strip():
        items = [str(res["obstacle"]).lower()]
    if not items:
        return False
    return all(any(w in t for w in PERSON_WORDS) and not any(w in t for w in NON_PERSON_WORDS) for t in items)


def judge(png_b64, aisle="", retries=1, crop_b64=None, double_check=None):
    """최종 판단. 1차 판단이 '통과' 면 확대 이미지로 한 번 더 확인하고, 거기서 걸리면 통과시키지 않는다."""
    out = _judge_once(png_b64, aisle, retries)
    if (DOUBLE_CHECK if double_check is None else double_check) and out["decision"]:
        crop = crop_b64 or center_crop_b64(png_b64)
        if crop is None:
            out["double_check"] = "skipped"
            return out
        o2 = _judge_once(crop, aisle, retries)
        out["double_check_request"] = o2.get("request")              # 확대 재확인으로 한 번 더 보낸 요청과 그 답 (화면에 보여 준다)
        out["double_check_raw"] = o2.get("raw")
        out["double_check"] = o2.get("action") or ("error" if o2["error"] else None)
        out["sec"] = round((out["sec"] or 0) + (o2["sec"] or 0), 2)
        if not o2["decision"]:
            out.update(decision=False, passable=False, action=o2.get("action") or "reroute",
                       obstacle=o2["obstacle"], reason="확대 재확인: " + (o2["reason"] or o2["error"] or ""),
                       hazards=o2.get("hazards"))
    return out


USER_TEXT = "Aisle {aisle} entrance. Can the AMR drive through this aisle?"      # 사진과 함께 보내는 질문 (칸에는 없고 고칠 수 없다)


def user_text(aisle=""):
    """사진과 함께 보내는 질문. {aisle} 은 통로 글자(A~D)."""
    return USER_TEXT.format(aisle=aisle)


# ---- 라이다 막힘 때 보내는 프롬프트 (통로 입구 판단과 다르다: 통과/막힘을 묻지 않고 '앞의 장면' 만 묻는다) ----------------
# 로봇이 통로 안에서 라이다에 막혀 멈췄을 때, 앞에 무엇이 있는지 Cosmos 가 말해 주는 용도다. 프롬프트 칸의 글은 쓰지 않는다.
BLOCK_SYSTEM = """Describe in ONE short Korean sentence what you see in front of the robot.
Reply with ONLY one JSON object: {"reason": "<한 문장, 한국어>"}"""
BLOCK_USER = "The robot stopped because the way ahead is blocked. What scene is in front of the robot?"


def explain_block(png_b64, retries=1):
    """라이다 막힘 때: 앞의 장면을 Cosmos 에게 묻는다. 결과 {"reason": 한 문장, "sec", "raw", "error"} (판단은 하지 않는다)."""
    out = {"reason": "", "sec": None, "raw": "", "error": None}
    for attempt in range(retries + 1):
        try:
            raw = cc.ask(BLOCK_SYSTEM, BLOCK_USER, image_b64=png_b64, max_tokens=256, extra=GEN_EXTRA)
            out["raw"], out["sec"] = raw, cc.ask.last_sec
            try:
                text = str(cc.extract_json(raw).get("reason", "")).strip()
            except ValueError:
                text = raw.strip()                                  # JSON 이 아니면 글 그대로 (코드 블록 표시만 뺀다)
                text = text.replace("```json", "").replace("```", "").strip()
            out["reason"] = CJK_RE.sub("", text)
            out["error"] = None
            break
        except Exception as e:
            out["error"] = f"{type(e).__name__}: {e}"
    return out


def effective_system():
    """Cosmos 에 보내는 시스템 프롬프트: 프롬프트 칸의 글(최대 PROMPT_MAX_CHARS 자) + 고정 답 형식.
    칸을 비우면 답 형식만 보낸다 (예전에는 비면 기본 프롬프트로 채워 보내서, 비운 줄 알았는데 채워진 채로 갔다)."""
    text = "".join(ch for ch in (PROMPT_TEXT or "") if ch == "\n" or ch >= " ").strip()[:PROMPT_MAX_CHARS]
    return (text + "\n\n" + FORMAT_TAIL) if text else FORMAT_TAIL


def _judge_once(png_b64, aisle="", retries=1):
    system = effective_system()
    out = {"passable": None, "obstacle": "", "seen": [], "confidence": "low", "reason": "", "decision": False,
           "raw": "", "request": None, "sec": None, "error": None}
    text = user_text(aisle)
    for attempt in range(retries + 1):
        try:
            raw = cc.ask(system, text, image_b64=png_b64, max_tokens=512, extra=GEN_EXTRA)
            out["raw"], out["sec"] = raw, cc.ask.last_sec
            out["request"] = cc.ask.last_body                        # 이 호출에서 보낸 요청 JSON (화면의 Request 칸용, 사진은 크기만)
            try:
                j = cc.extract_json(raw)
            except ValueError:
                # 답이 잘려 JSON 이 깨져도 맨 앞의 action 은 읽을 수 있다 (v5 는 action 을 먼저 쓴다)
                m = ACTION_RE.search(raw)
                if not m:
                    raise
                j = {"action": m.group(1), "confidence": "low" if m.group(1) == "pass" else "medium",
                     "obstacle": "", "reason": "(답이 잘려 행동만 읽음)"}
                out["truncated"] = True
            # 이유를 한자(중국어)로만 써서 한자를 지우면 거의 남지 않는 답: 한 번 더 묻고, 그래도 같으면 아래에서 안내 문구로 바꾼다
            raw_reason = str(j.get("reason", ""))
            lost = bool(CJK_RE.search(raw_reason)) and len(CJK_RE.sub("", raw_reason).strip(" .,·")) < 3
            out["reason_lost"] = lost
            if lost and attempt < retries:
                continue
            action = str(j.get("action", "")).strip().lower()           # v4 는 행동으로 답한다
            if action in ("pass", "wait", "reroute"):
                passable = action == "pass"
            else:
                passable = j.get("passable", j.get("path_clear"))      # v3 는 path_clear 로 답한다
            out.update(passable=bool(passable), obstacle=str(j.get("obstacle", "")),
                       confidence=str(j.get("confidence", "low")).lower(), reason=str(j.get("reason", "")),
                       floor_objects=j.get("floor_objects"), blocked_percent=j.get("blocked_percent"),
                       action=action or None, hazards=j.get("hazards"),
                       seen=[s for s in (CJK_RE.sub("", str(x)).strip() for x in (j.get("seen") or [])) if s][:4],
                       error=None)
            # 모델이 경로 위 물체를 적어 놓고도 '통과 가능' 이라고 하는 모순은 막힘으로 본다
            # (v3 시험에서 "센터에 상자가 놓여 있습니다" 라고 쓰고 path_clear=true 로 답한 적이 있다)
            ok_words = incidents.words("pass")        # 사용자가 '통과' 로 정한 장애물은 막힘 근거가 아니다
            objs = [o for o in (j.get("path_objects") or j.get("hazards") or [])
                    if str(o).strip() not in ("", "없음", "none", "None")
                    and not str(o).strip().lower().startswith(("no ", "없"))      # "no people" 같은 '없음' 표시는 빼기
                    and not any(w in str(o).lower() for w in ok_words)]            # 판단은 통과 / 막힘 두 가지로 정리: 예전 v4~v6 의 '대기(wait)' 는 막힘(reroute)으로 본다
            if action == "wait":
                action = "reroute"
                out["action"] = "reroute"
            if out["passable"] and objs:
                out["passable"] = False
            # 차선 바닥에서 본 것(seen)에 '막힘' 으로 정한 장애물이 있는데 통과라고 한 모순도 막힘으로 본다
            # (예: seen 에 "yellow caution sign" 을 적고도 hazards 는 비우고 "aisle is clear" 라고 답한 경우)
            block_words = incidents.words("block")
            if out["passable"] and any(any(w in str(x).lower() for w in block_words) for x in out["seen"]):
                out["passable"] = False
            # 설명 문장에는 물체가 있다고 쓰고 목록은 비워 둔 모순도 막힘으로 본다 (안전 우선)
            text_all = (out["obstacle"] + " " + out["reason"]).lower()
            if out["passable"] and any(w in text_all for w in incidents.words("block")) \
                    and not any(n in text_all for n in NEGATIONS):
                out["passable"] = False
            # 모델이 '많이 막혔다' 고 하면서 통과 가능이라고 하는 모순은 막힘으로 본다
            try:
                if out["passable"] and float(j.get("blocked_percent", 0)) >= 60:
                    out["passable"] = False
            except (TypeError, ValueError):
                pass
            break
        except Exception as e:                      # 형식 오류·시간 초과 → 한 번 더, 그래도 실패면 안전 쪽
            out["error"] = f"{type(e).__name__}: {e}"
    # 코드의 최종 판단: 확실히 '통과 가능' 일 때만 통과 (모르면 막힘으로 본다)
    out["decision"] = bool(out["passable"]) and out["error"] is None and out["confidence"] != "low"
    # 행동도 최종 판단에 맞춘다: 코드가 막힘으로 바꿨는데 행동이 pass 로 남아 있으면 reroute
    if not out["decision"] and out.get("action") in (None, "pass"):
        out["action"] = "reroute"
    for k in ("obstacle", "reason"):
        out[k] = CJK_RE.sub("", out.get(k) or "")
    if out.get("reason_lost"):
        out["reason"] = "이유를 한국어로 받지 못했습니다"
    return out
