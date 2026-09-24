"""camara_2.0 — ตรวจสีบล็อกในตาราง 3x3 แบบยืดหยุ่น (ต่อจาก camara_1.0)

เรียกจากโปรแกรมอื่น:  get_blocks() -> {ช่อง: สี} เช่น {1: "g", 2: "r", ...} ครบทุกช่อง
ตั้งค่ากรอบ/สี (UI):   python3 camara_2.0.py

ต่างจาก 1.0:
- สีเดียวกันซ้ำกี่ก้อนก็ได้ ไม่มี over/no color block อีกแล้ว (1.0 บังคับสีละ 2 ก้อนเป๊ะ)
- มองทีละ "ช่อง" แทนที่จะมองทีละ "สี" -> แต่ละช่องสรุปสีของตัวเอง
- ช่องที่มีหลายสีปนกัน -> เอาสีที่มีพิกเซลมากที่สุด

ระบบสำรอง (ใช้ต่อเมื่อ "หาสีไม่เจอ" จริงๆ เท่านั้น ไล่จากหลักฐานมากไปน้อย):
    1. เจอสีถึงเกณฑ์ตั้งแต่ range ปกติ          -> sure  (ปกติควรได้ทุกช่องแบบนี้)
    2. ไม่ถึงเกณฑ์ -> ขยาย HSV range ทีละขั้น    -> sure  (grow > 0)
    3. ขยายจนสุดแล้วยังไม่ถึงเกณฑ์ แต่ยังพอมีพิกเซลของสีอยู่บ้าง -> เอาสีที่มีมากสุด (weak)
    4. ไม่เหลือพิกเซลของสีไหนเลย -> เดาจาก H เฉลี่ยของช่อง เอาสีที่ใกล้ที่สุด (hue)
ข้อ 3-4 คือการเดา จะถูกทำเครื่องหมายไว้ทั้งบนหน้าจอและตอนรันจริง

หมายเลขช่องเป็นมุมมอง "ภาพกล้อง" เสมอ (ซ้าย->ขวา, บน->ล่าง):
    1 2 3
    4 c 5
    6 7 8
ฝั่ง main_0.7 เป็นคนกลับด้าน 180° เองถ้าตั้ง flip_camera ไว้
"""

import os
import glob
import json
import math
import cv2 as cv
import numpy as np

CONFIG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "camera_config.json")

# ข้อความที่ขึ้นบนหน้าต่าง OpenCV ต้องเป็นอังกฤษ (putText วาดภาษาไทยไม่ได้ จะขึ้นเป็น ???)
COLOR_NAMES = {"g": "green", "r": "red", "y": "yellow", "b": "blue"}
DRAW_BGR = {"g": (0, 220, 0), "r": (0, 0, 255), "y": (0, 220, 220), "b": (255, 150, 0)}

# ช่วง HSV ตั้งต้น (OpenCV: H 0-179, S/V 0-255) แดงมี 2 ช่วงเพราะ H วนรอบ 0
DEFAULT_CONFIG = {
    # กล้องที่อยากใช้ก่อน — เลข index หรือ path ก็ได้
    # path จาก /dev/v4l/by-id/ จะผูกกับตัวกล้อง ไม่สลับเวลาถอด-เสียบ USB
    # ถ้าเปิดตัวนี้ไม่ได้ (ไม่ได้เสียบกล้องเสริม) จะไล่หากล้องอื่นในเครื่องให้เอง เช่น กล้องโน้ตบุ๊ก
    "camera_index": 0,
    "warmup_frames": 25,    # ทิ้งเฟรมแรกๆ ระหว่างที่กล้องปรับแสง (C270 พ่นภาพดำช่วงแรก)
    "frame": {"cx": 320, "cy": 240, "w": 300, "h": 300},  # กรอบ 3x3 (ปรับกว้าง/สูงแยกกันได้)
    # พื้นที่ต่ำสุดที่นับว่า "เจอสีนั้นจริง" ในหนึ่งช่อง คิดเป็นสัดส่วนของพื้นที่ 1 ช่อง
    # กรอบใหญ่ขึ้น (ซูมเข้า / กล้องใกล้ขึ้น) บล็อกในภาพก็ใหญ่ขึ้นตาม เกณฑ์จึงโตตามไปเอง
    "min_area_ratio": 0.04,
    "cell_inset": 0.15,     # หดขอบช่องเข้ามาข้างละกี่ส่วน กันสีช่องข้างๆ กับเส้นตารางล้นเข้ามา
    "expand_limit": 6,      # ขยาย range ได้กี่ขั้นก่อนจะยอมเดาสีที่ใกล้ที่สุด
    "colors": {
        "g": {"h": [[40, 85]], "s": [80, 255], "v": [60, 255]},
        "r": {"h": [[0, 10], [170, 179]], "s": [90, 255], "v": [60, 255]},
        "y": {"h": [[20, 35]], "s": [90, 255], "v": [80, 255]},
        "b": {"h": [[90, 130]], "s": [80, 255], "v": [60, 255]},
    },
}

# ช่อง (แถว, คอลัมน์) ตามมุมมองภาพ ช่องกลาง (1,1) คือจุดสร้าง Tower ไม่นับเป็นบล็อก
CELL_AT = {(0, 0): 1, (0, 1): 2, (0, 2): 3,
           (1, 0): 4, (1, 1): "c", (1, 2): 5,
           (2, 0): 6, (2, 1): 7, (2, 2): 8}
CELL_POS = {cell: pos for pos, cell in CELL_AT.items()}
BLOCK_CELLS = [c for c in (1, 2, 3, 4, 5, 6, 7, 8)]


class CameraError(Exception):
    """เปิด/อ่านกล้องไม่สำเร็จ — ฝั่ง main จะจับ error นี้แล้วให้กรอกลำดับเอง
    (2.0 ไม่โยน error เรื่องสีอีกแล้ว ทุกช่องได้สีเสมอ)"""


def load_config():
    cfg = json.loads(json.dumps(DEFAULT_CONFIG))  # deep copy
    data = {}
    try:
        with open(CONFIG_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
        cfg.update({k: v for k, v in data.items() if k != "colors"})
        # merge ทีละสี ไม่ใช่ update ทั้ง dict — ไฟล์ที่ตั้งมาแค่บางคีย์ (เช่นปรับแต่ h)
        # จะได้ไม่ทำให้ s/v หายไปทั้งคู่ แล้วไป KeyError ตอนสร้างมาสก์
        for name, spec in (data.get("colors") or {}).items():
            if isinstance(spec, dict):
                cfg["colors"].setdefault(name, {}).update(spec)
        for name in [n for n, spec in cfg["colors"].items()
                     if not all(k in spec for k in ("h", "s", "v"))]:
            print(f"⚠️ สี '{name}' ใน camera_config.json ขาดคีย์ h/s/v — ข้ามสีนี้ไป")
            cfg["colors"].pop(name)
    except FileNotFoundError:
        pass
    except Exception as e:
        print(f"⚠️ อ่าน camera_config.json ไม่ได้ ({e}) ใช้ค่าเริ่มต้น")
    # ไฟล์เก่าเก็บ min_area เป็นพิกเซลตายตัว -> แปลงเป็นสัดส่วนของช่องตามกรอบที่เซฟไว้ตอนนั้น
    old = cfg.pop("min_area", None)
    if old is not None and "min_area_ratio" not in data:
        cw, ch = cell_size(cfg)
        cfg["min_area_ratio"] = round(old / (cw * ch), 4)
        print(f"ℹ️ แปลง min_area {old} px -> min_area_ratio {cfg['min_area_ratio']} "
              f"(ช่องตอนนั้นขนาด {cw:.0f}x{ch:.0f} px)")
    return cfg


def save_config(cfg):
    try:
        with open(CONFIG_FILE, "w", encoding="utf-8") as f:
            json.dump(cfg, f, indent=4, ensure_ascii=False)
        print("💾 บันทึก camera_config.json แล้ว")
    except Exception as e:
        print(f"❌ บันทึกไม่สำเร็จ: {e}")


# ==========================================
# 📐 กรอบตาราง
# ==========================================
def grid_rect(cfg):
    """มุมซ้ายบน + กว้าง/สูง ของกรอบ 3x3"""
    f = cfg["frame"]
    w = f.get("w", f.get("size", 300))
    h = f.get("h", f.get("size", 300))
    return f["cx"] - w // 2, f["cy"] - h // 2, w, h


def cell_size(cfg):
    """ขนาด 1 ช่องของตาราง 3x3 (พิกเซล)"""
    _, _, w, h = grid_rect(cfg)
    return w / 3.0, h / 3.0


def min_area_of(cfg):
    """พื้นที่ต่ำสุดที่นับว่าเจอสีนั้นจริง (พิกเซล) = สัดส่วนที่ตั้งไว้ x พื้นที่ 1 ช่อง"""
    cw, ch = cell_size(cfg)
    return max(50, int(cfg.get("min_area_ratio", 0.04) * cw * ch))


def cell_rect(cfg, cell):
    """กรอบของช่องหนึ่ง หดขอบเข้ามาตาม cell_inset กันสีช่องข้างๆ ล้นเข้ามา"""
    x0, y0, w, h = grid_rect(cfg)
    row, col = CELL_POS[cell]
    sx, sy = w / 3.0, h / 3.0
    inset = cfg.get("cell_inset", 0.15)
    mx, my = sx * inset, sy * inset
    return (int(x0 + col * sx + mx), int(y0 + row * sy + my),
            max(2, int(sx - 2 * mx)), max(2, int(sy - 2 * my)))


def cell_roi(hsv, cfg, cell):
    """ภาพ HSV เฉพาะในช่องนั้น (ตัดตามขอบภาพให้ด้วย) คืน (roi, x0, y0)"""
    x, y, w, h = cell_rect(cfg, cell)
    H, W = hsv.shape[:2]
    x0, y0 = max(0, x), max(0, y)
    x1, y1 = min(W, x + w), min(H, y + h)
    if x1 <= x0 or y1 <= y0:
        return None, x0, y0
    return hsv[y0:y1, x0:x1], x0, y0


# ==========================================
# 🎯 ตรวจสี
# ==========================================
def color_mask(hsv, spec, grow=0):
    """มาสก์ของสีหนึ่ง grow = จำนวนครั้งที่ขยาย range (0 = ช่วงตั้งต้น)"""
    s_lo = max(30, spec["s"][0] - 15 * grow)
    v_lo = max(30, spec["v"][0] - 15 * grow)
    mask = None
    for h_lo, h_hi in spec["h"]:
        lo = np.array([max(0, h_lo - 3 * grow), s_lo, v_lo], dtype=np.uint8)
        hi = np.array([min(179, h_hi + 3 * grow), spec["s"][1], spec["v"][1]], dtype=np.uint8)
        part = cv.inRange(hsv, lo, hi)
        mask = part if mask is None else cv.bitwise_or(mask, part)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
    return cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)


def hue_gap(hue, ranges):
    """ระยะจาก H ไปยังช่วงที่ใกล้ที่สุด (H วนรอบที่ 180 จึงต้องคิดแบบวงกลม)"""
    best = 180.0
    for lo, hi in ranges:
        if lo <= hue <= hi:
            return 0.0
        for edge in (lo, hi):
            d = abs(hue - edge)
            best = min(best, min(d, 180 - d))
    return best


def mean_hue(roi):
    """H เฉลี่ยแบบวงกลมของพิกเซลที่ "มีสี" ที่สุดในช่อง (S สูงสุด 20% แรก)"""
    h, s, v = cv.split(roi)
    thr = float(np.percentile(s, 80))
    sel = (s >= max(40.0, thr)) & (v >= 40)
    if not np.any(sel):
        sel = np.ones_like(s, dtype=bool)
    ang = np.deg2rad(h[sel].astype(np.float32) * 2.0)     # H 0-179 -> องศา 0-358
    deg = math.degrees(math.atan2(float(np.sin(ang).mean()), float(np.cos(ang).mean()))) % 360
    return deg / 2.0


def mask_center(mask, rx, ry):
    """จุดกึ่งกลางของมาสก์ (แปลงกลับเป็นพิกัดในภาพเต็ม)"""
    m = cv.moments(mask, binaryImage=True)
    if not m["m00"]:
        return None
    return int(rx + m["m10"] / m["m00"]), int(ry + m["m01"] / m["m00"])


def classify_cell(hsv, cfg, cell):
    """สรุปสีของช่องหนึ่ง — ทางหลักคือ "เจอสีจริง" ส่วนที่เหลือเป็นระบบสำรองไล่ระดับลงมา

    1. นับพิกเซลของทุกสีในช่อง สีที่มากที่สุดชนะ (หลายสีปนกันตัดสินด้วยจำนวน)
       ถึงเกณฑ์ min_area -> จบ (sure=True, reason="found")
    2. ไม่ถึงเกณฑ์ -> ขยาย HSV range ทีละขั้นจนถึง expand_limit แล้วนับใหม่
       (sure=True, reason="grow")
    -- ต่อจากนี้คือ "หาสีไม่เจอ" แล้ว ถือเป็นการเดา (sure=False) --
    3. ยังพอมีพิกเซลของสีอยู่บ้าง (แค่ไม่ถึงเกณฑ์) -> เอาสีที่มีมากที่สุดเท่าที่เคยเห็น (weak)
    4. ไม่เหลือพิกเซลของสีไหนเลย -> เดาจาก H เฉลี่ยของช่อง เอาสีที่ใกล้ที่สุด (hue)

    คืน dict: color, count, grow, sure, reason, center, scores"""
    roi, rx, ry = cell_roi(hsv, cfg, cell)
    if roi is None or roi.size == 0:
        return {"color": None, "count": 0, "grow": 0, "sure": False,
                "reason": "empty", "center": None, "scores": {}}
    need = min_area_of(cfg)
    scores = {}
    seen = None          # หลักฐานที่ดีที่สุดเท่าที่เจอ (ยังไม่ถึงเกณฑ์)
    for grow in range(cfg.get("expand_limit", 6) + 1):
        masks = {c: color_mask(roi, spec, grow) for c, spec in cfg["colors"].items()}
        scores = {c: int(cv.countNonZero(m)) for c, m in masks.items()}
        best = max(scores, key=lambda c: scores[c])
        if scores[best] >= need:
            return {"color": best, "count": scores[best], "grow": grow, "sure": True,
                    "reason": "grow" if grow else "found",
                    "center": mask_center(masks[best], rx, ry), "scores": scores}
        if scores[best] > 0 and (seen is None or scores[best] > seen["count"]):
            seen = {"color": best, "count": scores[best], "grow": grow,
                    "center": mask_center(masks[best], rx, ry)}

    x, y, w, h = cell_rect(cfg, cell)
    if seen:      # (3) มีร่องรอยสีอยู่ แค่จางหรือเล็กเกินเกณฑ์
        return {"color": seen["color"], "count": seen["count"], "grow": seen["grow"],
                "sure": False, "reason": "weak", "center": seen["center"], "scores": scores}
    # (4) ไม่เหลืออะไรให้ยึดเลย เดาจากโทนสีเฉลี่ยของช่อง
    hue = mean_hue(roi)
    guess = min(cfg["colors"], key=lambda c: hue_gap(hue, cfg["colors"][c]["h"]))
    return {"color": guess, "count": 0, "grow": None, "sure": False, "reason": "hue",
            "center": (x + w // 2, y + h // 2), "scores": scores}


def analyse(frame, cfg):
    """ตรวจทุกช่อง (ยกเว้นช่องกลาง) คืน {ช่อง: ผลของช่องนั้น}"""
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    return {cell: classify_cell(hsv, cfg, cell) for cell in BLOCK_CELLS}


def detect(frame, cfg):
    """{ช่อง: สี} ครบ 8 ช่อง — ทุกช่องได้สีเสมอ"""
    return {cell: r["color"] for cell, r in analyse(frame, cfg).items() if r["color"]}


def summary(result, cfg):
    """ข้อความสรุปหนึ่งบรรทัดสำหรับแถบล่างของ UI (อังกฤษ)"""
    counts = {}
    for r in result.values():
        if r["color"]:
            counts[r["color"]] = counts.get(r["color"], 0) + 1
    grown = {c: r["grow"] for c, r in result.items() if r["sure"] and r["grow"]}
    guessed = [(c, r["reason"]) for c, r in result.items() if r["color"] and not r["sure"]]
    note = " ".join(f"{c}={counts.get(c, 0)}" for c in cfg["colors"])
    if grown:
        note += " | grow " + " ".join(f"{c}+{g}" for c, g in sorted(grown.items(), key=str))
    if guessed:
        note += " | guess " + " ".join(f"{c}({w})" for c, w in sorted(guessed, key=lambda t: str(t[0])))
    return note + f" | min_area={min_area_of(cfg)}px"


# ==========================================
# 📷 กล้อง
# ==========================================
def _device_key(src):
    """ใช้เทียบว่าเป็นกล้องตัวเดียวกันไหม (path กับ index อาจชี้ตัวเดียวกัน)"""
    path = src if isinstance(src, str) else f"/dev/video{src}"
    return os.path.realpath(path)


def camera_candidates(cfg):
    """ลำดับการลองเปิดกล้อง: ตัวที่ตั้งไว้ใน camera_index ก่อน
    แล้วค่อยไล่กล้องอื่นที่มีในเครื่อง — ถ้าไม่ได้เสียบกล้องเสริม ก็จะตกมาที่กล้องโน้ตบุ๊กเอง"""
    seen, out = set(), []
    for src in ([cfg["camera_index"]]
                + sorted(glob.glob("/dev/v4l/by-id/*-video-index0"))
                + list(range(4))):
        key = _device_key(src)
        if key not in seen:
            seen.add(key)
            out.append(src)
    return out


def _quiet_opencv(on):
    """ปิด warning ของ OpenCV ตอนไล่เปิดกล้องทีละตัว (ไม่งั้นรกเต็มจอ)"""
    try:
        from cv2.utils import logging as cvlog
        lv = cvlog.getLogLevel()
        cvlog.setLogLevel(cvlog.LOG_LEVEL_ERROR if on else lv)
        return lv
    except Exception:
        return None


def open_camera(cfg):
    """เปิดกล้องตัวแรกที่ใช้งานได้จริง (เปิดติด + อ่านภาพออก) คืน (cam, ที่มาที่ใช้จริง)"""
    want = cfg["camera_index"]
    prev, tried = _quiet_opencv(True), []
    try:
        for src in camera_candidates(cfg):
            cam = cv.VideoCapture(src, cv.CAP_V4L2) if isinstance(src, str) else cv.VideoCapture(src)
            if cam.isOpened() and cam.read()[0]:
                if _device_key(src) != _device_key(want):
                    print(f"⚠️ เปิดกล้องที่ตั้งไว้ ({want}) ไม่ได้ — ใช้ {src} แทน")
                return cam, src
            cam.release()
            tried.append(str(src))
    finally:
        if prev is not None:
            _quiet_opencv(False)
    raise CameraError("cannot open any camera, tried: " + ", ".join(tried))


def grab_frame(cfg):
    cam, src = open_camera(cfg)
    print(f"📷 ใช้กล้อง: {src}")
    try:
        frame = None
        for _ in range(max(1, cfg.get("warmup_frames", 25))):
            ok, f = cam.read()          # ทิ้งเฟรมแรกๆ ระหว่างกล้องปรับแสง
            if ok and f is not None:
                frame = f
        if frame is None:
            raise CameraError("cannot read frame from camera")
        return frame
    finally:
        cam.release()


def get_blocks():
    """API หลักที่ main_0.7 เรียกใช้ คืน {ช่อง: สี} ในมุมมองภาพกล้อง (ครบ 8 ช่อง)"""
    cfg = load_config()
    result = analyse(grab_frame(cfg), cfg)
    blocks = {cell: r["color"] for cell, r in result.items() if r["color"]}
    grown = {c: r["grow"] for c, r in result.items() if r["sure"] and r["grow"]}
    weak = [c for c, r in result.items() if r["reason"] == "weak"]
    hue = [c for c, r in result.items() if r["reason"] == "hue"]
    if grown:
        print("ℹ️ ต้องขยาย range: " + ", ".join(f"ช่อง {c}+{g}" for c, g in sorted(grown.items())))
    if weak:
        print("⚠️ สีจาง/เล็กกว่าเกณฑ์ ใช้สีที่มีมากที่สุดในช่องแทน: "
              + ", ".join(f"ช่อง {c}={blocks[c]}" for c in sorted(weak)))
    if hue:
        print("⚠️ ไม่เจอสีในช่องเลย เดาจากโทนสีเฉลี่ย (ไม่ชัวร์): "
              + ", ".join(f"ช่อง {c}={blocks[c]}" for c in sorted(hue)))
    return blocks


# ==========================================
# 🖥️ UI ตั้งกรอบ 3x3 (รันไฟล์นี้ตรงๆ)
# ==========================================
HELP = ["w/a/s/d = move grid", "+/- (or z/x) = zoom grid", "t/g = taller/shorter",
        "f/h = wider/narrower", "[ / ] = min area %", "m = mask overlay", "k = SAVE", "q = quit"]

GUESS_GRAY = (130, 130, 130)   # ชิปสี + จุดกึ่งกลางของช่องที่ "เดา" ใช้สีเทา ไม่ใช่สีจริง


def mask_overlay(frame, cfg):
    """ภาพจริงหรี่ลง 50% แล้วทาบมาสก์ของทุกสีด้วยสีประจำหมวดของมัน
    เห็นทีเดียวว่าแต่ละสีจับอะไรไปบ้าง และทับพื้นที่กันตรงไหน"""
    view = (frame * 0.5).astype(np.uint8)
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    for color, spec in cfg["colors"].items():
        view[color_mask(hsv, spec, 0) > 0] = DRAW_BGR.get(color, (255, 255, 255))
    return view


def draw_overlay(frame, cfg, result, note):
    x0, y0, w, h = grid_rect(cfg)
    sx, sy = w // 3, h // 3
    cv.rectangle(frame, (x0, y0), (x0 + w, y0 + h), (255, 255, 255), 2)
    for i in (1, 2):
        cv.line(frame, (x0 + i * sx, y0), (x0 + i * sx, y0 + h), (255, 255, 255), 1)
        cv.line(frame, (x0, y0 + i * sy), (x0 + w, y0 + i * sy), (255, 255, 255), 1)
    for (row, col), cell in CELL_AT.items():
        r = result.get(cell) or {}
        color, sure = r.get("color"), r.get("sure")
        bgr = DRAW_BGR.get(color, (200, 200, 200))
        # กรอบย่อยที่ใช้ตัดสินสีของช่องนั้นจริงๆ
        if cell != "c":
            cx, cy, cw, ch = cell_rect(cfg, cell)
            cv.rectangle(frame, (cx, cy), (cx + cw, cy + ch), (90, 90, 90), 1)
        # มุมซ้ายบนของช่อง: เลขช่อง
        cv.putText(frame, str(cell), (x0 + col * sx + 6, y0 + row * sy + 20),
                   cv.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2)
        # มุมขวาล่างของช่อง: สีที่สรุปได้ (มี ? ต่อท้าย = เดาเอา ไม่ได้เจอสีชัดๆ)
        bx, by = x0 + (col + 1) * sx - 10, y0 + (row + 1) * sy - 10
        if color:
            # เจอจริง = ชิปสีจริง | เดา = ชิปสีเทา แต่ตัวอักษรยังเป็นสีนั้นอยู่ ต่อท้ายด้วย (?)
            # จัดให้ขอบขวาของตัวอักษรชนขอบช่องพอดี ป้ายยาว "g(?)" จะได้ไม่ล้นออกนอกช่อง
            label = color if sure else color + "(?)"
            (tw, _), _ = cv.getTextSize(label, cv.FONT_HERSHEY_SIMPLEX, 0.55, 2)
            tx = bx - tw
            cv.putText(frame, label, (tx, by), cv.FONT_HERSHEY_SIMPLEX, 0.55, bgr, 2)
            chip = bgr if sure else GUESS_GRAY
            cv.rectangle(frame, (tx - 22, by - 15), (tx - 6, by + 1), chip, -1)
            cv.rectangle(frame, (tx - 22, by - 15), (tx - 6, by + 1), (255, 255, 255), 1)
        elif cell != "c":
            cv.putText(frame, "-", (bx - 8, by), cv.FONT_HERSHEY_SIMPLEX, 0.6, (120, 120, 120), 2)
    # จุดอ้างอิงกึ่งกลางของสีที่ชนะในแต่ละช่อง
    for cell, r in result.items():
        if not r.get("center") or not r.get("color"):
            continue
        px, py = r["center"]
        sure = r.get("sure")
        dot = DRAW_BGR.get(r["color"], (255, 255, 255)) if sure else GUESS_GRAY
        cv.drawMarker(frame, (px, py), (255, 255, 255) if sure else GUESS_GRAY,
                      cv.MARKER_CROSS, 14, 3 if sure else 1)
        cv.circle(frame, (px, py), 5, dot, -1)
        cv.circle(frame, (px, py), 5, (255, 255, 255), 1)
    for i, line in enumerate(HELP):
        cv.putText(frame, line, (10, 20 + 18 * i), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    cv.putText(frame, note, (10, frame.shape[0] - 12), cv.FONT_HERSHEY_SIMPLEX, 0.55,
               (255, 255, 255), 2)
    return frame


def setup():
    cfg = load_config()
    try:
        cam, src = open_camera(cfg)
    except CameraError as e:
        print(f"❌ {e}")
        return
    print(f"📷 ใช้กล้อง: {src}")
    if _device_key(src) != _device_key(cfg["camera_index"]):
        print("   ⚠️ ไม่ใช่กล้องที่ตั้งไว้ — กรอบที่จัดตอนนี้จะตรงกับกล้องตัวนี้เท่านั้น")
        print(f"   ถ้าจะใช้ตัวนี้ถาวร แก้ camera_index ใน camera_config.json เป็น {src!r}")
    mask_view = False     # True = ทาบมาสก์ของทุกสีบนภาพที่หรี่ลง 50%
    print("🖥️ จัดกรอบให้ตรงกับตาราง 3x3 แล้วกด k เพื่อบันทึก (q = ออก) — ข้อความบนหน้าต่างเป็นอังกฤษ")
    print("   ชิปสีเทา + (?) = ช่องนั้นหาสีไม่เจอ เลยใช้ระบบสำรองเดาให้ (ตัวอักษรยังเป็นสีที่เดาได้)")
    print("   กด m = ทาบมาสก์ของทุกสีลงบนภาพจริงที่หรี่ลง 50% เพื่อดูว่าแต่ละสีจับอะไรไปบ้าง")
    while True:
        ok, frame = cam.read()
        if not ok:
            print("❌ อ่านภาพจากกล้องไม่ได้")
            break
        result = analyse(frame, cfg)
        note = summary(result, cfg)

        if mask_view:
            view = mask_overlay(frame, cfg)
            note = "[mask] " + note
        else:
            view = frame.copy()
        cv.imshow("camara setup (3x3)", draw_overlay(view, cfg, result, note))

        key = cv.waitKey(30) & 0xFF
        f = cfg["frame"]
        f.setdefault("w", f.get("size", 300))
        f.setdefault("h", f.get("size", 300))
        if key in (ord("q"), 27):
            break
        elif key == ord("w"):
            f["cy"] -= 5
        elif key == ord("s"):
            f["cy"] += 5
        elif key == ord("a"):
            f["cx"] -= 5
        elif key == ord("d"):
            f["cx"] += 5
        elif key in (ord("+"), ord("="), ord("z")):      # ซูมเข้า (กรอบใหญ่ขึ้น)
            f["w"], f["h"] = f["w"] + 6, f["h"] + 6
        elif key in (ord("-"), ord("_"), ord("x")):      # ซูมออก (กรอบเล็กลง)
            f["w"], f["h"] = max(30, f["w"] - 6), max(30, f["h"] - 6)
        elif key == ord("t"):
            f["h"] += 6
        elif key == ord("g"):
            f["h"] = max(30, f["h"] - 6)
        elif key == ord("f"):
            f["w"] += 6
        elif key == ord("h"):
            f["w"] = max(30, f["w"] - 6)
        elif key == ord("["):
            cfg["min_area_ratio"] = round(max(0.002, cfg.get("min_area_ratio", 0.04) - 0.005), 4)
        elif key == ord("]"):
            cfg["min_area_ratio"] = round(min(0.5, cfg.get("min_area_ratio", 0.04) + 0.005), 4)
        elif key == ord("m"):
            mask_view = not mask_view
        elif key == ord("k"):
            save_config(cfg)
    cam.release()
    cv.destroyAllWindows()


if __name__ == "__main__":
    setup()
