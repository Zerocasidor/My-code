"""camara_1.0 — ตรวจสีบล็อกในตาราง 3x3 แล้วส่งให้ฝั่ง low-level (main_0.7)

เรียกจากโปรแกรมอื่น:  get_blocks() -> {ช่อง: สี} เช่น {1: "g", 3: "r", ...}
ตั้งค่ากรอบ/สี (UI):   python3 camara_1.0.py

หมายเลขช่องเป็นมุมมอง "ภาพกล้อง" เสมอ (ซ้าย->ขวา, บน->ล่าง):
    1 2 3
    4 c 5
    6 7 8
ฝั่ง main_0.7 เป็นคนกลับด้าน 180° เองถ้าตั้ง flip_camera ไว้
"""

import os
import json
import cv2 as cv
import numpy as np

CONFIG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "camera_config.json")

COLOR_NAMES = {"g": "เขียว", "r": "แดง", "y": "เหลือง", "b": "ฟ้า"}
DRAW_BGR = {"g": (0, 220, 0), "r": (0, 0, 255), "y": (0, 220, 220), "b": (255, 150, 0)}

# ช่วง HSV ตั้งต้น (OpenCV: H 0-179, S/V 0-255) แดงมี 2 ช่วงเพราะ H วนรอบ 0
DEFAULT_CONFIG = {
    "camera_index": 0,
    "frame": {"cx": 320, "cy": 240, "size": 300},  # กรอบ 3x3 (สี่เหลี่ยมจัตุรัส)
    "min_area": 400,        # พื้นที่ต่ำสุดที่นับเป็นบล็อก (พิกเซล)
    "expand_limit": 6,      # ขยาย range ได้กี่ครั้งก่อนจะฟ้อง no color block
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


class CameraError(Exception):
    """ตรวจสีไม่สำเร็จ — ฝั่ง main จะจับ error นี้แล้วให้กรอกลำดับเอง"""


def load_config():
    cfg = json.loads(json.dumps(DEFAULT_CONFIG))  # deep copy
    try:
        with open(CONFIG_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
        cfg.update({k: v for k, v in data.items() if k != "colors"})
        cfg["colors"].update(data.get("colors", {}))
    except FileNotFoundError:
        pass
    except Exception as e:
        print(f"⚠️ อ่าน camera_config.json ไม่ได้ ({e}) ใช้ค่าเริ่มต้น")
    return cfg


def save_config(cfg):
    try:
        with open(CONFIG_FILE, "w", encoding="utf-8") as f:
            json.dump(cfg, f, indent=4, ensure_ascii=False)
        print("💾 บันทึก camera_config.json แล้ว")
    except Exception as e:
        print(f"❌ บันทึกไม่สำเร็จ: {e}")


# ==========================================
# 🎯 ตรวจจับสี
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


def grid_rect(cfg):
    """มุมซ้ายบนและขนาดของกรอบ 3x3"""
    f = cfg["frame"]
    half = f["size"] // 2
    return f["cx"] - half, f["cy"] - half, f["size"]


def cell_of(cfg, px, py):
    """แปลงพิกัดในภาพเป็นหมายเลขช่อง คืน None ถ้าอยู่นอกกรอบ 3x3 (ตัดทิ้งไม่นับ)"""
    x0, y0, size = grid_rect(cfg)
    if not (x0 <= px < x0 + size and y0 <= py < y0 + size):
        return None
    col = int((px - x0) * 3 // size)
    row = int((py - y0) * 3 // size)
    return CELL_AT[(row, col)]


def find_color(hsv, cfg, color):
    """หาบล็อกของสีหนึ่งให้ได้ 2 ก้อน ขยาย range ทีละขั้นจนกว่าจะเจอ
    คืน [(ช่อง, พื้นที่), ...] 2 ตัว หรือโยน CameraError"""
    spec = cfg["colors"][color]
    name = COLOR_NAMES.get(color, color)
    for grow in range(cfg["expand_limit"] + 1):
        mask = color_mask(hsv, spec, grow)
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        found = {}
        for cnt in contours:
            area = cv.contourArea(cnt)
            if area < cfg["min_area"]:
                continue
            m = cv.moments(cnt)
            if m["m00"] == 0:
                continue
            cell = cell_of(cfg, m["m10"] / m["m00"], m["m01"] / m["m00"])
            if cell is None or cell == "c":   # นอกกรอบ / ช่องกลาง -> ตัดทิ้ง
                continue
            if area > found.get(cell, (0,))[0]:
                found[cell] = (area, grow)
        if len(found) > 2:
            raise CameraError(f"over color block: {color} ({name}) เจอ {len(found)} ก้อน "
                              f"ที่ช่อง {sorted(found)} (ควรมี 2)")
        if len(found) == 2:
            return [(cell, v[0]) for cell, v in found.items()], grow
    raise CameraError(f"no color block: {color} ({name}) เจอ {len(found)} ก้อน "
                      f"หลังขยาย range {cfg['expand_limit']} ครั้งแล้ว (ควรมี 2)")


def detect(frame, cfg):
    """ตรวจทุกสีจากภาพหนึ่งเฟรม คืน {ช่อง: สี} ทั้งหมด 8 ช่อง"""
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    blocks, areas = {}, {}
    for color in cfg["colors"]:
        cells, grow = find_color(hsv, cfg, color)
        if grow:
            print(f"ℹ️ สี {color} ต้องขยาย range {grow} ขั้นถึงจะเจอครบ 2 ก้อน")
        for cell, area in cells:
            if cell in blocks and areas[cell] >= area:
                raise CameraError(f"ช่อง {cell} เจอทั้งสี {blocks[cell]} และ {color} ซ้อนกัน")
            blocks[cell] = color
            areas[cell] = area
    return blocks


def grab_frame(cfg):
    cam = cv.VideoCapture(cfg["camera_index"])
    if not cam.isOpened():
        raise CameraError(f"เปิดกล้อง index {cfg['camera_index']} ไม่ได้")
    try:
        for _ in range(5):        # ทิ้งเฟรมแรกๆ ให้กล้องปรับแสงก่อน
            ok, frame = cam.read()
        if not ok:
            raise CameraError("อ่านภาพจากกล้องไม่ได้")
        return frame
    finally:
        cam.release()


def get_blocks():
    """API หลักที่ main_0.7 เรียกใช้ คืน {ช่อง: สี} ในมุมมองภาพกล้อง"""
    cfg = load_config()
    return detect(grab_frame(cfg), cfg)


# ==========================================
# 🖥️ UI ตั้งกรอบ 3x3 (รันไฟล์นี้ตรงๆ)
# ==========================================
HELP = ["w/a/s/d = เลื่อนกรอบ", "+/- = ซูมกรอบเข้า/ออก", "[ / ] = min_area",
        "m = สลับดูมาสก์รายสี", "k = บันทึกค่า", "q = ออก"]


def draw_overlay(frame, cfg, blocks, note):
    x0, y0, size = grid_rect(cfg)
    step = size // 3
    cv.rectangle(frame, (x0, y0), (x0 + size, y0 + size), (255, 255, 255), 2)
    for i in (1, 2):
        cv.line(frame, (x0 + i * step, y0), (x0 + i * step, y0 + size), (255, 255, 255), 1)
        cv.line(frame, (x0, y0 + i * step), (x0 + size, y0 + i * step), (255, 255, 255), 1)
    for (row, col), cell in CELL_AT.items():
        cx, cy = x0 + col * step + 6, y0 + row * step + 20
        color = blocks.get(cell)
        label = f"{cell}:{color}" if color else str(cell)
        cv.putText(frame, label, (cx, cy), cv.FONT_HERSHEY_SIMPLEX, 0.6,
                   DRAW_BGR.get(color, (200, 200, 200)), 2)
    for i, line in enumerate(HELP):
        cv.putText(frame, line, (10, 20 + 18 * i), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    cv.putText(frame, note, (10, frame.shape[0] - 12), cv.FONT_HERSHEY_SIMPLEX, 0.55,
               (255, 255, 255), 2)
    return frame


def setup():
    cfg = load_config()
    cam = cv.VideoCapture(cfg["camera_index"])
    if not cam.isOpened():
        print(f"❌ เปิดกล้อง index {cfg['camera_index']} ไม่ได้")
        return
    mask_view = None      # None = ภาพปกติ, หรือชื่อสีเพื่อดูมาสก์
    colors = list(cfg["colors"])
    print("🖥️ ตั้งกรอบให้ตรงกับตาราง 3x3 แล้วกด k เพื่อบันทึก (q = ออก)")
    while True:
        ok, frame = cam.read()
        if not ok:
            print("❌ อ่านภาพจากกล้องไม่ได้")
            break
        try:
            blocks = detect(frame, cfg)
            note = "OK: " + " ".join(f"{c}={blocks[c]}" for c in sorted(blocks))
        except CameraError as e:
            blocks, note = {}, str(e)

        if mask_view:
            hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
            view = cv.cvtColor(color_mask(hsv, cfg["colors"][mask_view], 0), cv.COLOR_GRAY2BGR)
            note = f"[mask {mask_view}] " + note
        else:
            view = frame.copy()
        cv.imshow("camara setup (3x3)", draw_overlay(view, cfg, blocks, note))

        key = cv.waitKey(30) & 0xFF
        f = cfg["frame"]
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
        elif key in (ord("+"), ord("=")):
            f["size"] += 6
        elif key in (ord("-"), ord("_")):
            f["size"] = max(30, f["size"] - 6)
        elif key == ord("["):
            cfg["min_area"] = max(50, cfg["min_area"] - 50)
        elif key == ord("]"):
            cfg["min_area"] += 50
        elif key == ord("m"):
            idx = -1 if mask_view is None else colors.index(mask_view)
            mask_view = colors[idx + 1] if idx + 1 < len(colors) else None
        elif key == ord("k"):
            save_config(cfg)
    cam.release()
    cv.destroyAllWindows()


if __name__ == "__main__":
    setup()
