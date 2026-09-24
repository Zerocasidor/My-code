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
import glob
import json
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
    # พื้นที่ต่ำสุดที่นับเป็นบล็อก คิดเป็น "สัดส่วนของ 1 ช่อง" ไม่ใช่พิกเซลตายตัว
    # กรอบใหญ่ขึ้น (ซูมเข้า / กล้องใกล้ขึ้น) บล็อกในภาพก็ใหญ่ขึ้นตาม เกณฑ์จึงโตตามไปเอง
    "min_area_ratio": 0.04,
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
    data = {}
    try:
        with open(CONFIG_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
        cfg.update({k: v for k, v in data.items() if k != "colors"})
        cfg["colors"].update(data.get("colors", {}))
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


def cell_size(cfg):
    """ขนาด 1 ช่องของตาราง 3x3 (พิกเซล)"""
    _, _, w, h = grid_rect(cfg)
    return w / 3.0, h / 3.0


def min_area_of(cfg):
    """พื้นที่ต่ำสุดที่นับเป็นบล็อก (พิกเซล) = สัดส่วนที่ตั้งไว้ x พื้นที่ 1 ช่อง"""
    cw, ch = cell_size(cfg)
    return max(50, int(cfg.get("min_area_ratio", 0.04) * cw * ch))


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
    """มุมซ้ายบน + กว้าง/สูง ของกรอบ 3x3"""
    f = cfg["frame"]
    w = f.get("w", f.get("size", 300))
    h = f.get("h", f.get("size", 300))
    return f["cx"] - w // 2, f["cy"] - h // 2, w, h


def cell_of(cfg, px, py):
    """แปลงพิกัดในภาพเป็นหมายเลขช่อง คืน None ถ้าอยู่นอกกรอบ 3x3 (ตัดทิ้งไม่นับ)"""
    x0, y0, w, h = grid_rect(cfg)
    if not (x0 <= px < x0 + w and y0 <= py < y0 + h):
        return None
    col = int((px - x0) * 3 // w)
    row = int((py - y0) * 3 // h)
    return CELL_AT[(row, col)]


def scan_color(hsv, cfg, color, grow=0):
    """สแกนสีหนึ่งที่ระดับการขยาย range = grow (ไม่บังคับว่าต้องเจอกี่ก้อน)
    คืน (ในกรอบ {ช่อง: (พื้นที่, จุดกึ่งกลาง)}, นอกกรอบ [จุดกึ่งกลาง, ...])"""
    mask = color_mask(hsv, cfg["colors"][color], grow)
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    min_area = min_area_of(cfg)
    inside, outside = {}, []
    for cnt in contours:
        area = cv.contourArea(cnt)
        if area < min_area:
            continue
        m = cv.moments(cnt)
        if m["m00"] == 0:
            continue
        px, py = int(m["m10"] / m["m00"]), int(m["m01"] / m["m00"])
        cell = cell_of(cfg, px, py)
        if cell is None or cell == "c":   # นอกกรอบ / ช่องกลาง -> ตัดทิ้ง
            outside.append((px, py))
            continue
        if area > inside.get(cell, (0,))[0]:
            inside[cell] = (area, (px, py))
    return inside, outside


def preview(frame, cfg):
    """ตรวจแบบหลวมสำหรับ UI ตั้งกรอบ — ไม่บังคับว่าต้องเจอสีละ 2 ก้อน
    เจอกี่ก้อนก็โชว์เท่านั้น จะได้เห็นว่ากล้อง "เห็น" สีนั้นแล้วหรือยัง
    และก้อนที่ตกนอกกรอบก็โชว์ด้วย (ของจริงใน get_blocks จะถูกตัดทิ้ง)
    คืน (blocks, centers, outside, note)"""
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    blocks, areas, centers, outside, counts = {}, {}, {}, [], []
    for color in cfg["colors"]:
        found, out = scan_color(hsv, cfg, color, 0)
        counts.append((color, len(found)))
        outside += [(color, p) for p in out]
        for cell, (area, center) in found.items():
            if area <= areas.get(cell, 0):
                continue          # ช่องเดียวกันเจอ 2 สี -> เอาก้อนที่ใหญ่กว่า
            blocks[cell], areas[cell], centers[cell] = color, area, center
    ready = all(n == 2 for _, n in counts)
    note = ("READY " if ready else "found ") + " ".join(f"{c}={n}" for c, n in counts)
    if not ready:
        note += " (need 2 each)"
    if outside:
        note += f" | {len(outside)} outside grid"
    return blocks, centers, outside, (
        note + f" | min_area={min_area_of(cfg)}px"
        f" ({cfg.get('min_area_ratio', 0.04) * 100:.1f}% of cell)")


def find_color(hsv, cfg, color):
    """หาบล็อกของสีหนึ่งให้ได้ 2 ก้อน ขยาย range ทีละขั้นจนกว่าจะเจอ
    คืน [(ช่อง, พื้นที่, จุดกึ่งกลาง), ...] 2 ตัว หรือโยน CameraError"""
    name = COLOR_NAMES.get(color, color)
    found = {}
    for grow in range(cfg["expand_limit"] + 1):
        found, _ = scan_color(hsv, cfg, color, grow)
        if len(found) > 2:
            raise CameraError(f"over color block: {color} ({name}) found {len(found)} "
                              f"at cells {sorted(found)} (need 2)")
        if len(found) == 2:
            return [(cell, area, center) for cell, (area, center) in found.items()], grow
    raise CameraError(f"no color block: {color} ({name}) found {len(found)} "
                      f"after {cfg['expand_limit']} range expansions (need 2)")


def detect(frame, cfg, grown=None, centers=None):
    """ตรวจทุกสีจากภาพหนึ่งเฟรม คืน {ช่อง: สี} ทั้งหมด 8 ช่อง
    grown   = dict รับว่าสีไหนต้องขยาย range กี่ขั้น
    centers = dict รับจุดกึ่งกลางของแต่ละก้อน {ช่อง: (x, y)} ไว้วาดจุดอ้างอิง"""
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    blocks, areas = {}, {}
    for color in cfg["colors"]:
        cells, grow = find_color(hsv, cfg, color)
        if grow and grown is not None:
            grown[color] = grow
        for cell, area, center in cells:
            if cell in blocks and areas[cell] >= area:
                raise CameraError(f"cell {cell}: both {blocks[cell]} and {color} detected")
            blocks[cell] = color
            areas[cell] = area
            if centers is not None:
                centers[cell] = center
    return blocks


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
    """API หลักที่ main_0.7 เรียกใช้ คืน {ช่อง: สี} ในมุมมองภาพกล้อง"""
    cfg = load_config()
    grown = {}
    blocks = detect(grab_frame(cfg), cfg, grown)
    if grown:
        print("ℹ️ ต้องขยาย range: " + ", ".join(f"{c}+{n}" for c, n in grown.items()))
    return blocks


# ==========================================
# 🖥️ UI ตั้งกรอบ 3x3 (รันไฟล์นี้ตรงๆ)
# ==========================================
HELP = ["w/a/s/d = move grid", "+/- (or z/x) = zoom grid", "t/g = taller/shorter",
        "f/h = wider/narrower", "[ / ] = min area %", "m = mask view", "k = SAVE", "q = quit"]


def draw_overlay(frame, cfg, blocks, note, centers=None, outside=None):
    x0, y0, w, h = grid_rect(cfg)
    sx, sy = w // 3, h // 3
    cv.rectangle(frame, (x0, y0), (x0 + w, y0 + h), (255, 255, 255), 2)
    for i in (1, 2):
        cv.line(frame, (x0 + i * sx, y0), (x0 + i * sx, y0 + h), (255, 255, 255), 1)
        cv.line(frame, (x0, y0 + i * sy), (x0 + w, y0 + i * sy), (255, 255, 255), 1)
    for (row, col), cell in CELL_AT.items():
        color = blocks.get(cell)
        bgr = DRAW_BGR.get(color, (200, 200, 200))
        # มุมซ้ายบนของช่อง: เลขช่อง
        cv.putText(frame, str(cell), (x0 + col * sx + 6, y0 + row * sy + 20),
                   cv.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2)
        # มุมขวาล่างของช่อง: สีที่คิดว่าอยู่ในช่องนี้ (ตัวอักษร + สี่เหลี่ยมสีนั้น)
        bx, by = x0 + (col + 1) * sx - 10, y0 + (row + 1) * sy - 10
        if color:
            cv.rectangle(frame, (bx - 26, by - 16), (bx - 10, by), bgr, -1)
            cv.rectangle(frame, (bx - 26, by - 16), (bx - 10, by), (255, 255, 255), 1)
            cv.putText(frame, color, (bx - 8, by), cv.FONT_HERSHEY_SIMPLEX, 0.6, bgr, 2)
        else:
            cv.putText(frame, "-", (bx - 8, by), cv.FONT_HERSHEY_SIMPLEX, 0.6, (120, 120, 120), 2)
    # จุดอ้างอิงกึ่งกลางของแต่ละก้อนที่เจอ
    for cell, (px, py) in (centers or {}).items():
        bgr = DRAW_BGR.get(blocks.get(cell), (255, 255, 255))
        cv.drawMarker(frame, (px, py), (255, 255, 255), cv.MARKER_CROSS, 14, 3)
        cv.circle(frame, (px, py), 5, bgr, -1)
        cv.circle(frame, (px, py), 5, (255, 255, 255), 1)
    # ก้อนที่เจอแต่อยู่นอกกรอบ 3x3 (ของจริงจะถูกตัดทิ้ง) โชว์ให้เห็นว่ากล้องเห็นแต่กรอบไม่ครอบ
    for color, (px, py) in (outside or []):
        bgr = DRAW_BGR.get(color, (150, 150, 150))
        cv.drawMarker(frame, (px, py), bgr, cv.MARKER_TILTED_CROSS, 16, 2)
        cv.putText(frame, f"{color} out", (px + 10, py + 4),
                   cv.FONT_HERSHEY_SIMPLEX, 0.45, bgr, 1)
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
    mask_view = None      # None = ภาพปกติ, หรือชื่อสีเพื่อดูมาสก์
    colors = list(cfg["colors"])
    print("🖥️ จัดกรอบให้ตรงกับตาราง 3x3 แล้วกด k เพื่อบันทึก (q = ออก) — ข้อความบนหน้าต่างเป็นอังกฤษ")
    while True:
        ok, frame = cam.read()
        if not ok:
            print("❌ อ่านภาพจากกล้องไม่ได้")
            break
        # ใช้ preview (หลวม) ไม่ใช่ detect (เข้มงวด) เพราะระหว่างจัดกรอบ
        # มักยังวางบล็อกไม่ครบสีละ 2 ก้อน ถ้าใช้ detect จะโยน error แล้วไม่โชว์อะไรเลย
        blocks, centers, outside, note = preview(frame, cfg)

        if mask_view:
            hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
            view = cv.cvtColor(color_mask(hsv, cfg["colors"][mask_view], 0), cv.COLOR_GRAY2BGR)
            note = f"[mask {mask_view}] " + note
        else:
            view = frame.copy()
        cv.imshow("camara setup (3x3)",
                  draw_overlay(view, cfg, blocks, note, centers, outside))

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
            idx = -1 if mask_view is None else colors.index(mask_view)
            mask_view = colors[idx + 1] if idx + 1 < len(colors) else None
        elif key == ord("k"):
            save_config(cfg)
    cam.release()
    cv.destroyAllWindows()


if __name__ == "__main__":
    setup()
