"""camara_2.0 - flexible 3x3 block colour detection.

From another program:  get_blocks() -> {cell: color}, e.g. {1: "g", 2: "r", ...}, all 8 cells
Frame/colour setup UI:  python3 camara_2.0.py

Difference from 1.0: it looks at one CELL at a time instead of one COLOUR at a time,
so a colour may repeat any number of times and a cell with mixed colours is decided
by pixel count. No over/no-color-block errors any more.

Fallback ladder, used only when a colour really cannot be found:
    1. colour passes min_area at the normal range        -> sure  (reason "found")
    2. passes only after widening the HSV range          -> sure  (reason "grow")
    3. never passes but some pixels remain               -> guess (reason "weak")
    4. no pixels at all, nearest colour by mean hue      -> guess (reason "hue")
Guesses are marked in the window and reported at run time.

Cell numbers are always in CAMERA view (left->right, top->bottom):
    1 2 3
    4 c 5
    6 7 8
main flips the view 180 itself when flip_camera is set.
"""

import os
import glob
import json
import math
import cv2 as cv
import numpy as np

CONFIG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "camera_config.json")

# OpenCV putText cannot draw Thai, so all on-window text is English
COLOR_NAMES = {"g": "green", "r": "red", "y": "yellow", "b": "blue"}
DRAW_BGR = {"g": (0, 220, 0), "r": (0, 0, 255), "y": (0, 220, 220), "b": (255, 150, 0)}

# Default HSV ranges (OpenCV: H 0-179, S/V 0-255). Red needs 2 ranges, H wraps at 0.
DEFAULT_CONFIG = {
    # Preferred camera: index or path. A /dev/v4l/by-id/ path sticks to one device.
    # If it cannot be opened, the other cameras are tried (e.g. the built-in one).
    "camera_index": 0,
    "warmup_frames": 25,    # drop the first frames while exposure settles
    "frame": {"cx": 320, "cy": 240, "w": 300, "h": 300},  # 3x3 frame
    # Smallest area counted as "colour found", as a fraction of one cell,
    # so the threshold scales with the frame instead of being fixed pixels.
    "min_area_ratio": 0.04,
    "cell_inset": 0.15,     # shrink each cell edge to keep neighbours and grid lines out
    "expand_limit": 6,      # range-widening steps before guessing
    "rotation": 0,          # cell numbering turned 90 deg CW this many times (r in the UI)
    "colors": {
        "g": {"h": [[40, 85]], "s": [80, 255], "v": [60, 255]},
        "r": {"h": [[0, 10], [170, 179]], "s": [90, 255], "v": [60, 255]},
        "y": {"h": [[20, 35]], "s": [90, 255], "v": [80, 255]},
        "b": {"h": [[90, 130]], "s": [80, 255], "v": [60, 255]},
    },
}

# (row, col) in camera view; the centre (1,1) is the tower spot, not a block
CELL_AT = {(0, 0): 1, (0, 1): 2, (0, 2): 3,
           (1, 0): 4, (1, 1): "c", (1, 2): 5,
           (2, 0): 6, (2, 1): 7, (2, 2): 8}
CELL_POS = {cell: pos for pos, cell in CELL_AT.items()}
BLOCK_CELLS = [c for c in (1, 2, 3, 4, 5, 6, 7, 8)]


def cell_at(cfg):
    """(row, col) -> cell number, with the numbering turned 90 deg CW `rotation` times.
    Only the labels move; the boxes on screen stay where they are."""
    out = {}
    for (row, col), cell in CELL_AT.items():
        for _ in range(cfg.get("rotation", 0) % 4):
            row, col = col, 2 - row
        out[(row, col)] = cell
    return out


def cell_pos(cfg):
    """cell number -> (row, col), honouring rotation."""
    return {cell: pos for pos, cell in cell_at(cfg).items()}


class CameraError(Exception):
    """Camera could not be opened or read. 2.0 never raises for colours."""


def load_config():
    cfg = json.loads(json.dumps(DEFAULT_CONFIG))  # deep copy
    data = {}
    try:
        with open(CONFIG_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
        cfg.update({k: v for k, v in data.items() if k != "colors"})
        # merge per colour, so a file that sets only h keeps s/v
        for name, spec in (data.get("colors") or {}).items():
            if isinstance(spec, dict):
                cfg["colors"].setdefault(name, {}).update(spec)
        for name in [n for n, spec in cfg["colors"].items()
                     if not all(k in spec for k in ("h", "s", "v"))]:
            print(f"! colour '{name}' in camera_config.json has no h/s/v, skipped")
            cfg["colors"].pop(name)
    except FileNotFoundError:
        pass
    except Exception as e:
        print(f"! cannot read camera_config.json ({e}), using defaults")
    # legacy min_area in pixels -> ratio, using the frame saved with it
    old = cfg.pop("min_area", None)
    if old is not None and "min_area_ratio" not in data:
        cw, ch = cell_size(cfg)
        cfg["min_area_ratio"] = round(old / (cw * ch), 4)
        print(f"min_area {old} px -> min_area_ratio {cfg['min_area_ratio']} "
              f"(cell was {cw:.0f}x{ch:.0f} px)")
    return cfg


def save_config(cfg):
    try:
        with open(CONFIG_FILE, "w", encoding="utf-8") as f:
            json.dump(cfg, f, indent=4, ensure_ascii=False)
        print("saved camera_config.json")
    except Exception as e:
        print(f"! save failed: {e}")


# ==========================================
# grid
# ==========================================
def grid_rect(cfg):
    """Top-left corner plus width/height of the 3x3 frame."""
    f = cfg["frame"]
    w = f.get("w", f.get("size", 300))
    h = f.get("h", f.get("size", 300))
    return f["cx"] - w // 2, f["cy"] - h // 2, w, h


def cell_size(cfg):
    """Size of one cell in pixels."""
    _, _, w, h = grid_rect(cfg)
    return w / 3.0, h / 3.0


def min_area_of(cfg):
    """Minimum area in pixels = ratio x cell area."""
    cw, ch = cell_size(cfg)
    return max(50, int(cfg.get("min_area_ratio", 0.04) * cw * ch))


def cell_rect(cfg, cell):
    """One cell box, shrunk by cell_inset to keep neighbours out."""
    x0, y0, w, h = grid_rect(cfg)
    row, col = cell_pos(cfg)[cell]
    sx, sy = w / 3.0, h / 3.0
    inset = cfg.get("cell_inset", 0.15)
    mx, my = sx * inset, sy * inset
    return (int(x0 + col * sx + mx), int(y0 + row * sy + my),
            max(2, int(sx - 2 * mx)), max(2, int(sy - 2 * my)))


def cell_roi(hsv, cfg, cell):
    """HSV crop of one cell, clipped to the image. Returns (roi, x0, y0)."""
    x, y, w, h = cell_rect(cfg, cell)
    H, W = hsv.shape[:2]
    x0, y0 = max(0, x), max(0, y)
    x1, y1 = min(W, x + w), min(H, y + h)
    if x1 <= x0 or y1 <= y0:
        return None, x0, y0
    return hsv[y0:y1, x0:x1], x0, y0


# ==========================================
# colour detection
# ==========================================
def color_mask(hsv, spec, grow=0):
    """Mask of one colour; grow = range-widening steps (0 = as configured)."""
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
    """Distance from a hue to the nearest range, wrapping at 180."""
    best = 180.0
    for lo, hi in ranges:
        if lo <= hue <= hi:
            return 0.0
        for edge in (lo, hi):
            d = abs(hue - edge)
            best = min(best, min(d, 180 - d))
    return best


def mean_hue(roi):
    """Circular mean hue of the most saturated 20% of the cell."""
    h, s, v = cv.split(roi)
    thr = float(np.percentile(s, 80))
    sel = (s >= max(40.0, thr)) & (v >= 40)
    if not np.any(sel):
        sel = np.ones_like(s, dtype=bool)
    ang = np.deg2rad(h[sel].astype(np.float32) * 2.0)     # H 0-179 -> degrees 0-358
    deg = math.degrees(math.atan2(float(np.sin(ang).mean()), float(np.cos(ang).mean()))) % 360
    return deg / 2.0


def mask_center(mask, rx, ry):
    """Mask centroid, mapped back to full-image coordinates."""
    m = cv.moments(mask, binaryImage=True)
    if not m["m00"]:
        return None
    return int(rx + m["m10"] / m["m00"]), int(ry + m["m01"] / m["m00"])


def classify_cell(hsv, cfg, cell):
    """Decide the colour of one cell: most pixels wins; widen the range if nothing
    passes min_area; then fall back to the best partial evidence, then to mean hue.
    Returns dict: color, count, grow, sure, reason, center, scores."""
    roi, rx, ry = cell_roi(hsv, cfg, cell)
    if roi is None or roi.size == 0:
        return {"color": None, "count": 0, "grow": 0, "sure": False,
                "reason": "empty", "center": None, "scores": {}}
    need = min_area_of(cfg)
    scores = {}
    seen = None          # best evidence so far, still below min_area
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
    if seen:      # (3) some pixels of a colour, just faint or small
        return {"color": seen["color"], "count": seen["count"], "grow": seen["grow"],
                "sure": False, "reason": "weak", "center": seen["center"], "scores": scores}
    # (4) nothing to go on, guess from the cell mean hue
    hue = mean_hue(roi)
    guess = min(cfg["colors"], key=lambda c: hue_gap(hue, cfg["colors"][c]["h"]))
    return {"color": guess, "count": 0, "grow": None, "sure": False, "reason": "hue",
            "center": (x + w // 2, y + h // 2), "scores": scores}


def analyse(frame, cfg):
    """Every cell except the centre. Returns {cell: result}."""
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    return {cell: classify_cell(hsv, cfg, cell) for cell in BLOCK_CELLS}


def detect(frame, cfg):
    """{cell: color} for all 8 cells."""
    return {cell: r["color"] for cell, r in analyse(frame, cfg).items() if r["color"]}


def summary(result, cfg):
    """One-line status for the bottom of the window."""
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
    rot = cfg.get("rotation", 0) % 4
    return note + (f" | rot {rot * 90}" if rot else "") + f" | min_area={min_area_of(cfg)}px"


# ==========================================
# camera
# ==========================================
def _device_key(src):
    """Key for comparing devices; a path and an index may be the same camera."""
    path = src if isinstance(src, str) else f"/dev/video{src}"
    return os.path.realpath(path)


def camera_candidates(cfg):
    """camera_index first, then the other cameras on the machine, so an
    unplugged USB camera falls back to the built-in one."""
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
    """Silence OpenCV warnings while probing cameras one by one."""
    try:
        from cv2.utils import logging as cvlog
        lv = cvlog.getLogLevel()
        cvlog.setLogLevel(cvlog.LOG_LEVEL_ERROR if on else lv)
        return lv
    except Exception:
        return None


def open_camera(cfg):
    """First camera that opens AND reads a frame. Returns (cam, source)."""
    want = cfg["camera_index"]
    prev, tried = _quiet_opencv(True), []
    try:
        for src in camera_candidates(cfg):
            cam = cv.VideoCapture(src, cv.CAP_V4L2) if isinstance(src, str) else cv.VideoCapture(src)
            if cam.isOpened() and cam.read()[0]:
                if _device_key(src) != _device_key(want):
                    print(f"! cannot open configured camera ({want}), using {src}")
                return cam, src
            cam.release()
            tried.append(str(src))
    finally:
        if prev is not None:
            _quiet_opencv(False)
    raise CameraError("cannot open any camera, tried: " + ", ".join(tried))


def grab_frame(cfg):
    cam, src = open_camera(cfg)
    print(f"camera: {src}")
    try:
        frame = None
        for _ in range(max(1, cfg.get("warmup_frames", 25))):
            ok, f = cam.read()          # drop frames while exposure settles
            if ok and f is not None:
                frame = f
        if frame is None:
            raise CameraError("cannot read frame from camera")
        return frame
    finally:
        cam.release()


def get_blocks():
    """Main entry point for the robot program. {cell: color} in camera view."""
    cfg = load_config()
    result = analyse(grab_frame(cfg), cfg)
    blocks = {cell: r["color"] for cell, r in result.items() if r["color"]}
    grown = {c: r["grow"] for c, r in result.items() if r["sure"] and r["grow"]}
    weak = [c for c, r in result.items() if r["reason"] == "weak"]
    hue = [c for c, r in result.items() if r["reason"] == "hue"]
    if grown:
        print("range widened: " + ", ".join(f"cell {c}+{g}" for c, g in sorted(grown.items())))
    if weak:
        print("! faint/small, used the most common colour: "
              + ", ".join(f"cell {c}={blocks[c]}" for c in sorted(weak)))
    if hue:
        print("! no colour found, guessed from mean hue: "
              + ", ".join(f"cell {c}={blocks[c]}" for c in sorted(hue)))
    return blocks


# ==========================================
# setup window (run this file directly)
# ==========================================
HELP = ["w/a/s/d = move grid", "+/- (or z/x) = zoom grid", "t/g = taller/shorter",
        "f/h = wider/narrower", "[ / ] = min area %", "r = rotate numbers 90",
        "m = mask overlay", "k = SAVE", "q = quit"]

GUESS_GRAY = (130, 130, 130)   # chip and dot colour for guessed cells


def mask_overlay(frame, cfg):
    """Frame at 50% brightness with every colour mask painted in its own colour."""
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
    for (row, col), cell in cell_at(cfg).items():
        r = result.get(cell) or {}
        color, sure = r.get("color"), r.get("sure")
        bgr = DRAW_BGR.get(color, (200, 200, 200))
        # the box actually used to decide the cell
        if cell != "c":
            cx, cy, cw, ch = cell_rect(cfg, cell)
            cv.rectangle(frame, (cx, cy), (cx + cw, cy + ch), (90, 90, 90), 1)
        # top-left: cell number
        cv.putText(frame, str(cell), (x0 + col * sx + 6, y0 + row * sy + 20),
                   cv.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2)
        # bottom-right: decided colour, (?) means guessed
        bx, by = x0 + (col + 1) * sx - 10, y0 + (row + 1) * sy - 10
        if color:
            # found = real chip colour, guessed = grey chip but the letter keeps its colour
            # right-align the label so "g(?)" stays inside the cell
            label = color if sure else color + "(?)"
            (tw, _), _ = cv.getTextSize(label, cv.FONT_HERSHEY_SIMPLEX, 0.55, 2)
            tx = bx - tw
            cv.putText(frame, label, (tx, by), cv.FONT_HERSHEY_SIMPLEX, 0.55, bgr, 2)
            chip = bgr if sure else GUESS_GRAY
            cv.rectangle(frame, (tx - 22, by - 15), (tx - 6, by + 1), chip, -1)
            cv.rectangle(frame, (tx - 22, by - 15), (tx - 6, by + 1), (255, 255, 255), 1)
        elif cell != "c":
            cv.putText(frame, "-", (bx - 8, by), cv.FONT_HERSHEY_SIMPLEX, 0.6, (120, 120, 120), 2)
    # centroid of the winning colour in each cell
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
        print(f"! {e}")
        return
    print(f"camera: {src}")
    if _device_key(src) != _device_key(cfg["camera_index"]):
        print("  ! not the configured camera - this frame fits THIS camera only")
        print(f"  to keep it, set camera_index in camera_config.json to {src!r}")
    mask_view = False     # True = mask overlay on a dimmed frame
    print("align the 3x3 frame, press k to SAVE, q to quit")
    print("  grey chip + (?) = that cell was guessed, not found")
    print("  m = paint every colour mask over a dimmed frame")
    while True:
        ok, frame = cam.read()
        if not ok:
            print("! cannot read a frame")
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
        elif key in (ord("+"), ord("="), ord("z")):      # zoom in (bigger frame)
            f["w"], f["h"] = f["w"] + 6, f["h"] + 6
        elif key in (ord("-"), ord("_"), ord("x")):      # zoom out (smaller frame)
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
        elif key == ord("r"):
            cfg["rotation"] = (cfg.get("rotation", 0) + 1) % 4
        elif key == ord("m"):
            mask_view = not mask_view
        elif key == ord("k"):
            save_config(cfg)
    cam.release()
    cv.destroyAllWindows()


if __name__ == "__main__":
    setup()
