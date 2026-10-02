"""main_1.63 - Dobot Magician block-stacking.
Fixes which of two same-coloured blocks is left for last: the cell next to the
tower is now picked early, while the tower is still short.

---
main_1.62
Same motion as main_1.61, but the arm parks above the first block to pick while
the plan is on screen, so pressing Enter starts with the descent.

---
main_1.61
Teach has three kinds, every point of the chosen kind is required (no skipping):
  p = strict, 13 points: cells 1-8, centre, temp 1-4   (nothing is interpolated)
  Enter = 6 points: 4 grid corners + 2 temp ends
  q = quick, 3 corners on the floor, temp off
Run with --sim for simulation (no robot needed)."""

import os
import re
import sys
import glob
import json
import time
import math
import struct
import importlib.util
from pydobot import Dobot
from pydobot.message import Message

SETTINGS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "smart_settings.json")

DEFAULT_SETTINGS = {
    "port": "/dev/ttyUSB0",
    "robot_speed": 100,
    "robot_accel": 100,
    "block_height": 25.0,
    "ground_z": None,
    "grip_offset": 0.0,
    "keep_rotation": True,          # keep pick angle through place, so blocks land square
    "temp_count": 2,                # how many of the LAST blocks are parked at temp first
    "order": [1, 2, 3, 4],
    "colors": ["g", "r", "y", "b"],
    "last_input": "g r y b",
    "flip_camera": True,            # camera faces the robot, so its view is rotated 180
    "corner_ref": "top",            # grid corners taught on a block TOP, or on the FLOOR (quick teach)
    "teach_slot": None,             # which teach kind is active now: normal / full / quick
    "saves": {},                    # one kept copy per teach kind, restored by [l] Load
    "positions": {k: None for k in
                  ("grid_1", "grid_3", "grid_8", "grid_6", "temp_top", "temp_last")
                  + tuple(f"cell_{n}" for n in range(1, 9)) + ("cell_c",)
                  + tuple(f"temp_{i}" for i in range(1, 5))},
}

CELL_KEYS = tuple(f"cell_{n}" for n in range(1, 9))   # the 8 cells, taught one by one (strict)
CENTER_KEY = "cell_c"                                 # the tower cell, taught too (strict)
TEMP_KEYS = tuple(f"temp_{i}" for i in range(1, 5))   # all four temp slots (strict)
STRICT_KEYS = CELL_KEYS + (CENTER_KEY,) + TEMP_KEYS

SUCK_DELAY_MS = 50
RELEASE_DELAY_MS = 100

# 3x3 board, left->right, top->bottom: [1,2,3 / 4,c,5 / 6,7,8]
GRID_LAYOUT = {1: (0, 0), 2: (0, 1), 3: (0, 2),
               4: (1, 0), "c": (1, 1), 5: (1, 2),
               6: (2, 0), 7: (2, 1), 8: (2, 2)}

CAMERA_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "high-level")
CAMERA_FUNCS = ("get_blocks", "detect_blocks")   # must return {cell: color}

COLORS = {"g": "green", "r": "red", "y": "yellow", "b": "blue"}
FLIP_CELL = {1: 8, 2: 7, 3: 6, 4: 5, 5: 4, 6: 3, 7: 2, 8: 1}


def load_settings():
    s = dict(DEFAULT_SETTINGS)
    s["positions"] = dict(DEFAULT_SETTINGS["positions"])
    try:
        with open(SETTINGS_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
        s["positions"].update(data.pop("positions", None) or {})
        old = data.pop("use_temp", None)              # legacy bool
        if old is not None and "temp_orders" not in data and "temp_count" not in data:
            data["temp_count"] = 2 if old else 0
        slots = data.pop("temp_orders", None)         # legacy list of place slots
        if slots is not None and "temp_count" not in data:
            # the tower only gets taller, so a slot list is really "the last N blocks"
            data["temp_count"] = 5 - min(slots) if slots else 0
        s.update(data)
    except FileNotFoundError:
        pass
    except Exception as e:
        print(f"! cannot read smart_settings.json ({e}), using defaults")
    return s


def save_settings(settings):
    try:
        with open(SETTINGS_FILE, "w", encoding="utf-8") as f:
            json.dump(settings, f, indent=4, ensure_ascii=False)
        print("saved smart_settings.json")
    except Exception as e:
        print(f"! save failed: {e}")


def connect_robot(settings):
    port = settings.get("port")
    try:
        device = Dobot(port=port)
    except Exception as e:
        print(f"! cannot open {port} ({e}), scanning ports...")
        try:
            device = Dobot()
        except Exception as err:
            print(f"! connect failed: {err}")
            if input("use simulation mode instead? (y/n): ").strip().lower() == "y":
                return FakeDobot()
            return None
    time.sleep(1.5)
    if hasattr(device, "clear_alarms"):
        device.clear_alarms()
        time.sleep(0.5)
    device.speed(settings["robot_speed"], settings["robot_accel"])
    print("Dobot connected")
    return device


def queued_wait(device, ms):
    """SetWAITCmd (ID 110): dwell queued on the robot, no Python-side wait."""
    msg = Message()
    msg.id = 110
    msg.ctrl = 0x03
    msg.params = bytearray(struct.pack("I", ms))
    return device._extract_cmd_index(device._send_command(msg))


def emergency_stop(device):
    """Stop now, clear the queue, release the cup."""
    for cmd_id in (242, 245):  # ForceStopExec, ClearQueue
        msg = Message()
        msg.id = cmd_id
        msg.ctrl = 0x01
        device._send_command(msg)
    device._set_queued_cmd_start_exec()
    device.suck(False)


class FakePosition:
    def __init__(self, x, y, z, r):
        self.x, self.y, self.z, self.r = x, y, z, r


class FakePose:
    def __init__(self, position):
        self.position = position


class FakeDobot:
    """Simulation: prints the commands instead of moving. Teach/SetGround are blocked."""

    HOME = (200.0, 0.0, 0.0, 0.0)

    def __init__(self):
        self.pos = list(self.HOME)
        self.index = 0

    def _queue(self, text):
        self.index += 1
        print(f"  [sim {self.index:3d}] {text}")
        return self.index

    def get_pose(self):
        return FakePose(FakePosition(*self.pos))

    def move_to(self, x, y, z, r, mode=None):
        self.pos = [x, y, z, r]
        return self._queue(f"move  x={x:7.2f}  y={y:7.2f}  z={z:7.2f}  r={r:7.2f}")

    def suck(self, on):
        return self._queue(f"suck  {'ON' if on else 'OFF'}")

    def speed(self, *args):
        pass

    def wait_for_cmd(self, index):
        pass

    def close(self):
        pass

    def _send_command(self, msg):
        if msg.id == 110:
            self._queue(f"wait  {struct.unpack('I', bytes(msg.params))[0]} ms")
        return msg

    def _extract_cmd_index(self, msg):
        return self.index

    def _set_queued_cmd_start_exec(self):
        pass


def is_sim(device):
    return isinstance(device, FakeDobot)


class Mover:
    """Queues a whole block sequence at once, waiting only on the previous block
    (<= 2 blocks, ~20 commands, robot queue holds 32)."""

    def __init__(self, device):
        self.d = device
        p = device.get_pose().position
        self.pos = (p.x, p.y, p.z, p.r)
        self.last = None
        self.prev_block = None

    def move(self, x, y, z, r):
        if all(abs(a - b) < 0.005 for a, b in zip((x, y, z, r), self.pos)):
            return              # already there; a zero-length PTP still costs a stop
        self.last = self.d.move_to(x, y, z, r)
        self.pos = (x, y, z, r)

    def suck(self, on, ms):
        self.last = self.d.suck(on)
        if ms > 0:
            self.last = queued_wait(self.d, ms)

    def lift(self, safe_z):
        x, y, z, r = self.pos
        if z < safe_z - 2.0:
            self.move(x, y, safe_z, r)

    def pick_and_place(self, src, tgt, carry_z, empty_z, keep_rotation=True):
        # empty travel at empty_z, loaded travel at carry_z, no lift after release
        # keep_rotation: hold the pick angle so the block is not turned on the way
        r_place = src["r"] if keep_rotation else tgt["r"]
        self.lift(empty_z)
        self.move(src["x"], src["y"], empty_z, src["r"])
        self.move(src["x"], src["y"], src["z"], src["r"])
        self.suck(True, SUCK_DELAY_MS)
        self.move(src["x"], src["y"], carry_z, src["r"])
        self.move(tgt["x"], tgt["y"], carry_z, r_place)
        self.move(tgt["x"], tgt["y"], tgt["z"], r_place)
        self.suck(False, RELEASE_DELAY_MS)
        if self.prev_block is not None:
            self.d.wait_for_cmd(self.prev_block)
        self.prev_block = self.last

    def finish(self):
        if self.last is not None:
            self.d.wait_for_cmd(self.last)


def safe_z_for(base, block_h, tower_height, carrying=True):
    """Half a block of clearance over the tallest obstacle, plus the carried block."""
    obstacle = max(tower_height, 1)
    return base + (obstacle + (1.5 if carrying else 0.5)) * block_h


# ==========================================
# geometry
# ==========================================
def point(x, y, z=None):
    """r = atan2(y, x); z (if given) = block-top level at that spot."""
    p = {"x": round(x, 2), "y": round(y, 2),
         "r": round(math.degrees(math.atan2(y, x)), 2)}
    if z is not None:
        p["z"] = round(z, 2)
    return p


def lerp(a, b, t):
    return a + (b - a) * t


def solve3(rows):
    """Gaussian elimination on 3 unknowns; rows = [[a, b, c, rhs], ...]."""
    m = [list(r) for r in rows]
    for i in range(3):
        piv = max(range(i, 3), key=lambda k: abs(m[k][i]))
        if abs(m[piv][i]) < 1e-9:
            return None
        m[i], m[piv] = m[piv], m[i]
        for k in range(i + 1, 3):
            f = m[k][i] / m[i][i]
            for j in range(i, 4):
                m[k][j] -= f * m[i][j]
    x = [0.0, 0.0, 0.0]
    for i in (2, 1, 0):
        x[i] = (m[i][3] - sum(m[i][j] * x[j] for j in range(i + 1, 3))) / m[i][i]
    return x


def fit_plane(pts):
    """z = a + b*x + c*y (3 points exact, 4 points least-squares).
    The table is not flat: ~3-4 mm per 100 mm along x, so a single z is off by ~3 mm."""
    pts = [p for p in pts if p and p.get("z") is not None]
    if len(pts) < 3:
        return None
    basis = [(1.0, p["x"], p["y"]) for p in pts]
    rows = [[sum(b[i] * b[j] for b in basis) for j in range(3)]
            + [sum(b[i] * p["z"] for b, p in zip(basis, pts))] for i in range(3)]
    return solve3(rows)


REQUIRED_CORNERS = ("grid_1", "grid_3", "grid_8")   # top-left, top-right, bottom-right
OPTIONAL_CORNER = "grid_6"                          # bottom-left
GRID_CORNERS = REQUIRED_CORNERS + (OPTIONAL_CORNER,)


def taught_cells(positions):
    """The 8 cells if every one of them was taught one by one, else None."""
    cells = {n: positions.get(f"cell_{n}") for n in range(1, 9)}
    return cells if all(cells.values()) else None


def build_grid(positions, settings=None):
    """Affine map: pos = origin + col*u + row*v, so a rotated board still works.
    3 corners fit exactly, 4 corners use least-squares. z comes from fit_plane.
    If all 8 cells were taught directly (p mode) those are used as-is and the
    centre is their mean. corner_ref="floor" means the corner z is the floor,
    so one block height is added to get the block top."""
    settings = settings or {}
    cells = taught_cells(positions)
    if cells:
        out = {n: point(c["x"], c["y"], c.get("z")) for n, c in cells.items()}
        mid = positions.get(CENTER_KEY)
        if mid:
            out["c"] = point(mid["x"], mid["y"], mid.get("z"))
        else:                       # not taught: the ring of 8 averages to the centre
            out["c"] = point(*(sum(c[k] for c in cells.values()) / 8 for k in "xy"),
                             sum(c["z"] for c in cells.values()) / 8
                             if all(c.get("z") is not None for c in cells.values()) else None)
        return out
    p1, p3, p8 = (positions.get(k) for k in REQUIRED_CORNERS)
    if not (p1 and p3 and p8):
        return None
    p6 = positions.get(OPTIONAL_CORNER)
    if p6:
        corners = (p1, p3, p6, p8)
        ux, uy = (((p3[c] + p8[c]) - (p1[c] + p6[c])) / 4 for c in "xy")
        vx, vy = (((p6[c] + p8[c]) - (p1[c] + p3[c])) / 4 for c in "xy")
        ox = sum(p["x"] for p in corners) / 4 - ux - vx
        oy = sum(p["y"] for p in corners) / 4 - uy - vy
    else:
        ox, oy = p1["x"], p1["y"]
        ux, uy = ((p3[c] - p1[c]) / 2 for c in "xy")
        vx, vy = ((p8[c] - p3[c]) / 2 for c in "xy")
    plane = fit_plane((p1, p3, p8, p6))
    lift = settings.get("block_height", 25.0) if settings.get("corner_ref") == "floor" else 0.0
    out = {}
    for name, (row, col) in GRID_LAYOUT.items():
        x, y = ox + col * ux + row * vx, oy + col * uy + row * vy
        out[name] = point(x, y, None if plane is None
                          else plane[0] + plane[1] * x + plane[2] * y + lift)
    return out


def warn_missing_grid(positions):
    missing = [k for k in REQUIRED_CORNERS if not positions.get(k)]
    old_style = (positions.get("grid_1") and positions.get("grid_8")
                 and not positions.get("grid_3") and not positions.get(OPTIONAL_CORNER))
    if old_style:
        print("! taught data is the old 2-corner kind and cannot be used")
        print("  press [6] ResetPositions then [2] Teach")
    else:
        print(f"! grid corners incomplete, missing: {', '.join(missing)} - use [2] Teach")


def build_temps(positions, block_h):
    """Temp slots 1-4 along temp_top..temp_last. Both are taught on the floor,
    so block_h is added back: z means block-top level, same as the grid."""
    direct = {i: positions.get(f"temp_{i}") for i in range(1, 5)}
    if all(direct.values()):        # strict teach: every slot was touched on the floor
        return {i: point(p["x"], p["y"],
                         p["z"] + block_h if p.get("z") is not None else None)
                for i, p in direct.items()}
    a, b = positions.get("temp_top"), positions.get("temp_last")
    if not (a and b):
        return None
    has_z = a.get("z") is not None and b.get("z") is not None
    return {i: point(lerp(a["x"], b["x"], (i - 1) / 3), lerp(a["y"], b["y"], (i - 1) / 3),
                     lerp(a["z"], b["z"], (i - 1) / 3) + block_h if has_z else None)
            for i in range(1, 5)}


MIN_BLOCK_H = 8.0   # below this the corners were taught on the floor, not on a block


def measured_block_height(positions, ground_z=None):
    """Mean corner top - floor. Floor from the two temp points, else ground_z."""
    keys = CELL_KEYS + (CENTER_KEY,) if taught_cells(positions) else GRID_CORNERS
    tops = [positions[k]["z"] for k in keys
            if positions.get(k) and positions[k].get("z") is not None]
    floor_keys = TEMP_KEYS if all(positions.get(k) for k in TEMP_KEYS) else ("temp_top", "temp_last")
    floors = [positions[k]["z"] for k in floor_keys
              if positions.get(k) and positions[k].get("z") is not None]
    if not tops:
        return None
    if floors:
        base = sum(floors) / len(floors)
    elif ground_z is not None:
        base = ground_z
    else:
        return None
    return sum(tops) / len(tops) - base


def check_z_scheme(positions, settings):
    """Guard: corners taught on the floor make every z one block too low and the
    cup would press into the table. Returns (message, must_stop) or None."""
    if settings.get("corner_ref") == "floor":
        return None      # quick teach puts the corners on the floor on purpose
    exact = measured_block_height(positions)
    h = exact if exact is not None else measured_block_height(positions, settings.get("ground_z"))
    if h is None:
        return None
    if h < MIN_BLOCK_H:
        return (f"! corner tops are only {h:.2f} mm above the floor - corners look taught "
                f"on the floor\n  press [6] ResetPositions then [2] Teach on block tops",
                True)
    if exact is None:
        return None      # one ground_z touch is too coarse to judge block_height
    diff = h - settings.get("block_height", 25.0)
    if abs(diff) > 2.0:
        return (f"! block_height is {settings.get('block_height', 25.0):.2f} mm but measured "
                f"{h:.2f} mm ({diff:+.2f}) - [2] Teach updates it",
                False)
    return None


def valid_order(order):
    """Four distinct cells 1-8."""
    try:
        order = [int(b) for b in order]
    except (TypeError, ValueError):
        return None
    if len(order) != 4 or len(set(order)) != 4 or any(b < 1 or b > 8 for b in order):
        return None
    return order


def latest_camera_file():
    """Highest camara_x.x.py in high-level."""
    def version(path):
        m = re.search(r"camara_(\d+)\.(\d+)", os.path.basename(path))
        return (int(m.group(1)), int(m.group(2))) if m else (-1, -1)

    files = [f for f in glob.glob(os.path.join(CAMERA_DIR, "camara_*.py")) if version(f) != (-1, -1)]
    return max(files, key=version) if files else None


def load_camera():
    """Import the newest camara_x.x.py. Returns (module, name) or (None, None)."""
    path = latest_camera_file()
    if not path:
        print(f"cam: no camara_*.py in {CAMERA_DIR}")
        return None, None
    name = os.path.basename(path)
    try:
        spec = importlib.util.spec_from_file_location("camara_latest", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    except Exception as e:
        print(f"cam: cannot load {name} ({e})")
        return None, None
    return module, name


def camera_setup():
    """[9] Open the camera 3x3 setup window.
    Press k in the window to save; without it the run keeps the old frame."""
    module, name = load_camera()
    if module is None:
        return
    fn = getattr(module, "setup", None)
    if not callable(fn):
        print(f"cam: {name} has no setup()")
        return
    cfg_file = getattr(module, "CONFIG_FILE", None)

    def snapshot():
        try:
            with open(cfg_file, "rb") as f:
                return f.read()
        except Exception:
            return None

    before = snapshot()
    print(f"cam: {name} setup window - k = SAVE, q = quit")
    print("     press k BEFORE q, otherwise the run uses the old frame")
    try:
        fn()
    except Exception as e:
        print(f"cam: setup failed ({e})")
        return
    if cfg_file and snapshot() == before:
        print("! camera config NOT saved - reopen [9] and press k")
    else:
        print("camera config saved")


def blocks_from_camera(settings):
    """{cell: color} from the newest camera file, or None if unusable."""
    module, name = load_camera()
    if module is None:
        return None

    fn = next((getattr(module, n) for n in CAMERA_FUNCS if callable(getattr(module, n, None))), None)
    if fn is None:
        print(f"cam: {name} has no {' / '.join(CAMERA_FUNCS)}")
        return None
    try:
        blocks = fn()
    except Exception as e:
        print(f"cam: {name}: {e}")
        return None

    try:
        blocks = {int(cell): str(color) for cell, color in dict(blocks).items()}
    except Exception:
        print(f"cam: {name} returned a bad shape (need dict {{cell: color}})")
        return None
    if any(c < 1 or c > 8 for c in blocks) or any(v not in COLORS for v in blocks.values()):
        print(f"cam: {name} returned an unknown cell or color: {blocks}")
        return None
    if settings.get("flip_camera", True):
        blocks = {FLIP_CELL[c]: v for c, v in blocks.items()}
        print("cam: view flipped 180 to robot side (flip_camera)")
    print(f"cam: {name} found " + " ".join(f"{c}={blocks[c]}" for c in sorted(blocks)))
    return blocks


def valid_colors(colors):
    """Four of g/r/y/b, each colour at most twice."""
    colors = [str(c).lower() for c in colors]
    if len(colors) != 4 or any(c not in COLORS for c in colors):
        return None
    if any(colors.count(c) > 2 for c in colors):
        print("! a colour can repeat at most twice")
        return None
    return colors


def ask_input(settings, numbers_only=False, enter_hint=None):
    """Read 4 items: cell numbers (1-8, distinct) or colours (g/r/y/b).
    Returns ("order", [cells]) / ("colors", [colours]) / None."""
    # numbers_only is the camera-failed fallback, so it must default to the cell order
    default_order = " ".join(str(b) for b in settings.get("order", []))
    cur = default_order if numbers_only else (settings.get("last_input") or default_order)
    legend = ", ".join(f"{k}={v}" for k, v in COLORS.items())
    prompt = ("order of 4 - cells 1-8 e.g. '3 5 8 2'"
              + ("" if numbers_only else f" or colours ({legend}) e.g. 'g r y b'")
              + f" [{cur}] ({enter_hint or 'Enter=keep'}): ")
    tokens = (input(prompt).strip().lower() or cur).replace(",", " ").split()

    if tokens and all(t.isdigit() for t in tokens):
        order = valid_order(tokens)
        if not order:
            print("! need 4 distinct cells 1-8")
            return None
        settings["last_input"] = " ".join(str(b) for b in order)
        settings["order"] = order
        return "order", order

    if numbers_only:
        print("! camera unusable, enter 4 cell numbers 1-8")
        return None
    colors = valid_colors(tokens)
    if not colors:
        print(f"! need 4 cells 1-8 or 4 colours {' / '.join(COLORS)}")
        return None
    settings["last_input"] = " ".join(colors)
    settings["colors"] = colors
    return "colors", colors


def near_rank(grid):
    """Rank by nearness to the robot: row band first, then distance within the row."""
    def dist(cell):
        p = grid[cell]
        return math.hypot(p["x"], p["y"])

    rows = {}
    for cell, (row, _) in GRID_LAYOUT.items():
        if cell != "c":
            rows.setdefault(row, []).append(cell)
    row_order = sorted(rows, key=lambda r: sum(dist(c) for c in rows[r]) / len(rows[r]))
    return lambda cell: (row_order.index(GRID_LAYOUT[cell][0]), dist(cell))


def order_from_colors(colors, blocks, grid):
    """Pick the blocks of that colour nearest the robot, then decide the order they
    go up by distance from the TOWER, closest first.

    The tower grows with every layer, so the last pick is the one most likely to clip
    it. Edge cells sit ~33 mm from the centre and corner cells ~47 mm, so leaving an
    edge cell for last is the worst case - exactly what used to happen with e.g.
    red at 1,2 and green at 6,7 asked as "r r g g": cell 7 ended up last."""
    rank = near_rank(grid)
    centre = grid["c"]

    def tower_gap(cell):
        p = grid[cell]
        return math.hypot(p["x"] - centre["x"], p["y"] - centre["y"])

    by_color = {}
    for cell, color in blocks.items():
        by_color.setdefault(color, []).append(cell)
    for color in by_color:
        by_color[color].sort(key=rank)   # near -> far from the robot

    order = [None] * 4
    for color in set(colors):
        slots = [i for i, c in enumerate(colors) if c == color]
        cells = by_color.get(color, [])
        if len(cells) < len(slots):
            print(f"! {color} ({COLORS[color]}) needs {len(slots)} blocks, camera found {len(cells)}")
            return None
        for slot, cell in zip(slots, sorted(cells[:len(slots)], key=tower_gap)):
            order[slot] = cell
    return order


def tokens_of(text):
    return text.strip().lower().replace(",", " ").split()


def plan_order(settings, grid, on_plan=None):
    """Resolve the saved order straight away - the camera is read here, before the
    prompt - print the plan, then wait for Enter. Typing a new order re-plans
    without reading the camera again. Returns the cell order, or None.
    on_plan(order) runs as soon as a plan exists, before the prompt, so the arm
    can get into position while the operator reads it."""
    blocks, cam_failed = None, False

    def resolve(tokens):
        nonlocal blocks, cam_failed
        if tokens and all(t.isdigit() for t in tokens):
            order = valid_order(tokens)
            if not order:
                print("! need 4 distinct cells 1-8")
                return None
            settings["last_input"] = " ".join(str(b) for b in order)
            settings["order"] = order
            print("plan: " + " -> ".join(f"cell {b}" for b in order))
            return order
        colors = valid_colors(tokens)
        if not colors:
            print(f"! need 4 cells 1-8 or 4 colours {' / '.join(COLORS)}")
            return None
        if blocks is None and not cam_failed:
            blocks = blocks_from_camera(settings)
            cam_failed = not blocks
        if cam_failed:
            print("camera unusable, enter cell numbers instead")
            return None
        order = order_from_colors(colors, blocks, grid)
        if not order:
            return None
        settings["last_input"] = " ".join(colors)
        settings["colors"] = colors
        settings["order"] = order
        print("plan: " + " -> ".join(f"{c}(cell {b})" for c, b in zip(colors, order)))
        return order

    order = resolve(tokens_of(settings.get("last_input")
                              or " ".join(str(b) for b in settings.get("order", []))))
    while True:
        if order and on_plan:
            on_plan(order)
        raw = input("[Enter]=RUN | type a new order | q=cancel > " if order
                    else "type an order (cells 1-8, or colours) | q=cancel > ").strip()
        if raw.lower() == "q":
            print("cancelled")
            return None
        if raw:
            order = resolve(tokens_of(raw))
        elif order:
            return order


LAYOUT_DWELL_MS = 500


def walk_layout(device, settings, grid, temps):
    """Touch every point (1,2,3,4,c,5,6,7,8 then temp 1-4) to verify the taught grid."""
    ground_z = settings.get("ground_z")
    if ground_z is None:
        print("! no ground set ([3]), cannot walk")
        return
    block_h = settings.get("block_height", 25.0)
    grip = settings.get("grip_offset", 0.0)
    fallback_top = ground_z + block_h
    base_z = grid["c"].get("z", fallback_top) - block_h
    hover_z = safe_z_for(base_z, block_h, 0, carrying=False)

    stops = [(f"cell {n}", grid[n]) for n in (1, 2, 3, 4, "c", 5, 6, 7, 8)]
    stops += [(f"temp_{i}", temps[i]) for i in range(1, 5)] if temps else []
    print(f"walking {len(stops)} points, {LAYOUT_DWELL_MS / 1000:.1f}s each - Ctrl+C to stop")

    mover = Mover(device)
    for label, p in stops:
        table_z = p.get("z", fallback_top) + grip
        print(f"  -> {label}: ({p['x']:.2f}, {p['y']:.2f}) z={table_z:.2f} r={p['r']:.2f}")
        mover.lift(hover_z)
        mover.move(p["x"], p["y"], hover_z, p["r"])
        mover.move(p["x"], p["y"], table_z, p["r"])
        mover.last = queued_wait(device, LAYOUT_DWELL_MS)
        device.wait_for_cmd(mover.last)
    mover.lift(hover_z)
    mover.finish()
    print("walk done")


def show_layout(settings, device=None):
    """Print every computed point; with a device, offer to walk them."""
    grid = build_grid(settings["positions"], settings)
    temps = build_temps(settings["positions"], settings.get("block_height", 25.0))
    if not grid:
        warn_missing_grid(settings["positions"])
        return
    warn = check_z_scheme(settings["positions"], settings)
    if warn:
        print(warn[0])
    if taught_cells(settings["positions"]):
        mode = "8 cells taught one by one"
    else:
        mode = "4 corners (least-squares)" if settings["positions"].get(OPTIONAL_CORNER) else "3 corners"
        if settings.get("corner_ref") == "floor":
            mode += " on the floor + block_height"
    print(f"\ngrid 3x3 from {mode}:")
    for row in ([1, 2, 3], [4, "c", 5], [6, 7, 8]):
        print("  " + " | ".join(
            f"{n}: ({grid[n]['x']:7.2f},{grid[n]['y']:7.2f}) r={grid[n]['r']:6.2f}" for n in row))
    zs = [grid[n].get("z") for n in GRID_LAYOUT]
    if None not in zs:
        src = "taught" if taught_cells(settings["positions"]) else "fitted plane"
        print(f"block top per cell ({src}, spread {max(zs) - min(zs):.2f} mm):")
        for row in ([1, 2, 3], [4, "c", 5], [6, 7, 8]):
            print("  " + " | ".join(f"{n}: {grid[n]['z']:7.2f}" for n in row))
    else:
        print("! taught data has no z, falling back to ground_z + block_height everywhere")
    if temps:
        print("temp slots (top -> bottom):")
        for i in range(1, 5):
            z = f" z={temps[i]['z']:7.2f}" if temps[i].get("z") is not None else ""
            print(f"  temp_{i}: ({temps[i]['x']:7.2f},{temps[i]['y']:7.2f}) r={temps[i]['r']:6.2f}{z}")
    else:
        print("! temp points not taught (temp_top / temp_last)")

    if device is None:
        return
    if warn and warn[1]:
        print("! not walking, z data is wrong (see above)")
        return
    if input("walk the points with the arm? (y/n): ").strip().lower() == "y":
        walk_layout(device, settings, grid, temps)


# ==========================================
# run
# ==========================================
def run_operation(device, settings):
    positions = settings["positions"]
    grid = build_grid(positions, settings)
    temps = build_temps(positions, settings.get("block_height", 25.0))
    temp_count = max(0, min(4, int(settings.get("temp_count", 2))))
    if not grid:
        warn_missing_grid(positions)
        return
    warn = check_z_scheme(positions, settings)
    if warn:
        print(warn[0])
        if warn[1]:
            return
    if temp_count and not temps:
        print("! temp points not taught - use [2] Teach or turn temp off in [7]")
        return
    ground_z = settings.get("ground_z")
    if ground_z is None:
        print("! no ground set, use [3] SetGround")
        return

    block_h = settings.get("block_height", 25.0)
    grip = settings.get("grip_offset", 0.0)
    keep_rot = settings.get("keep_rotation", True)
    center = grid["c"]
    fallback_top = ground_z + block_h

    def top_of(pos):
        return pos.get("z", fallback_top) + grip

    base_z = center.get("z", fallback_top) - block_h   # floor under the tower
    carry_z = safe_z_for(base_z, block_h, 0)
    empty_z = safe_z_for(base_z, block_h, 0, carrying=False)
    # where every run already ends: above the centre, clear of a 4-high tower
    park_z = safe_z_for(base_z, block_h, 4, carrying=False)

    # the last temp_count blocks are parked at temp first (the tower is tallest then);
    # temp slots fill 4 -> 1, i.e. nearest the robot first
    def staged_for(order):
        staged, free_slots = {}, [4, 3, 2, 1]
        for i, b in enumerate(order):
            if i >= len(order) - temp_count:
                staged[b] = temps[free_slots.pop(0)]
        return staged

    mover = Mover(device)      # made before the prompt and reused for the run

    def get_ready(order):
        """Park the cup above the first block that will be picked, while the plan is
        on screen. The clock starts at Enter, so this traverse is off the critical
        path. Phase 1 goes first, so with temp on that is the first staged block."""
        staged = staged_for(order)
        first = next((grid[b] for b in order if b in staged), grid[order[0]])
        print("  (arm moving above the first pick - keep hands clear)")
        mover.lift(empty_z)
        mover.move(first["x"], first["y"], empty_z, first["r"])

    def park():
        """Back over the centre, high, so the arm does not sit in the camera's view."""
        mover.lift(park_z)
        mover.move(center["x"], center["y"], park_z, center["r"])
        mover.finish()

    order = plan_order(settings, grid, get_ready)
    if not order:
        park()
        return

    staged = staged_for(order)
    if not staged:
        print("! temp not used, picking straight from the board")

    print("run: " + " -> ".join(f"{b}{'(temp)' if b in staged else ''}" for b in order))

    def at(pos, z):
        return {"x": pos["x"], "y": pos["y"], "z": z, "r": pos["r"]}

    start = time.perf_counter()

    # phase 1: park blocks at temp
    for b in order:
        if b in staged:
            print(f"block {b} -> temp")
            mover.pick_and_place(at(grid[b], top_of(grid[b])), at(staged[b], top_of(staged[b])),
                                 carry_z, empty_z, keep_rot)

    # phase 2: stack at cell c, layer 1 lands on the block-top level there
    for layer, b in enumerate(order):
        src = staged.get(b, grid[b])
        tgt_z = top_of(center) + layer * block_h
        print(f"block {b} -> tower layer {layer + 1}")
        mover.pick_and_place(at(src, top_of(src)), at(center, tgt_z),
                             safe_z_for(base_z, block_h, layer),
                             safe_z_for(base_z, block_h, layer, carrying=False), keep_rot)
    mover.lift(safe_z_for(base_z, block_h, len(order), carrying=False))
    mover.finish()
    print(f"tower done, {len(order)} layers | {time.perf_counter() - start:.2f} sec")


# ==========================================
# teach
# ==========================================
# All 4 corners are taught on a block top; the 2 temp points are taught on the floor.
TEACH_STEPS = [
    ("grid_1", "cell 1 (top-left) block TOP"),
    ("grid_3", "cell 3 (top-right) block TOP"),
    ("grid_8", "cell 8 (bottom-right) block TOP"),
    ("grid_6", "cell 6 (bottom-left) block TOP"),
    ("temp_top", "temp_1 (topmost temp slot) FLOOR"),
    ("temp_last", "temp_4 (lowest temp slot) FLOOR"),
]


STRICT_STEPS = ([(f"cell_{n}", f"cell {n} block TOP") for n in range(1, 9)]
                + [(CENTER_KEY, "centre cell (tower spot) block TOP")]
                + [(f"temp_{i}", f"temp slot {i} FLOOR") for i in range(1, 5)])
QUICK_STEPS = [("grid_1", "cell 1 (top-left) FLOOR"),
               ("grid_3", "cell 3 (top-right) FLOOR"),
               ("grid_8", "cell 8 (bottom-right) FLOOR")]
QUICK_BLOCK_H = 25.0                         # quick teach cannot measure it, so assume 25 mm


def collect_points(device, positions, steps):
    """Walk the steps saving the arm position at each. False if the user quit.
    Every point of a teach kind is required - skipping used to leave a point from
    an older teach mixed in, which pulls the whole grid off (0.75x the board shift)."""
    print(f"{len(steps)} points | [Enter]=save point | q=quit")
    for i, (key, label) in enumerate(steps, 1):
        was = positions.get(key)
        note = f" [was ({was['x']}, {was['y']})]" if was else ""
        cmd = input(f"[{i}/{len(steps)}] move arm to {label}{note}, then [Enter]: ").strip().lower()
        if cmd == "q":
            # a half-finished teach would mix new points with old ones and nothing
            # downstream can tell, so drop the whole set of this kind
            clear_keys(positions, [k for k, _ in steps])
            print("! stopped early - the points of this teach were cleared ([l] Load restores a saved one)")
            return False
        p = device.get_pose().position
        positions[key] = {"x": round(p.x, 2), "y": round(p.y, 2),
                          "z": round(p.z, 2), "r": round(p.r, 2)}
        print(f"  {key}: {positions[key]}")
    return True


def clear_keys(positions, keys):
    for k in keys:
        if k in positions:
            positions[k] = None


def update_block_height(settings):
    h = measured_block_height(settings["positions"], settings.get("ground_z"))
    if h is None:
        return
    if h < MIN_BLOCK_H:
        print(f"! tops are only {h:.2f} mm above the floor - they must be taught on block TOPS")
        return
    was = settings.get("block_height", 25.0)
    settings["block_height"] = round(h, 2)
    print(f"block_height = {h:.2f} mm (was {was:.2f}), updated")


def teach_normal(device, settings):
    """[Enter] 4 grid corners on block tops + 2 temp points on the floor."""
    positions = settings["positions"]
    print("teach 6 points: 4 grid corners on block TOPS + 2 temp points on the FLOOR")
    clear_keys(positions, STRICT_KEYS)
    settings["corner_ref"] = "top"
    if not collect_points(device, positions, TEACH_STEPS):
        settings["teach_slot"] = None
        return
    t1, t4 = positions.get("temp_top"), positions.get("temp_last")
    if t1 and t4:
        settings["ground_z"] = round((t1["z"] + t4["z"]) / 2, 2)
        print(f"ground_z = {settings['ground_z']:.2f} mm "
              f"(mean of temp {t1['z']:.2f} / {t4['z']:.2f})")
        update_block_height(settings)
    else:
        print("! temp points incomplete, ground not set - use [3] SetGround")
    snapshot_teach(settings, "normal")
    show_layout(settings)


def teach_strict(device, settings):
    """p: 13 points - cells 1-8 and the centre on their block TOPS, then all four
    temp slots on the FLOOR. Nothing is interpolated and nothing may be skipped."""
    positions = settings["positions"]
    print("strict teach: cells 1-8 + centre on block TOPS, then temp 1-4 on the FLOOR")
    clear_keys(positions, GRID_CORNERS + ("temp_top", "temp_last"))
    settings["corner_ref"] = "top"
    if not collect_points(device, positions, STRICT_STEPS):
        settings["teach_slot"] = None
        return
    floors = [positions[k]["z"] for k in TEMP_KEYS]
    settings["ground_z"] = round(sum(floors) / len(floors), 2)
    print(f"ground_z = {settings['ground_z']:.2f} mm (mean of the 4 temp slots)")
    update_block_height(settings)
    snapshot_teach(settings, "strict")
    show_layout(settings)


def teach_quick(device, settings):
    """q: only cells 1, 3, 8 touched on the FLOOR. Temp is switched off and the
    block height is assumed to be 25 mm, so the whole setup is 3 points."""
    positions = settings["positions"]
    print("quick teach: cells 1, 3 and 8 with the cup on the FLOOR (no block under it)")
    clear_keys(positions, STRICT_KEYS + (OPTIONAL_CORNER, "temp_top", "temp_last"))
    if not collect_points(device, positions, QUICK_STEPS):
        settings["teach_slot"] = None
        return
    zs = [positions[k]["z"] for k in REQUIRED_CORNERS if positions.get(k)]
    if len(zs) < 3:
        print("! all 3 corners are needed")
        return
    settings["corner_ref"] = "floor"
    settings["temp_count"] = 0
    settings["block_height"] = QUICK_BLOCK_H
    settings["ground_z"] = round(sum(zs) / len(zs), 2)
    print(f"ground_z = {settings['ground_z']:.2f} mm (mean of the 3 corners), "
          f"block_height = {QUICK_BLOCK_H:.1f} mm, temp OFF")
    snapshot_teach(settings, "quick")
    show_layout(settings)


TEACH_SLOTS = {"normal": "Enter - 4 corners + 2 temp ends",
               "strict": "p - 13 points, cells 1-8 + centre + temp 1-4",
               "quick": "q - 3 corners on the floor"}
SLOT_FIELDS = ("ground_z", "block_height", "corner_ref", "temp_count")


def snapshot_teach(settings, slot):
    """Keep what was just taught under its own kind so [l] Load can bring it back."""
    data = {k: settings.get(k) for k in SLOT_FIELDS}
    data["positions"] = json.loads(json.dumps(settings["positions"]))
    settings.setdefault("saves", {})[slot] = data
    settings["teach_slot"] = slot


def load_teach(settings):
    """[l] Restore an earlier teach of a different kind, e.g. quick broke mid-run
    and the full teach from before is still kept."""
    saves = settings.get("saves") or {}
    current = settings.get("teach_slot")
    options = [(k, TEACH_SLOTS[k]) for k in TEACH_SLOTS if k in saves and k != current]
    if not options:
        print("! nothing else saved - teach with [2] first" if not saves
              else "! only the teach in use is saved")
        return
    print(f"\nin use: {TEACH_SLOTS.get(current, '-')}")
    for i, (_, label) in enumerate(options, 1):
        print(f"  [{i}] {label}")
    raw = input(f"load which one? 1-{len(options)} (Enter=cancel) > ").strip()
    if not raw.isdigit() or not 1 <= int(raw) <= len(options):
        print("cancelled")
        return
    slot, label = options[int(raw) - 1]
    data = saves[slot]
    settings["positions"] = json.loads(json.dumps(data["positions"]))
    for k in SLOT_FIELDS:
        if data.get(k) is not None:
            settings[k] = data[k]
    settings["teach_slot"] = slot
    print(f"loaded: {label}")
    if current == "quick":
        settings["temp_count"] = 1
        print("temp set to 1 - leaving quick usually means blocks were hitting the tower")
    show_layout(settings)


def teach_mode(device, settings):
    choice = input("\nteach - [Enter]=normal 6 points | p=strict 13 points | "
                   "q=quick 3 corners on floor (temp off) > ").strip().lower()
    if choice == "p":
        teach_strict(device, settings)
    elif choice == "q":
        teach_quick(device, settings)
    else:
        teach_normal(device, settings)


def configure_temp(settings):
    """How many of the LAST blocks get parked at temp before the tower is built.
    The tower grows with every block, so the risky ones are always the last."""
    cur = settings.get("temp_count", 2)
    raw = input(f"park how many of the LAST blocks at temp? 0-4 (0 = off) "
                f"[now: {cur}] (Enter=keep, q=cancel) > ").strip().lower()
    if raw in ("", "q"):
        print("unchanged")
        return
    try:
        n = int(raw)
    except ValueError:
        n = -1
    if not 0 <= n <= 4:
        print("! need 0-4, unchanged")
        return
    settings["temp_count"] = n
    print("temp off, every block goes straight to the tower" if n == 0
          else f"temp: the last {n} block(s) are parked first")


def unsaved_changes(settings):
    """True when memory differs from the file."""
    try:
        return settings != load_settings()
    except Exception:
        return True


def save_order(settings):
    """[8] enter the order and write it to the file right away."""
    answer = ask_input(settings,
                       enter_hint="Enter=Save" if unsaved_changes(settings) else "Enter=Exit")
    if not answer:
        print("not saved (bad order)")
        return
    if not unsaved_changes(settings):
        print("already matches the file, nothing to save")
        return
    kind, value = answer
    if kind == "order":
        print(f"cells: {' '.join(str(b) for b in value)} - used as typed, no camera")
    else:
        names = ", ".join(COLORS[c] for c in value)
        print(f"colours: {' '.join(value)} ({names}) - camera resolves them at run time")
    save_settings(settings)


def reset_positions(settings):
    """Clear all taught points; other settings are untouched."""
    filled = [k for k, v in settings["positions"].items() if v]
    if not filled:
        print("nothing taught yet")
        return
    print(f"taught points ({len(filled)}): {', '.join(filled)}")
    if input("clear them all? (y/n): ").strip().lower() != "y":
        print("cancelled")
        return
    settings["positions"] = {k: None for k in settings["positions"]}
    settings["teach_slot"] = None          # so [l] Load can offer the set just cleared
    saved = len(settings.get("saves") or {})
    print("cleared - press [4] Save or [8] to write the file"
          + (f" | [l] Load can restore {saved} saved teach(es)" if saved else ""))


def set_ground(device, settings):
    input("touch the table with the cup, then [Enter]...")
    settings["ground_z"] = round(device.get_pose().position.z, 2)
    print(f"ground_z = {settings['ground_z']:.2f} mm")
    h = input(f"block height [{settings['block_height']}] (Enter=keep): ").strip()
    if h:
        try:
            settings["block_height"] = round(float(h), 2)
        except ValueError:
            print("! bad value, kept")


def main():
    settings = load_settings()
    device = FakeDobot() if "--sim" in sys.argv else connect_robot(settings)
    if not device:
        raise SystemExit(1)
    if is_sim(device):
        print("SIM mode: nothing moves; [2] Teach and [3] SetGround are disabled")
    try:
        while True:
            n = settings.get("temp_count", 2)
            temp_state = f"last {n}" if n else "OFF"
            # show the saved order, and in colour mode the cell backup the fallback will offer
            numbers = " ".join(str(b) for b in settings.get("order", []))
            primary = settings.get("last_input") or numbers
            is_colors = bool(primary) and not primary.replace(" ", "").isdigit()
            backup = ((f" (cells {numbers})" if numbers else " (no cell backup)")
                      if is_colors else "")
            dirty = " *unsaved" if unsaved_changes(settings) else ""
            tag = "SIM " if is_sim(device) else ""
            choice = input(
                f"\n{tag}[1]Run [2]Teach:{settings.get('teach_slot') or '-'} [l]Load "
                f"[3]SetGround [4]Save [5]ShowLayout [9]Camera\n"
                f"[6]ResetPositions [7]Temp:{temp_state} [8]Order:{primary or '-'}{backup}"
                f"{dirty} [Enter]Exit > ").strip()
            if choice == "1":
                run_operation(device, settings)
            elif choice == "6":
                reset_positions(settings)
            elif choice == "7":
                configure_temp(settings)
            elif choice == "2":
                if is_sim(device):
                    print("! SIM mode cannot read the real arm position")
                else:
                    teach_mode(device, settings)
            elif choice == "3":
                if is_sim(device):
                    print("! SIM mode cannot read the real arm position")
                else:
                    set_ground(device, settings)
            elif choice == "4":
                save_settings(settings)
            elif choice == "5":
                show_layout(settings, device)
            elif choice == "8":
                save_order(settings)
            elif choice == "9":
                camera_setup()
            elif choice.lower() == "l":
                load_teach(settings)
            elif choice == "":
                break
    except KeyboardInterrupt:
        print("\nstopped (Ctrl+C)")
    finally:
        try:
            if not is_sim(device):
                emergency_stop(device)
            device.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
