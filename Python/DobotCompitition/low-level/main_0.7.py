import os
import re
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
    "order": [1, 2, 3, 4],
    "colors": ["g", "r", "y", "b"],
    "flip_camera": True,   # กล้องอยู่ฝั่งตรงข้ามหุ่น ภาพจึงกลับด้าน 180°
    "positions": {k: None for k in ("grid_1", "grid_8", "temp_top", "temp_last")},
}

SUCK_DELAY_MS = 50      # รอหัวดูดจับบล็อก
RELEASE_DELAY_MS = 100  # รอหัวดูดปล่อยบล็อก

# ผัง 3x3 เรียงซ้าย->ขวา, บน->ล่าง : [1,2,3 / 4,c,5 / 6,7,8]
GRID_LAYOUT = {1: (0, 0), 2: (0, 1), 3: (0, 2),
               4: (1, 0), "c": (1, 1), 5: (1, 2),
               6: (2, 0), 7: (2, 1), 8: (2, 2)}

# ฝั่งกล้อง: ไฟล์ camara_x.x.py เวอร์ชันล่าสุดในโฟลเดอร์ high-level
CAMERA_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "high-level")
# ฟังก์ชันในไฟล์กล้องที่จะเรียกหา (ตัวแรกที่เจอถูกใช้) ต้องคืน {ช่อง: สี} เช่น {1: "g", 3: "r", ...}
CAMERA_FUNCS = ("get_blocks", "detect_blocks")

COLORS = {"g": "เขียว", "r": "แดง", "y": "เหลือง", "b": "ฟ้า"}
# ภาพกล้องกลับด้าน 180° กับฝั่งหุ่น: ช่องตรงข้ามกันคือ 1<->8, 2<->7, 3<->6, 4<->5
FLIP_CELL = {1: 8, 2: 7, 3: 6, 4: 5, 5: 4, 6: 3, 7: 2, 8: 1}


def load_settings():
    s = dict(DEFAULT_SETTINGS)
    s["positions"] = dict(DEFAULT_SETTINGS["positions"])
    try:
        with open(SETTINGS_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
        s["positions"].update(data.pop("positions", None) or {})
        s.update(data)
    except FileNotFoundError:
        pass
    except Exception as e:
        print(f"⚠️ อ่าน smart_settings.json ไม่ได้ ({e}) ใช้ค่าเริ่มต้น")
    return s


def save_settings(settings):
    try:
        with open(SETTINGS_FILE, "w", encoding="utf-8") as f:
            json.dump(settings, f, indent=4, ensure_ascii=False)
        print("💾 บันทึก smart_settings.json แล้ว")
    except Exception as e:
        print(f"❌ บันทึกไม่สำเร็จ: {e}")


def connect_robot(settings):
    port = settings.get("port")
    try:
        device = Dobot(port=port)
    except Exception as e:
        print(f"⚠️ เชื่อมต่อ {port} ไม่ได้ ({e}) ลองค้นหาพอร์ตอัตโนมัติ...")
        try:
            device = Dobot()
        except Exception as err:
            print(f"❌ เชื่อมต่อล้มเหลว: {err}")
            return None
    time.sleep(1.5)
    if hasattr(device, "clear_alarms"):
        device.clear_alarms()
        time.sleep(0.5)
    device.speed(settings["robot_speed"], settings["robot_accel"])
    print("✅ เชื่อมต่อ Dobot แล้ว")
    return device


def queued_wait(device, ms):
    """คำสั่งรอ (SetWAITCmd, ID 110) ที่เข้าคิวบนหุ่นยนต์ ไม่ต้องรอฝั่ง Python"""
    msg = Message()
    msg.id = 110
    msg.ctrl = 0x03
    msg.params = bytearray(struct.pack("I", ms))
    return device._extract_cmd_index(device._send_command(msg))


def emergency_stop(device):
    """หยุดหุ่นยนต์ทันที ล้างคิวคำสั่ง แล้วปิดหัวดูด"""
    for cmd_id in (242, 245):  # ForceStopExec, ClearQueue
        msg = Message()
        msg.id = cmd_id
        msg.ctrl = 0x01
        device._send_command(msg)
    device._set_queued_cmd_start_exec()
    device.suck(False)


class Mover:
    """ส่งคำสั่งทั้งชุดของแต่ละบล็อกเข้าคิวของหุ่นยนต์ในครั้งเดียว
    แล้วรอแค่บล็อกก่อนหน้า (คิวมีไม่เกิน 2 บล็อก ~20 คำสั่ง < 32)"""

    def __init__(self, device):
        self.d = device
        p = device.get_pose().position
        self.pos = (p.x, p.y, p.z, p.r)
        self.last = None        # index ของคำสั่งล่าสุดในคิว
        self.prev_block = None  # index ของคำสั่งสุดท้ายของบล็อกก่อนหน้า

    def move(self, x, y, z, r):
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

    def pick_and_place(self, src, tgt, carry_z, empty_z):
        # ขาไปตัวเปล่า (ไม่มีบล็อก) เดินที่ empty_z, ขาถือบล็อกเดินที่ carry_z
        # ไม่ยกขึ้นหลังวาง: การยกครั้งถัดไป (lift) จะยกตรงไปที่ empty_z ของบล็อกถัดไปในครั้งเดียว
        self.lift(empty_z)
        self.move(src["x"], src["y"], empty_z, src["r"])
        self.move(src["x"], src["y"], src["z"], src["r"])
        self.suck(True, SUCK_DELAY_MS)
        self.move(src["x"], src["y"], carry_z, src["r"])
        self.move(tgt["x"], tgt["y"], carry_z, tgt["r"])
        self.move(tgt["x"], tgt["y"], tgt["z"], tgt["r"])
        self.suck(False, RELEASE_DELAY_MS)
        if self.prev_block is not None:
            self.d.wait_for_cmd(self.prev_block)
        self.prev_block = self.last

    def finish(self):
        if self.last is not None:
            self.d.wait_for_cmd(self.last)


def safe_z_for(base, block_h, tower_height, carrying=True):
    """เว้นระยะ 0.5 บล็อกเหนือสิ่งกีดขวางที่สูงที่สุด (บล็อกบนโต๊ะสูง 1 ชั้น หรือ Tower)
    - ถือบล็อก : + ความสูงบล็อกที่ถืออีก 1 ชั้น -> 2.5x / 3.5x / 4.5x
    - ตัวเปล่า  : 1.5x / 2.5x / 3.5x / 4.5x"""
    obstacle = max(tower_height, 1)
    return base + (obstacle + (1.5 if carrying else 0.5)) * block_h


# ==========================================
# 📐 คำนวณพิกัดจากจุดที่สอนไว้ 4 จุด
# ==========================================
def point(x, y):
    """มุมหมุน r = atan2(y, x) เสมอ (ตรงกับค่าที่สอนไว้ทุกจุดในเวอร์ชันก่อน)"""
    return {"x": round(x, 2), "y": round(y, 2),
            "r": round(math.degrees(math.atan2(y, x)), 2)}


def lerp(a, b, t):
    return a + (b - a) * t


def build_grid(positions):
    """คำนวณ 9 ช่องจากมุมบนซ้าย (บล็อก 1) และมุมล่างขวา (ตำแหน่ง 8)
    สมมติว่าแถว (บน->ล่าง) ไล่ไปตามแกน X และคอลัมน์ (ซ้าย->ขวา) ไล่ไปตามแกน Y ของหุ่นยนต์"""
    p1, p8 = positions.get("grid_1"), positions.get("grid_8")
    if not (p1 and p8):
        return None
    return {name: point(lerp(p1["x"], p8["x"], row / 2), lerp(p1["y"], p8["y"], col / 2))
            for name, (row, col) in GRID_LAYOUT.items()}


def build_temps(positions):
    """คำนวณจุดพัก 4 ช่อง จากช่องบนสุด (temp_top) ถึงช่องล่างสุด (temp_last)"""
    a, b = positions.get("temp_top"), positions.get("temp_last")
    if not (a and b):
        return None
    return {i: point(lerp(a["x"], b["x"], (i - 1) / 3), lerp(a["y"], b["y"], (i - 1) / 3))
            for i in range(1, 5)}


def valid_order(order):
    """ลำดับที่ใช้ได้: เลข 1-8 จำนวน 4 ตัว ไม่ซ้ำกัน"""
    try:
        order = [int(b) for b in order]
    except (TypeError, ValueError):
        return None
    if len(order) != 4 or len(set(order)) != 4 or any(b < 1 or b > 8 for b in order):
        return None
    return order


def latest_camera_file():
    """ไฟล์ camara_x.x.py เวอร์ชันสูงสุดใน high-level"""
    def version(path):
        m = re.search(r"camara_(\d+)\.(\d+)", os.path.basename(path))
        return (int(m.group(1)), int(m.group(2))) if m else (-1, -1)

    files = [f for f in glob.glob(os.path.join(CAMERA_DIR, "camara_*.py")) if version(f) != (-1, -1)]
    return max(files, key=version) if files else None


def blocks_from_camera(settings):
    """ขอ {ช่อง: สี} จากไฟล์กล้องเวอร์ชันล่าสุด คืน None ถ้าใช้ไม่ได้ (ให้ไปกรอกเอง)"""
    path = latest_camera_file()
    if not path:
        print(f"📷 ไม่พบไฟล์ camara_*.py ใน {CAMERA_DIR}")
        return None
    name = os.path.basename(path)
    try:
        spec = importlib.util.spec_from_file_location("camara_latest", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    except Exception as e:
        print(f"📷 โหลด {name} ไม่ได้ ({e})")
        return None

    fn = next((getattr(module, n) for n in CAMERA_FUNCS if callable(getattr(module, n, None))), None)
    if fn is None:
        print(f"📷 {name} ไม่มีฟังก์ชัน {' / '.join(CAMERA_FUNCS)}")
        return None
    try:
        blocks = fn()
    except Exception as e:
        print(f"📷 {name}: {e}")
        return None

    try:
        blocks = {int(cell): str(color) for cell, color in dict(blocks).items()}
    except Exception:
        print(f"📷 {name} คืนค่าผิดรูปแบบ (ต้องเป็น dict {{ช่อง: สี}})")
        return None
    if any(c < 1 or c > 8 for c in blocks) or any(v not in COLORS for v in blocks.values()):
        print(f"📷 {name} คืนช่องหรือสีที่ไม่รู้จัก: {blocks}")
        return None
    if settings.get("flip_camera", True):
        blocks = {FLIP_CELL[c]: v for c, v in blocks.items()}
        print("🔄 กลับด้านภาพกล้อง 180° เป็นมุมมองหุ่น (flip_camera)")
    print(f"📷 {name} เจอบล็อก: " + " ".join(f"{c}={blocks[c]}" for c in sorted(blocks)))
    return blocks


def ask_colors(settings):
    """กรอกลำดับสี 4 ตัว เช่น 'g r y b' (ซ้ำสีได้ไม่เกินสีละ 2)"""
    cur = " ".join(settings.get("colors", []))
    legend = ", ".join(f"{k}={v}" for k, v in COLORS.items())
    raw = input(f"ลำดับสี 4 ตัว ({legend}) [{cur}] (Enter=ค่าเดิม): ").strip().lower()
    colors = (raw or cur).replace(",", " ").split()
    if len(colors) != 4 or any(c not in COLORS for c in colors):
        print(f"❌ ต้องเป็น {' / '.join(COLORS)} จำนวน 4 ตัว")
        return None
    if any(colors.count(c) > 2 for c in colors):
        print("❌ สีเดียวกันซ้ำได้ไม่เกิน 2 (บนโต๊ะมีสีละ 2 ก้อน)")
        return None
    settings["colors"] = colors
    return colors


def near_rank(grid):
    """จัดอันดับความใกล้หุ่น: ดูแถวก่อน (แถวที่ใกล้ฐานหุ่นสุดมาก่อน) แล้วค่อยดูระยะในแถว"""
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
    """เลือกบล็อกตามลำดับสี: ปกติเอาก้อนที่ใกล้หุ่นสุด
    ถ้าสีนั้นอยู่ในลำดับ 2 ครั้ง ให้วางก้อนที่ไกลกว่าก่อน แล้วค่อยก้อนที่ใกล้"""
    rank = near_rank(grid)
    by_color = {}
    for cell, color in blocks.items():
        by_color.setdefault(color, []).append(cell)
    for color in by_color:
        by_color[color].sort(key=rank)   # ใกล้ -> ไกล

    order = [None] * 4
    for color in set(colors):
        slots = [i for i, c in enumerate(colors) if c == color]
        cells = by_color.get(color, [])
        if len(cells) < len(slots):
            print(f"❌ สี {color} ({COLORS[color]}) ต้องใช้ {len(slots)} ก้อน แต่กล้องเจอ {len(cells)} ก้อน")
            return None
        if len(slots) == 1:
            order[slots[0]] = cells[0]                    # ใกล้สุด
        else:
            order[slots[0]], order[slots[1]] = cells[1], cells[0]   # ไกลก่อน แล้วใกล้
    return order


def get_order(settings, grid, use_camera=True):
    """โหมดกล้อง: กรอกลำดับ 'สี' แล้วให้กล้องบอกว่าสีไหนอยู่ช่องไหน
    ถ้ากล้องใช้ไม่ได้ ตกมาที่การกรอกลำดับ 'ช่อง' เอง"""
    if use_camera:
        colors = ask_colors(settings)
        blocks = blocks_from_camera(settings) if colors else None
        if blocks:
            order = order_from_colors(colors, blocks, grid)
            if order:
                print("🎨 ลำดับที่ได้: " + " -> ".join(
                    f"{c}(ช่อง {b})" for c, b in zip(colors, order)))
                settings["order"] = order
                return order
        print("↩️ ใช้การกรอกลำดับช่องเองแทน")
    order = ask_order(settings)
    if order:
        settings["order"] = order
    return order


def show_layout(settings):
    """แสดงพิกัดที่คำนวณได้ทั้งหมด"""
    grid = build_grid(settings["positions"])
    temps = build_temps(settings["positions"])
    if not grid:
        print("❌ ยังไม่ได้สอนมุมกริด (ใช้ [2] Teach)")
        return
    print("\n📋 กริด 3x3 (ซ้าย->ขวา, บน->ล่าง):")
    for row in ([1, 2, 3], [4, "c", 5], [6, 7, 8]):
        print("   " + " | ".join(
            f"{n}: ({grid[n]['x']:7.2f},{grid[n]['y']:7.2f}) r={grid[n]['r']:6.2f}" for n in row))
    if temps:
        print("📋 จุดพัก (บน->ล่าง):")
        for i in range(1, 5):
            print(f"   temp_{i}: ({temps[i]['x']:7.2f},{temps[i]['y']:7.2f}) r={temps[i]['r']:6.2f}")
    else:
        print("⚠️ ยังไม่ได้สอนจุดพัก (temp_top / temp_last)")


# ==========================================
# 🚀 RUN
# ==========================================
def run_operation(device, settings, use_camera=True):
    positions = settings["positions"]
    grid = build_grid(positions)
    temps = build_temps(positions)
    if not grid or not temps:
        print("❌ ยังสอนตำแหน่งไม่ครบ 4 จุด กรุณาใช้ [2] Teach ก่อน")
        return
    ground_z = settings.get("ground_z")
    if ground_z is None:
        print("❌ ยังไม่ได้ตั้ง Ground ใช้ [3] SetGround ก่อน")
        return

    order = get_order(settings, grid, use_camera=use_camera)
    if not order:
        return

    block_h = settings.get("block_height", 25.0)
    grip = settings.get("grip_offset", 0.0)
    table_z = ground_z + block_h + grip   # ระดับผิวบนของบล็อกที่วางบนโต๊ะ
    center = grid["c"]

    # บล็อก 1-4 ต้องย้ายไปพักก่อน ใช้ช่องพักจาก 4 ไป 1 เพื่อป้องกันชน
    # ยกเว้นลำดับที่ 1-2 เพราะ Tower ยังสูงไม่เกิน 1 ชั้น ไม่มีอะไรให้ชน
    staged = {}
    free_slots = [4, 3, 2, 1]
    for i, b in enumerate(order):
        if b <= 4 and i >= 2:
            staged[b] = temps[free_slots.pop(0)]

    plan = " -> ".join(f"{b}{'(พัก)' if b in staged else ''}" for b in order)
    print(f"\n🗒️ ลำดับ: {plan}")

    def at(pos, z):
        return {"x": pos["x"], "y": pos["y"], "z": z, "r": pos["r"]}

    start = time.perf_counter()
    mover = Mover(device)

    # Phase 1: ย้ายบล็อก 1-4 ไปช่องพัก (ไล่ temp 4 -> 1)
    carry_z = safe_z_for(ground_z, block_h, 0)
    empty_z = safe_z_for(ground_z, block_h, 0, carrying=False)
    for b in order:
        if b in staged:
            print(f"📦 บล็อก {b} -> จุดพัก")
            mover.pick_and_place(at(grid[b], table_z), at(staged[b], table_z), carry_z, empty_z)

    # Phase 2: สร้าง Tower ที่ช่องกลาง (c)
    for layer, b in enumerate(order):
        src = staged.get(b, grid[b])
        tgt_z = ground_z + (layer + 1) * block_h + grip
        print(f"🏗️ บล็อก {b} -> Tower ชั้น {layer + 1}")
        mover.pick_and_place(at(src, table_z), at(center, tgt_z),
                             safe_z_for(ground_z, block_h, layer),
                             safe_z_for(ground_z, block_h, layer, carrying=False))
    mover.lift(safe_z_for(ground_z, block_h, len(order), carrying=False))
    mover.finish()
    print(f"🎉 สร้าง Tower เสร็จ {len(order)} ชั้น | ⏱️ {time.perf_counter() - start:.2f} sec")


# ==========================================
# 🎮 TEACH (4 จุด)
# ==========================================
TEACH_STEPS = [
    ("grid_1", "บล็อกช่อง 1 (บนซ้าย) — วางหัวดูดบน 'ผิวบนของบล็อก'"),
    ("grid_8", "ช่อง 8 (ล่างขวา) — วางหัวดูดแตะ 'พื้น'"),
    ("temp_top", "จุดพักช่องบนสุด (temp_1) — แตะ 'พื้น'"),
    ("temp_last", "จุดพักช่องล่างสุด (temp_4) — แตะ 'พื้น'"),
]


def teach_mode(device, settings):
    positions = settings["positions"]
    print("\n🎮 Teach 4 จุด (ที่เหลือคำนวณให้เอง) | [Enter]=บันทึกจุดนี้ | s=ข้าม | q=ออก")
    for key, label in TEACH_STEPS:
        old = positions.get(key)
        note = f" [เดิม: ({old['x']}, {old['y']})]" if old else ""
        cmd = input(f"👉 เลื่อนแขนไปที่ {label}{note} แล้วกด [Enter]: ").strip().lower()
        if cmd == "q":
            return
        if cmd == "s":
            continue
        p = device.get_pose().position
        positions[key] = {"x": round(p.x, 2), "y": round(p.y, 2),
                          "z": round(p.z, 2), "r": round(p.r, 2)}
        print(f"✅ {key}: {positions[key]}")

    # ช่อง 8 สอนที่พื้น -> ใช้เป็น Ground Z ได้เลย
    g8, g1 = positions.get("grid_8"), positions.get("grid_1")
    if g8:
        settings["ground_z"] = g8["z"]
        print(f"📏 ตั้ง Ground Z = {g8['z']:.2f} mm (จากช่อง 8)")
        if g1:
            print(f"   ความสูงบล็อกที่วัดได้จากช่อง 1 - ช่อง 8 = {g1['z'] - g8['z']:.2f} mm "
                  f"(ค่าที่ใช้อยู่ {settings['block_height']:.2f} mm)")
    show_layout(settings)


def set_ground(device, settings):
    input("👉 เลื่อนหัวดูดแตะพื้นโต๊ะ แล้วกด [Enter]...")
    settings["ground_z"] = round(device.get_pose().position.z, 2)
    print(f"✅ Ground Z = {settings['ground_z']:.2f} mm")
    h = input(f"ความสูงบล็อก [{settings['block_height']}] (Enter=ค่าเดิม): ").strip()
    if h:
        try:
            settings["block_height"] = round(float(h), 2)
        except ValueError:
            print("⚠️ ค่าไม่ถูกต้อง ใช้ค่าเดิม")


def main():
    settings = load_settings()
    device = connect_robot(settings)
    if not device:
        raise SystemExit(1)
    try:
        while True:
            choice = input(
                "\n[1]Run(กล้อง) [2]Teach&Save [3]SetGround [4]Save&Exit "
                "[5]ShowLayout [6]Run(กรอกเอง) [Enter]Exit > ").strip()
            if choice == "1":
                run_operation(device, settings)
            elif choice == "6":
                run_operation(device, settings, use_camera=False)
            elif choice == "2":
                teach_mode(device, settings)
            elif choice == "3":
                set_ground(device, settings)
            elif choice == "4":
                save_settings(settings)
                break
            elif choice == "5":
                show_layout(settings)
            elif choice == "":
                break
    except KeyboardInterrupt:
        print("\n🛑 หยุดฉุกเฉิน (Ctrl+C)")
    finally:
        try:
            emergency_stop(device)
            device.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
