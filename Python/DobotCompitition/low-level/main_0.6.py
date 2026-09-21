import os
import json
import time
import math
import struct
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
    # true = คงมุมหมุนตอนหยิบไว้จนวางเสร็จ บล็อกจะวางตรงแนวเดิมไม่หมุนตามแขน
    "keep_rotation": True,
    # ลำดับการวาง (1-4) ที่ต้องแวะพักที่ temp ก่อน / [] = ไม่ใช้ temp เลย
    "temp_orders": [3, 4],
    "order": [1, 2, 3, 4],
    "positions": {k: None for k in ("grid_1", "grid_3", "grid_8", "grid_6",
                                    "temp_top", "temp_last")},
}

SUCK_DELAY_MS = 50      # รอหัวดูดจับบล็อก
RELEASE_DELAY_MS = 100  # รอหัวดูดปล่อยบล็อก

# ผัง 3x3 เรียงซ้าย->ขวา, บน->ล่าง : [1,2,3 / 4,c,5 / 6,7,8]
GRID_LAYOUT = {1: (0, 0), 2: (0, 1), 3: (0, 2),
               4: (1, 0), "c": (1, 1), 5: (1, 2),
               6: (2, 0), 7: (2, 1), 8: (2, 2)}


def load_settings():
    s = dict(DEFAULT_SETTINGS)
    s["positions"] = dict(DEFAULT_SETTINGS["positions"])
    try:
        with open(SETTINGS_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
        s["positions"].update(data.pop("positions", None) or {})
        old = data.pop("use_temp", None)   # ไฟล์เก่าเก็บเป็น true/false -> แปลงเป็นรายการลำดับ
        if old is not None and "temp_orders" not in data:
            data["temp_orders"] = [3, 4] if old else []
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

    def pick_and_place(self, src, tgt, carry_z, empty_z, keep_rotation=True):
        # ขาไปตัวเปล่า (ไม่มีบล็อก) เดินที่ empty_z, ขาถือบล็อกเดินที่ carry_z
        # ไม่ยกขึ้นหลังวาง: การยกครั้งถัดไป (lift) จะยกตรงไปที่ empty_z ของบล็อกถัดไปในครั้งเดียว
        # keep_rotation: ใช้มุม r ตอนหยิบตลอดขาถือบล็อก บล็อกจึงวางลงตรงแนวเดิม
        # (ถ้าใช้ r ของจุดปลายทาง บล็อกจะถูกหมุนไปเท่ากับมุมที่แขนกวาดไป)
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
    """เว้นระยะ 0.5 บล็อกเหนือสิ่งกีดขวางที่สูงที่สุด (บล็อกบนโต๊ะสูง 1 ชั้น หรือ Tower)
    - ถือบล็อก : + ความสูงบล็อกที่ถืออีก 1 ชั้น -> 2.5x / 3.5x / 4.5x
    - ตัวเปล่า  : 1.5x / 2.5x / 3.5x / 4.5x"""
    obstacle = max(tower_height, 1)
    return base + (obstacle + (1.5 if carrying else 0.5)) * block_h


# ==========================================
# 📐 คำนวณพิกัดจากมุมกริดที่สอนไว้
# ==========================================
def point(x, y, z=None):
    """มุมหมุน r = atan2(y, x) เสมอ (ตรงกับค่าที่สอนไว้ทุกจุดในเวอร์ชันก่อน)
    z (ถ้ามี) = ระดับ "ผิวบนของบล็อก" ที่วางอยู่ตรงจุดนั้น"""
    p = {"x": round(x, 2), "y": round(y, 2),
         "r": round(math.degrees(math.atan2(y, x)), 2)}
    if z is not None:
        p["z"] = round(z, 2)
    return p


def lerp(a, b, t):
    return a + (b - a) * t


def solve3(rows):
    """แก้ระบบสมการ 3 ตัวแปรด้วย Gaussian elimination — rows = [[a, b, c, rhs], ...]"""
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
    """ระนาบ z = a + b*x + c*y จากจุดที่สอน (3 จุด = ผ่านพอดี, 4 จุด = least-squares)
    โต๊ะ/แขนกลไม่ได้ราบ 100% วัดจากของจริงได้ความชัน 3-4 mm ต่อระยะ 100 mm ในแกน x
    ถ้าใช้ z ค่าเดียวทั้งกระดานจะคลาดได้ถึง ~3 mm (ช่องไกลดูดไม่ติด / ช่องใกล้กดลงบล็อก)
    ใช้ normal equations เพราะมีแค่ 3 ตัวแปร ไม่ต้องพึ่ง numpy"""
    pts = [p for p in pts if p and p.get("z") is not None]
    if len(pts) < 3:
        return None
    basis = [(1.0, p["x"], p["y"]) for p in pts]
    rows = [[sum(b[i] * b[j] for b in basis) for j in range(3)]
            + [sum(b[i] * p["z"] for b, p in zip(basis, pts))] for i in range(3)]
    return solve3(rows)


REQUIRED_CORNERS = ("grid_1", "grid_3", "grid_8")   # บนซ้าย, บนขวา, ล่างขวา
OPTIONAL_CORNER = "grid_6"                          # ล่างซ้าย (ใส่เพิ่มเพื่อความแม่นยำ)
GRID_CORNERS = REQUIRED_CORNERS + (OPTIONAL_CORNER,)


def build_grid(positions):
    """คำนวณ 9 ช่องด้วยเวกเตอร์ 2 ทิศ (affine): ตำแหน่ง = จุดเริ่ม + คอลัมน์*u + แถว*v
    จึงรองรับตารางที่วางเอียงจากแกน X/Y ของหุ่นยนต์ได้

    - 3 มุม (grid_1, grid_3, grid_8): คำนวณตรงๆ ผ่านทั้งสามจุดพอดี
    - 4 มุม (มี grid_6 ด้วย): ใช้ least-squares เฉลี่ยความคลาดจากการสอนทั้งสี่มุม
      (มุมทั้งสี่อยู่ที่ (แถว,คอลัมน์) = (0,0),(0,2),(2,0),(2,2) เป็นดีไซน์สมดุล จึงมีสูตรปิด)

    z ของแต่ละช่องมาจากระนาบที่ฟิตจาก z ของมุมที่สอนไว้ (ทุกมุมสอนที่ผิวบนบล็อก)"""
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
    cells = {}
    for name, (row, col) in GRID_LAYOUT.items():
        x, y = ox + col * ux + row * vx, oy + col * uy + row * vy
        cells[name] = point(x, y, None if plane is None
                            else plane[0] + plane[1] * x + plane[2] * y)
    return cells


def warn_missing_grid(positions):
    """บอกให้ชัดว่าขาดมุมไหน และถ้าเป็นค่าเก่าแบบ 2 จุดก็บอกว่าต้องสอนใหม่"""
    missing = [k for k in REQUIRED_CORNERS if not positions.get(k)]
    old_style = (positions.get("grid_1") and positions.get("grid_8")
                 and not positions.get("grid_3") and not positions.get(OPTIONAL_CORNER))
    if old_style:
        print("❌ ค่าที่สอนไว้เป็นแบบเก่า (2 มุม) ใช้กับการคำนวณแบบใหม่ไม่ได้")
        print("   ตอนนี้ต้องสอน 3 มุม: grid_1 (บนซ้าย), grid_3 (บนขวา), grid_8 (ล่างขวา)")
        print("   กด [6] ResetPositions แล้ว [2] Teach ใหม่")
    else:
        print(f"❌ ยังสอนมุมกริดไม่ครบ ขาด: {', '.join(missing)} — ใช้ [2] Teach ก่อน")


def build_temps(positions, block_h):
    """คำนวณจุดพัก 4 ช่อง จากช่องบนสุด (temp_top) ถึงช่องล่างสุด (temp_last)
    สองจุดนี้สอนที่ "พื้น" จึงบวกความสูงบล็อกกลับเข้าไป ให้ z มีความหมายเดียวกับของกริด
    (= ผิวบนของบล็อกที่วางตรงนั้น) และเอียงตามพื้นจริงระหว่างสองจุด"""
    a, b = positions.get("temp_top"), positions.get("temp_last")
    if not (a and b):
        return None
    has_z = a.get("z") is not None and b.get("z") is not None
    return {i: point(lerp(a["x"], b["x"], (i - 1) / 3), lerp(a["y"], b["y"], (i - 1) / 3),
                     lerp(a["z"], b["z"], (i - 1) / 3) + block_h if has_z else None)
            for i in range(1, 5)}


MIN_BLOCK_H = 8.0   # ต่ำกว่านี้แปลว่ามุมกริดถูกสอนที่พื้น ไม่ใช่ผิวบนบล็อก


def measured_block_height(positions):
    """ความสูงบล็อกที่วัดได้ = ผิวบนเฉลี่ยของมุมกริด - พื้นเฉลี่ยของจุดพัก"""
    tops = [positions[k]["z"] for k in GRID_CORNERS
            if positions.get(k) and positions[k].get("z") is not None]
    floors = [positions[k]["z"] for k in ("temp_top", "temp_last")
              if positions.get(k) and positions[k].get("z") is not None]
    if not tops or not floors:
        return None
    return sum(tops) / len(tops) - sum(floors) / len(floors)


def check_z_scheme(positions, settings):
    """กันเคสอันตราย: ถ้ามุมกริดถูกสอนที่ "พื้น" แบบเวอร์ชันก่อน z ที่คำนวณได้จะต่ำไปทั้งกระดาน
    แขนจะกดลงโต๊ะ — คืน (ข้อความ, ต้องหยุดไหม) หรือ None ถ้าปกติ"""
    h = measured_block_height(positions)
    if h is None:
        return None
    if h < MIN_BLOCK_H:
        return (f"❌ ผิวบนของมุมกริดสูงกว่าพื้นแค่ {h:.2f} mm — มุมกริดน่าจะถูกสอนที่ 'พื้น'\n"
                f"   ตอนนี้ต้องสอนทั้ง 4 มุมที่ 'ผิวบนของบล็อก' กด [6] ResetPositions แล้ว [2] Teach ใหม่",
                True)
    diff = h - settings.get("block_height", 25.0)
    if abs(diff) > 2.0:
        return (f"⚠️ block_height ที่ตั้งไว้ {settings.get('block_height', 25.0):.2f} mm "
                f"ต่างจากที่วัดได้ {h:.2f} mm ({diff:+.2f}) — สอนใหม่ด้วย [2] Teach จะอัปเดตให้เอง",
                False)
    return None


def ask_order(settings):
    """เลือกลำดับบล็อก 4 ตัวจากทั้งหมด 8 ตัว"""
    cur = " ".join(str(b) for b in settings.get("order", []))
    raw = input(f"เลือกลำดับบล็อก 4 ตัว (1-8) เช่น '3 5 8 2' [{cur}] (Enter=ค่าเดิม): ").strip()
    if not raw:
        raw = cur
    try:
        order = [int(t) for t in raw.replace(",", " ").split()]
    except ValueError:
        order = []
    if len(order) != 4 or len(set(order)) != 4 or any(b < 1 or b > 8 for b in order):
        print("❌ ต้องเป็นเลข 1-8 จำนวน 4 ตัว ไม่ซ้ำกัน")
        return None
    settings["order"] = order
    return order


LAYOUT_DWELL_MS = 500   # ค้างที่แต่ละจุดตอนเดินตรวจตำแหน่ง


def walk_layout(device, settings, grid, temps):
    """เดินหัวดูดไปทีละจุด (1,2,3,4,c,5,6,7,8 แล้วต่อด้วย temp 1-4) ค้างจุดละ 0.5 วิ
    เพื่อดูว่าพิกัดที่คำนวณไว้ตรงกับของจริงไหม"""
    ground_z = settings.get("ground_z")
    if ground_z is None:
        print("❌ ยังไม่ได้ตั้ง Ground ([3]) เดินตรวจตำแหน่งไม่ได้")
        return
    block_h = settings.get("block_height", 25.0)
    grip = settings.get("grip_offset", 0.0)
    fallback_top = ground_z + block_h          # ใช้เมื่อฟิตระนาบไม่ได้ (ข้อมูลเก่าไม่มี z)
    base_z = grid["c"].get("z", fallback_top) - block_h
    hover_z = safe_z_for(base_z, block_h, 0, carrying=False)

    stops = [(f"ช่อง {n}", grid[n]) for n in (1, 2, 3, 4, "c", 5, 6, 7, 8)]
    stops += [(f"temp_{i}", temps[i]) for i in range(1, 5)] if temps else []
    print(f"🚶 เดินตรวจ {len(stops)} จุด (ลงไปแตะผิวบนบล็อกของแต่ละจุด ค้างจุดละ "
          f"{LAYOUT_DWELL_MS / 1000:.1f} วิ) — Ctrl+C เพื่อหยุด")

    mover = Mover(device)
    for label, p in stops:
        table_z = p.get("z", fallback_top) + grip
        print(f"   -> {label}: ({p['x']:.2f}, {p['y']:.2f}) z={table_z:.2f} r={p['r']:.2f}")
        mover.lift(hover_z)
        mover.move(p["x"], p["y"], hover_z, p["r"])
        mover.move(p["x"], p["y"], table_z, p["r"])
        mover.last = queued_wait(device, LAYOUT_DWELL_MS)
        device.wait_for_cmd(mover.last)
    mover.lift(hover_z)
    mover.finish()
    print("✅ เดินตรวจครบทุกจุดแล้ว")


def show_layout(settings, device=None):
    """แสดงพิกัดที่คำนวณได้ทั้งหมด และเลือกให้แขนเดินไล่ทุกจุดเพื่อตรวจได้"""
    grid = build_grid(settings["positions"])
    temps = build_temps(settings["positions"], settings.get("block_height", 25.0))
    if not grid:
        warn_missing_grid(settings["positions"])
        return
    warn = check_z_scheme(settings["positions"], settings)
    if warn:
        print(warn[0])
    mode = "4 มุม (least-squares)" if settings["positions"].get(OPTIONAL_CORNER) else "3 มุม"
    print(f"\n📋 กริด 3x3 (ซ้าย->ขวา, บน->ล่าง) — คำนวณจาก {mode}:")
    for row in ([1, 2, 3], [4, "c", 5], [6, 7, 8]):
        print("   " + " | ".join(
            f"{n}: ({grid[n]['x']:7.2f},{grid[n]['y']:7.2f}) r={grid[n]['r']:6.2f}" for n in row))
    zs = [grid[n].get("z") for n in GRID_LAYOUT]
    if None not in zs:
        print(f"📐 ผิวบนบล็อกแต่ละช่อง (จากระนาบที่ฟิตไว้ ต่างกันได้ {max(zs) - min(zs):.2f} mm):")
        for row in ([1, 2, 3], [4, "c", 5], [6, 7, 8]):
            print("   " + " | ".join(f"{n}: {grid[n]['z']:7.2f}" for n in row))
    else:
        print("⚠️ ข้อมูลที่สอนไม่มี z ครบ — ใช้ ground_z + block_height เท่ากันทุกช่องแทน")
    if temps:
        print("📋 จุดพัก (บน->ล่าง):")
        for i in range(1, 5):
            z = f" z={temps[i]['z']:7.2f}" if temps[i].get("z") is not None else ""
            print(f"   temp_{i}: ({temps[i]['x']:7.2f},{temps[i]['y']:7.2f}) r={temps[i]['r']:6.2f}{z}")
    else:
        print("⚠️ ยังไม่ได้สอนจุดพัก (temp_top / temp_last)")

    if device is not None and input("เดินตรวจตำแหน่งจริงด้วยแขนกลไหม? (y/n): ").strip().lower() == "y":
        walk_layout(device, settings, grid, temps)


# ==========================================
# 🚀 RUN
# ==========================================
def run_operation(device, settings):
    positions = settings["positions"]
    grid = build_grid(positions)
    temps = build_temps(positions, settings.get("block_height", 25.0))
    temp_orders = settings.get("temp_orders", [3, 4])
    if not grid:
        warn_missing_grid(positions)
        return
    warn = check_z_scheme(positions, settings)
    if warn:
        print(warn[0])
        if warn[1]:
            return
    if temp_orders and not temps:
        print("❌ ยังไม่ได้สอนจุดพัก (temp_top / temp_last) — ใช้ [2] Teach หรือปิด temp ที่เมนู [7]")
        return
    ground_z = settings.get("ground_z")
    if ground_z is None:
        print("❌ ยังไม่ได้ตั้ง Ground ใช้ [3] SetGround ก่อน")
        return

    order = ask_order(settings)
    if not order:
        return

    block_h = settings.get("block_height", 25.0)
    grip = settings.get("grip_offset", 0.0)
    keep_rot = settings.get("keep_rotation", True)
    center = grid["c"]
    fallback_top = ground_z + block_h     # ใช้เมื่อฟิตระนาบไม่ได้ (ข้อมูลเก่าไม่มี z)

    def top_of(pos):
        """ระดับที่หัวดูดต้องลงไปแตะผิวบนบล็อกตรงจุดนั้น (ไม่เท่ากันทุกช่องเพราะโต๊ะเอียง)"""
        return pos.get("z", fallback_top) + grip

    base_z = center.get("z", fallback_top) - block_h   # พื้นใต้ Tower ใช้อ้างอิงความสูงเดินทาง

    # ลำดับการวางที่เลือกไว้ในเมนู [7] จะถูกย้ายไปพักที่ temp ก่อน (ค่าเริ่มต้น 3,4)
    # เพราะตอนวางชั้นท้ายๆ Tower สูง 2-3 ชั้นแล้ว เสี่ยงชนตอนเอื้อมข้ามไปหยิบจากช่องเดิม
    # ช่องพักไล่จาก 4 ไป 1 เพื่อไม่ให้ชนกันเอง
    staged = {}
    free_slots = [4, 3, 2, 1]
    for i, b in enumerate(order):
        if (i + 1) in temp_orders:
            staged[b] = temps[free_slots.pop(0)]
    if not staged:
        print("⚠️ ไม่ได้ใช้จุดพัก (temp) — หยิบจากช่องเดิมไปวาง Tower ตรงๆ")

    plan = " -> ".join(f"{b}{'(พัก)' if b in staged else ''}" for b in order)
    print(f"\n🗒️ ลำดับ: {plan}")

    def at(pos, z):
        return {"x": pos["x"], "y": pos["y"], "z": z, "r": pos["r"]}

    start = time.perf_counter()
    mover = Mover(device)

    # Phase 1: ย้ายบล็อก 1-4 ไปช่องพัก (ไล่ temp 4 -> 1)
    carry_z = safe_z_for(base_z, block_h, 0)
    empty_z = safe_z_for(base_z, block_h, 0, carrying=False)
    for b in order:
        if b in staged:
            print(f"📦 บล็อก {b} -> จุดพัก")
            mover.pick_and_place(at(grid[b], top_of(grid[b])), at(staged[b], top_of(staged[b])),
                                 carry_z, empty_z, keep_rot)

    # Phase 2: สร้าง Tower ที่ช่องกลาง (c)
    # ชั้นที่ 1 วางที่ผิวบนบล็อกของช่อง c พอดี (= พื้นตรงนั้น + 1 บล็อก) แล้วบวกทีละชั้น
    for layer, b in enumerate(order):
        src = staged.get(b, grid[b])
        tgt_z = top_of(center) + layer * block_h
        print(f"🏗️ บล็อก {b} -> Tower ชั้น {layer + 1}")
        mover.pick_and_place(at(src, top_of(src)), at(center, tgt_z),
                             safe_z_for(base_z, block_h, layer),
                             safe_z_for(base_z, block_h, layer, carrying=False), keep_rot)
    mover.lift(safe_z_for(base_z, block_h, len(order), carrying=False))
    mover.finish()
    print(f"🎉 สร้าง Tower เสร็จ {len(order)} ชั้น | ⏱️ {time.perf_counter() - start:.2f} sec")


# ==========================================
# 🎮 TEACH (6 จุด)
# ==========================================
# มุมกริดทั้ง 4 สอนที่ "ผิวบนของบล็อก" เหมือนกันหมด (วางบล็อกไว้ที่ช่อง 1, 3, 6, 8 ก่อนสอน)
# ระดับพื้น (ground) เอามาจากจุดพัก 2 จุด ซึ่งเป็นพื้นที่ว่างจึงแตะพื้นได้จริง
TEACH_STEPS = [
    ("grid_1", "บล็อกช่อง 1 (บนซ้าย) — วางหัวดูดบน 'ผิวบนของบล็อก'"),
    ("grid_3", "บล็อกช่อง 3 (บนขวา) — วางหัวดูดบน 'ผิวบนของบล็อก'"),
    ("grid_8", "บล็อกช่อง 8 (ล่างขวา) — วางหัวดูดบน 'ผิวบนของบล็อก'"),
    ("grid_6", "บล็อกช่อง 6 (ล่างซ้าย) — วางหัวดูดบน 'ผิวบนของบล็อก'"),
    ("temp_top", "จุดพักช่องบนสุด (temp_1) — วางหัวดูดแตะ 'พื้น'"),
    ("temp_last", "จุดพักช่องล่างสุด (temp_4) — วางหัวดูดแตะ 'พื้น'"),
]


def teach_mode(device, settings):
    positions = settings["positions"]
    print("\n🎮 Teach 6 จุด: มุมกริด 4 จุด (ผิวบนบล็อก) + จุดพัก 2 จุด (พื้น) ที่เหลือคำนวณให้เอง")
    print("   [Enter]=บันทึกจุดนี้ | s=ข้าม | q=ออก")
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

    # จุดพักสอนที่พื้น -> ใช้หาระดับพื้น (เฉลี่ย 2 จุดกันพื้นเอียง)
    t1, t4 = positions.get("temp_top"), positions.get("temp_last")
    if t1 and t4:
        settings["ground_z"] = round((t1["z"] + t4["z"]) / 2, 2)
        print(f"📏 ตั้ง Ground Z = {settings['ground_z']:.2f} mm "
              f"(เฉลี่ยจากจุดพัก {t1['z']:.2f} / {t4['z']:.2f})")
        h = measured_block_height(positions)
        if h is None:
            pass
        elif h < MIN_BLOCK_H:
            print(f"⚠️ ผิวบนของมุมกริดสูงกว่าพื้นแค่ {h:.2f} mm "
                  f"— มุมกริดต้องสอนที่ 'ผิวบนของบล็อก' ไม่ใช่พื้น")
        else:
            old = settings.get("block_height", 25.0)
            settings["block_height"] = round(h, 2)
            print(f"📦 ความสูงบล็อกที่วัดได้ = {h:.2f} mm (เดิม {old:.2f}) -> อัปเดตให้แล้ว")
    else:
        print("⚠️ ยังไม่ได้สอนจุดพักครบ 2 จุด จึงยังไม่ได้ตั้ง Ground — ใช้ [3] SetGround แทนได้")
    show_layout(settings)


def configure_temp(settings):
    """เลือกว่า 'ลำดับการวาง' ไหนบ้างที่ต้องแวะพักที่ temp ก่อน
    เช่น '2 3 4' = ลำดับ 2,3,4 พัก | '4' = พักแค่ลำดับสุดท้าย | Enter เฉยๆ = ไม่ใช้ temp เลย"""
    cur = settings.get("temp_orders", [3, 4])
    print("เลขที่กรอกคือ 'ลำดับการวาง' (1-4) ไม่ใช่หมายเลขช่อง")
    print("   ลำดับท้ายๆ เสี่ยงชน Tower ตอนเอื้อมข้ามไปหยิบ ลำดับ 1-2 มักไม่ต้องพัก")
    raw = input(f"ลำดับที่ให้แวะพัก temp เช่น '2 3 4' หรือ '4' [ตอนนี้: "
                f"{','.join(map(str, cur)) or 'ไม่ใช้'}] (Enter=ไม่ใช้เลย, q=ยกเลิก): ").strip()
    if raw.lower() == "q":
        print("ยกเลิก ไม่เปลี่ยนค่าเดิม")
        return
    if not raw:
        settings["temp_orders"] = []
        print("⛔ ปิดการใช้จุดพัก: ทุกลำดับหยิบจากช่องเดิมไปวาง Tower ตรงๆ "
              "(เสี่ยงชน Tower ตอนเอื้อมข้าม)")
        return
    try:
        picked = sorted({int(t) for t in raw.replace(",", " ").split()})
    except ValueError:
        picked = []
    if not picked or picked[0] < 1 or picked[-1] > 4:
        print("❌ ต้องเป็นเลข 1-4 เท่านั้น ไม่เปลี่ยนค่าเดิม")
        return
    settings["temp_orders"] = picked
    print(f"✅ ลำดับที่จะแวะพักที่ temp: {', '.join(map(str, picked))}")


def reset_positions(settings):
    """ล้างพิกัดที่สอนไว้ทั้งหมด (มุมกริด 4 จุด + จุดพัก 2 จุด) เพื่อเริ่มสอนใหม่
    ค่าอื่น เช่น ground_z / ความเร็ว ไม่ถูกแตะ"""
    filled = [k for k, v in settings["positions"].items() if v]
    if not filled:
        print("ℹ️ ยังไม่มีพิกัดที่บันทึกไว้ ไม่ต้องล้าง")
        return
    print(f"พิกัดที่มีอยู่ ({len(filled)}): {', '.join(filled)}")
    if input("ล้างพิกัดทั้งหมดใช่ไหม? (y/n): ").strip().lower() != "y":
        print("ยกเลิก")
        return
    settings["positions"] = {k: None for k in settings["positions"]}
    print("🧹 ล้างพิกัดทั้งหมดแล้ว (กด [4] Save&Exit เพื่อเขียนลงไฟล์)")


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
            temp_state = ",".join(map(str, settings.get("temp_orders", []))) or "OFF"
            choice = input(
                "\n[1]Run [2]Teach&Save [3]SetGround [4]Save&Exit [5]ShowLayout "
                f"[6]ResetPositions [7]Temp:{temp_state} [Enter]Exit > ").strip()
            if choice == "1":
                run_operation(device, settings)
            elif choice == "2":
                teach_mode(device, settings)
            elif choice == "3":
                set_ground(device, settings)
            elif choice == "4":
                save_settings(settings)
                break
            elif choice == "5":
                show_layout(settings, device)
            elif choice == "6":
                reset_positions(settings)
            elif choice == "7":
                configure_temp(settings)
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
