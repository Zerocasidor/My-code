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
    # true = คงมุมหมุนตอนหยิบไว้จนวางเสร็จ บล็อกจะวางตรงแนวเดิมไม่หมุนตามแขน
    "keep_rotation": True,
    # ลำดับการวาง (1-4) ที่ต้องแวะพักที่ temp ก่อน / [] = ไม่ใช้ temp เลย
    "temp_orders": [3, 4],
    "order": [1, 2, 3, 4],
    "colors": ["g", "r", "y", "b"],
    "last_input": "g r y b",   # ลำดับที่กรอกล่าสุด (เลขช่องหรือสี) ใช้เป็นค่าเริ่มต้นครั้งถัดไป
    "flip_camera": True,   # กล้องอยู่ฝั่งตรงข้ามหุ่น ภาพจึงกลับด้าน 180°
    "positions": {k: None for k in ("grid_1", "grid_3", "grid_8", "grid_6",
                                    "temp_top", "temp_last")},
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
            if input("เปิดโหมดจำลอง (ไม่มีอะไรขยับจริง) แทนไหม? (y/n): ").strip().lower() == "y":
                return FakeDobot()
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


class FakePosition:
    def __init__(self, x, y, z, r):
        self.x, self.y, self.z, self.r = x, y, z, r


class FakePose:
    def __init__(self, position):
        self.position = position


class FakeDobot:
    """โหมดจำลอง — ใช้ตอนไม่มีหุ่นต่ออยู่ ไม่มีอะไรขยับจริง
    พิมพ์คำสั่งที่จะถูกส่งเข้าคิวออกมาแทน ตรวจลำดับ/พิกัด/กล้องได้ครบ
    ยกเว้น [2] Teach กับ [3] SetGround ที่ต้องอ่านตำแหน่งจริงจากแขน"""

    HOME = (200.0, 0.0, 0.0, 0.0)

    def __init__(self):
        self.pos = list(self.HOME)
        self.index = 0

    def _queue(self, text):
        self.index += 1
        print(f"   [จำลอง {self.index:3d}] {text}")
        return self.index

    def get_pose(self):
        return FakePose(FakePosition(*self.pos))

    def move_to(self, x, y, z, r, mode=None):
        self.pos = [x, y, z, r]
        return self._queue(f"move   x={x:7.2f}  y={y:7.2f}  z={z:7.2f}  r={r:7.2f}")

    def suck(self, on):
        return self._queue(f"suck   {'ON' if on else 'OFF'}")

    def speed(self, *args):
        pass

    def wait_for_cmd(self, index):
        pass

    def close(self):
        pass

    # ให้ queued_wait / emergency_stop เรียกได้เหมือนของจริง
    def _send_command(self, msg):
        if msg.id == 110:
            self._queue(f"wait   {struct.unpack('I', bytes(msg.params))[0]} ms")
        return msg

    def _extract_cmd_index(self, msg):
        return self.index

    def _set_queued_cmd_start_exec(self):
        pass


def is_sim(device):
    return isinstance(device, FakeDobot)


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


def measured_block_height(positions, ground_z=None):
    """ความสูงบล็อกที่วัดได้ = ผิวบนเฉลี่ยของมุมกริด - ระดับพื้น
    ระดับพื้นเอาจากจุดพัก 2 จุดก่อน (แตะพื้นจริง เฉลี่ยกันพื้นเอียง)
    ถ้ายังไม่ได้สอนจุดพัก ใช้ ground_z แทนได้ (หยาบกว่า แต่ยังพอใช้กันหัวดูดกดลงโต๊ะ)"""
    tops = [positions[k]["z"] for k in GRID_CORNERS
            if positions.get(k) and positions[k].get("z") is not None]
    floors = [positions[k]["z"] for k in ("temp_top", "temp_last")
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
    """กันเคสอันตราย: ถ้ามุมกริดถูกสอนที่ "พื้น" แบบเวอร์ชันก่อน z ที่คำนวณได้จะต่ำไปทั้งกระดาน
    แขนจะกดลงโต๊ะ — คืน (ข้อความ, ต้องหยุดไหม) หรือ None ถ้าปกติ

    1.11: เดิมเช็คได้เฉพาะตอนสอนจุดพักไว้แล้ว ถ้าปิด temp และยังไม่ได้สอนจุดพัก
    จะไม่มีการตรวจเลย (ด่านหลุด) ตอนนี้ถอยไปใช้ ground_z ซึ่ง run_operation บังคับให้มีอยู่แล้ว"""
    exact = measured_block_height(positions)            # วัดจากจุดพัก = แม่น
    h = exact if exact is not None else measured_block_height(positions, settings.get("ground_z"))
    if h is None:
        return None
    if h < MIN_BLOCK_H:
        return (f"❌ ผิวบนของมุมกริดสูงกว่าพื้นแค่ {h:.2f} mm — มุมกริดน่าจะถูกสอนที่ 'พื้น'\n"
                f"   ตอนนี้ต้องสอนทั้ง 4 มุมที่ 'ผิวบนของบล็อก' กด [6] ResetPositions แล้ว [2] Teach ใหม่",
                True)
    if exact is None:
        # เทียบกับ ground_z จุดเดียว ความละเอียดไม่พอจะฟันธงเรื่อง block_height (โต๊ะเอียง ~2-3 mm)
        return None
    diff = h - settings.get("block_height", 25.0)
    if abs(diff) > 2.0:
        return (f"⚠️ block_height ที่ตั้งไว้ {settings.get('block_height', 25.0):.2f} mm "
                f"ต่างจากที่วัดได้ {h:.2f} mm ({diff:+.2f}) — สอนใหม่ด้วย [2] Teach จะอัปเดตให้เอง",
                False)
    return None


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


def valid_colors(colors):
    """ลำดับสีที่ใช้ได้: g/r/y/b จำนวน 4 ตัว สีเดียวกันซ้ำได้ไม่เกิน 2 (บนโต๊ะมีสีละ 2 ก้อน)"""
    colors = [str(c).lower() for c in colors]
    if len(colors) != 4 or any(c not in COLORS for c in colors):
        return None
    if any(colors.count(c) > 2 for c in colors):
        print("❌ สีเดียวกันซ้ำได้ไม่เกิน 2 (บนโต๊ะมีสีละ 2 ก้อน)")
        return None
    return colors


def ask_input(settings, numbers_only=False, enter_hint=None):
    """รับลำดับ 4 ตัว ได้ทั้ง 'เลขช่อง' (1-8 ไม่ซ้ำ) และ 'สี' (g/r/y/b ซ้ำได้ไม่เกินสีละ 2)
    คืน ("order", [ช่อง...]) หรือ ("colors", [สี...]) หรือ None"""
    # 1.11: โหมด numbers_only (fallback ตอนกล้องพัง) ต้องเสนอ "ลำดับเลข" เป็นค่าเริ่มต้น
    # เดิมใช้ last_input ซึ่งตอนนั้นเป็นชุดสี -> หน้าจอบอก Enter=ค่าเดิม แต่กด Enter แล้วพังแน่นอน
    default_order = " ".join(str(b) for b in settings.get("order", []))
    cur = default_order if numbers_only else (settings.get("last_input") or default_order)
    legend = ", ".join(f"{k}={v}" for k, v in COLORS.items())
    prompt = ("ลำดับ 4 ตัว — เลขช่อง 1-8 เช่น '3 5 8 2'"
              + ("" if numbers_only else f" หรือสี ({legend}) เช่น 'g r y b'")
              + f" [{cur}] ({enter_hint or 'Enter=ค่าเดิม'}): ")
    tokens = (input(prompt).strip().lower() or cur).replace(",", " ").split()

    if tokens and all(t.isdigit() for t in tokens):
        order = valid_order(tokens)
        if not order:
            print("❌ เลขช่องต้องเป็น 1-8 จำนวน 4 ตัว ไม่ซ้ำกัน")
            return None
        settings["last_input"] = " ".join(str(b) for b in order)
        settings["order"] = order
        return "order", order

    if numbers_only:
        print("❌ ตอนนี้กล้องใช้ไม่ได้ ต้องกรอกเป็นเลขช่อง 1-8 จำนวน 4 ตัว")
        return None
    colors = valid_colors(tokens)
    if not colors:
        print(f"❌ ต้องเป็นเลขช่อง 1-8 จำนวน 4 ตัว หรือสี {' / '.join(COLORS)} จำนวน 4 ตัว")
        return None
    settings["last_input"] = " ".join(colors)
    settings["colors"] = colors
    return "colors", colors


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


def get_order(settings, grid):
    """กรอกเป็นเลขช่อง = ใช้ตรงๆ (ไม่แตะกล้อง) / กรอกเป็นสี = ถามกล้องว่าสีไหนอยู่ช่องไหน
    ถ้ากล้องใช้ไม่ได้ จะให้กรอกใหม่เป็นเลขช่อง"""
    answer = ask_input(settings)
    if not answer:
        return None
    kind, value = answer
    if kind == "order":
        return value

    colors = value
    blocks = blocks_from_camera(settings)
    if blocks:
        order = order_from_colors(colors, blocks, grid)
        if order:
            print("🎨 ลำดับที่ได้: " + " -> ".join(
                f"{c}(ช่อง {b})" for c, b in zip(colors, order)))
            settings["order"] = order
            return order
    print("↩️ กล้องใช้ไม่ได้ กรอกเป็นเลขช่องแทน")
    answer = ask_input(settings, numbers_only=True)
    return answer[1] if answer else None


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
    """แสดงพิกัดที่คำนวณได้ทั้งหมด และเลือกให้แขนเดินไล่ทุกจุดเพื่อตรวจได้ (ยกมาจาก main_1.00)"""
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

    if device is None:
        return
    if warn and warn[1]:
        # ด่านเดียวกับ [1] Run — ห้ามเอาแขนลงไปตามค่า z ที่รู้อยู่แล้วว่าผิด
        print("⛔ ไม่เดินตรวจด้วยแขนจริง เพราะข้อมูล z ยังไม่ถูกต้อง (ดูข้อความข้างบน)")
        return
    if input("เดินตรวจตำแหน่งจริงด้วยแขนกลไหม? (y/n): ").strip().lower() == "y":
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

    order = get_order(settings, grid)
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


def unsaved_changes(settings):
    """ค่าที่ถืออยู่ในโปรแกรมต่างจากในไฟล์ไหม (เทียบกับผลของ load_settings() เพื่อให้รูปแบบตรงกัน)
    เมนูและ [8] ใช้ตัวนี้บอกว่ากด Enter แล้วจะ Save หรือ Exit"""
    try:
        return settings != load_settings()
    except Exception:
        return True


def save_order(settings):
    """[8] กรอกลำดับแล้วบันทึกลงไฟล์ทันที (ไม่ต้องออกโปรแกรม)
    - กรอกเป็นเลข 1-8 -> เก็บเป็น "ลำดับเลข" ตอนรันใช้ช่องนั้นตรงๆ ไม่แตะกล้อง
    - กรอกเป็นสี g/r/y/b -> เก็บเป็น "ลำดับสี" ตอนรันจะถามกล้องว่าสีไหนอยู่ช่องไหน
    - กด Enter เฉยๆ -> ใช้ค่าเดิมที่ค้างอยู่ แล้วบันทึก"""
    answer = ask_input(settings,
                       enter_hint="Enter=Save" if unsaved_changes(settings) else "Enter=Exit")
    if not answer:
        print("↩️ ยังไม่บันทึก (ลำดับไม่ถูกต้อง)")
        return
    if not unsaved_changes(settings):
        print("ℹ️ ค่าในโปรแกรมตรงกับในไฟล์อยู่แล้ว ไม่ต้องบันทึกซ้ำ")
        return
    kind, value = answer
    if kind == "order":
        print(f"🔢 ลำดับเลข: {' '.join(str(b) for b in value)} — ตอนรันใช้ช่องนี้ตรงๆ ไม่ใช้กล้อง")
    else:
        names = ", ".join(COLORS[c] for c in value)
        print(f"🎨 ลำดับสี: {' '.join(value)} ({names}) — ตอนรันจะถามกล้องว่าสีไหนอยู่ช่องไหน")
    save_settings(settings)


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
    # รันด้วย --sim เพื่อเข้าโหมดจำลองเลย โดยไม่ต้องแตะพอร์ตของหุ่น
    device = FakeDobot() if "--sim" in sys.argv else connect_robot(settings)
    if not device:
        raise SystemExit(1)
    if is_sim(device):
        print("🧪 โหมดจำลอง: ไม่มีอะไรขยับจริง — ตรวจลำดับ/พิกัด/กล้องได้ "
              "แต่ [2] Teach กับ [3] SetGround ใช้ไม่ได้")
    try:
        while True:
            temp_state = ",".join(map(str, settings.get("temp_orders", []))) or "OFF"
            # โชว์ลำดับที่บันทึกไว้ และถ้าอยู่โหมดสี ให้โชว์ "เลขสำรอง" (settings["order"]) ด้วย
            # เพราะตอนกล้องพัง fallback จะเสนอชุดเลขนี้ ถ้ามันเก่าจะได้เห็นตั้งแต่ตอนตั้งค่า
            # ไม่ใช่ไปเจอตอนกดดันที่สุด (ลำดับเลขผิดจะรันจนจบโดยไม่ error)
            numbers = " ".join(str(b) for b in settings.get("order", []))
            primary = settings.get("last_input") or numbers
            is_colors = bool(primary) and not primary.replace(" ", "").isdigit()
            backup = ((f" (เลขสำรอง {numbers})" if numbers else " (ยังไม่มีเลขสำรอง)")
                      if is_colors else "")
            dirty = " ⚠️ยังไม่บันทึก" if unsaved_changes(settings) else ""
            tag = "🧪SIM " if is_sim(device) else ""
            choice = input(
                f"\n{tag}[1]Run [2]Teach&Save [3]SetGround [4]Save&Exit [5]ShowLayout\n"
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
                    print("❌ โหมดจำลองอ่านตำแหน่งจริงของแขนไม่ได้ — ต่อหุ่นก่อนถึงจะ Teach ได้")
                else:
                    teach_mode(device, settings)
            elif choice == "3":
                if is_sim(device):
                    print("❌ โหมดจำลองอ่านตำแหน่งจริงของแขนไม่ได้ — ต่อหุ่นก่อนถึงจะตั้ง Ground ได้")
                else:
                    set_ground(device, settings)
            elif choice == "4":
                save_settings(settings)
                break
            elif choice == "5":
                show_layout(settings, device)
            elif choice == "8":
                save_order(settings)
            elif choice == "":
                break
    except KeyboardInterrupt:
        print("\n🛑 หยุดฉุกเฉิน (Ctrl+C)")
    finally:
        try:
            if not is_sim(device):
                emergency_stop(device)
            device.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
