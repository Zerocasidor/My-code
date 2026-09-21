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
    # true = ลำดับที่ 3-4 แวะพักที่ temp ก่อน / false = หยิบจากช่องเดิมไปวาง Tower ตรงๆ
    "use_temp": True,
    "order": [1, 2, 3, 4],
    "positions": {k: None for k in ("grid_1", "grid_8", "temp_top", "temp_last")},
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
    table_z = ground_z + block_h + settings.get("grip_offset", 0.0)
    hover_z = safe_z_for(ground_z, block_h, 0, carrying=False)

    stops = [(f"ช่อง {n}", grid[n]) for n in (1, 2, 3, 4, "c", 5, 6, 7, 8)]
    stops += [(f"temp_{i}", temps[i]) for i in range(1, 5)] if temps else []
    print(f"🚶 เดินตรวจ {len(stops)} จุด (ลงไปที่ระดับหยิบ {table_z:.2f} mm ค้างจุดละ "
          f"{LAYOUT_DWELL_MS / 1000:.1f} วิ) — Ctrl+C เพื่อหยุด")

    mover = Mover(device)
    for label, p in stops:
        print(f"   -> {label}: ({p['x']:.2f}, {p['y']:.2f}) r={p['r']:.2f}")
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

    if device is not None and input("เดินตรวจตำแหน่งจริงด้วยแขนกลไหม? (y/n): ").strip().lower() == "y":
        walk_layout(device, settings, grid, temps)


# ==========================================
# 🚀 RUN
# ==========================================
def run_operation(device, settings):
    positions = settings["positions"]
    grid = build_grid(positions)
    temps = build_temps(positions)
    use_temp = settings.get("use_temp", True)
    if not grid or (use_temp and not temps):
        print("❌ ยังสอนตำแหน่งไม่ครบ กรุณาใช้ [2] Teach ก่อน"
              + ("" if use_temp else " (โหมดปิด temp ต้องมีอย่างน้อย grid_1 กับ grid_8)"))
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
    table_z = ground_z + block_h + grip   # ระดับผิวบนของบล็อกที่วางบนโต๊ะ
    center = grid["c"]

    # ลำดับที่ 3 และ 4 ต้องย้ายไปพักก่อนเสมอ (Tower สูง 2-3 ชั้นแล้ว เสี่ยงชนตอนเอื้อมข้าม)
    # ลำดับที่ 1-2 ไม่ต้องพัก เพราะ Tower ยังสูงไม่เกิน 1 ชั้น
    # ช่องพักไล่จาก 4 ไป 1 เพื่อไม่ให้ชนกันเอง / ปิดการพักได้ที่เมนู [7]
    staged = {}
    free_slots = [4, 3, 2, 1]
    if use_temp:
        for i, b in enumerate(order):
            if i >= 2:
                staged[b] = temps[free_slots.pop(0)]
    else:
        print("⚠️ ปิดการใช้จุดพัก (temp) อยู่ — หยิบจากช่องเดิมไปวาง Tower ตรงๆ")

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
            mover.pick_and_place(at(grid[b], table_z), at(staged[b], table_z),
                                 carry_z, empty_z, keep_rot)

    # Phase 2: สร้าง Tower ที่ช่องกลาง (c)
    for layer, b in enumerate(order):
        src = staged.get(b, grid[b])
        tgt_z = ground_z + (layer + 1) * block_h + grip
        print(f"🏗️ บล็อก {b} -> Tower ชั้น {layer + 1}")
        mover.pick_and_place(at(src, table_z), at(center, tgt_z),
                             safe_z_for(ground_z, block_h, layer),
                             safe_z_for(ground_z, block_h, layer, carrying=False), keep_rot)
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


def toggle_temp(settings):
    """เปิด/ปิดการแวะพักที่ temp ของลำดับที่ 3-4"""
    settings["use_temp"] = not settings.get("use_temp", True)
    if settings["use_temp"]:
        print("✅ เปิดการใช้จุดพัก: ลำดับที่ 3-4 จะแวะพักที่ temp ก่อน")
    else:
        print("⛔ ปิดการใช้จุดพัก: ทุกลำดับหยิบจากช่องเดิมไปวาง Tower ตรงๆ "
              "(เสี่ยงชน Tower ตอนเอื้อมข้าม)")


def reset_positions(settings):
    """ล้างพิกัดที่สอนไว้ทั้งหมด (grid_1, grid_8, temp_top, temp_last) เพื่อเริ่มสอนใหม่
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
            temp_state = "ON" if settings.get("use_temp", True) else "OFF"
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
                toggle_temp(settings)
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
