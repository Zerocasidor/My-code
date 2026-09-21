import os
import json
import time
import struct
from pydobot import Dobot
from pydobot.message import Message

SETTINGS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "settings.json")

DEFAULT_SETTINGS = {
    "port": "/dev/ttyUSB0",
    "robot_speed": 100,
    "robot_accel": 100,
    "block_height": 25.0,
    "ground_z": None,
    "grip_offset": 0.0,
    "positions": {k: None for k in (
        "pick_1", "temp_1", "pick_2", "temp_2",
        "pick_3", "temp_3", "pick_4", "temp_4", "center")},
}

SUCK_DELAY_MS = 50     # รอหัวดูดจับบล็อก
RELEASE_DELAY_MS = 100  # รอหัวดูดปล่อยบล็อก
#300 100 50  0
#350 150 100 0

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
        print(f"⚠️ อ่าน settings.json ไม่ได้ ({e}) ใช้ค่าเริ่มต้น")
    return s


def save_settings(settings):
    try:
        with open(SETTINGS_FILE, "w", encoding="utf-8") as f:
            json.dump(settings, f, indent=4, ensure_ascii=False)
        print("💾 บันทึก settings.json แล้ว")
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


def run_operation(device, settings):
    positions = settings["positions"]
    center = positions.get("center")
    if not center:
        print("❌ ยังไม่ได้บันทึก Center กรุณาใช้ [2] Teach ก่อน")
        return
    ground_z = settings.get("ground_z")
    block_h = settings.get("block_height", 25.0)
    grip = settings.get("grip_offset", 0.0)
    if ground_z is None:
        print("⚠️ ยังไม่ได้ตั้ง Ground ([3]) จะใช้ค่า Z จากการสอนแทน")

    def table_z(pos):
        return ground_z + block_h + grip if ground_z is not None else pos["z"]

    def at(pos, z):
        return {"x": pos["x"], "y": pos["y"], "z": z, "r": pos["r"]}

    base = ground_z if ground_z is not None else center["z"]
    start = time.perf_counter()
    mover = Mover(device)

    # Phase 1: ย้ายบล็อกจากจุดหยิบไปจุดพัก
    phase1_carry = safe_z_for(base, block_h, 0)
    phase1_empty = safe_z_for(base, block_h, 0, carrying=False)
    for i in range(1, 5):
        pick, temp = positions.get(f"pick_{i}"), positions.get(f"temp_{i}")
        if pick and temp:
            print(f"📦 บล็อก {i}: pick -> temp")
            mover.pick_and_place(at(pick, table_z(pick)), at(temp, table_z(temp)),
                                 phase1_carry, phase1_empty)

    # Phase 2: สร้าง Tower
    layer = 0
    for i in range(1, 5):
        src = positions.get(f"temp_{i}") or positions.get(f"pick_{i}")
        if not src:
            continue
        if ground_z is not None:
            tgt_z = ground_z + (layer + 1) * block_h + grip
        else:
            tgt_z = center["z"] + layer * block_h
        print(f"🏗️ บล็อก {i} -> Tower ชั้น {layer + 1}")
        mover.pick_and_place(at(src, table_z(src)), at(center, tgt_z),
                             safe_z_for(base, block_h, layer),
                             safe_z_for(base, block_h, layer, carrying=False))
        layer += 1
    mover.lift(safe_z_for(base, block_h, layer, carrying=False))
    mover.finish()
    print(f"🎉 สร้าง Tower เสร็จ {layer} ชั้น | ⏱️ {time.perf_counter() - start:.2f} sec")


def teach_mode(device, settings):
    positions = settings["positions"]
    print("🎮 Teach: 1-4=จุดหยิบ | b=จุดพักของจุดหยิบล่าสุด | b1-b4=จุดพัก | 5/c=Center | [Enter]=เสร็จ")
    last_pick = None
    while True:
        cmd = input("[Teach] > ").strip().lower()
        if cmd in ("", "s"):
            break
        if cmd in ("1", "2", "3", "4"):
            key = f"pick_{cmd}"
            last_pick = cmd
        elif cmd == "b":
            if not last_pick:
                print("⚠️ บันทึกจุดหยิบ (1-4) ก่อน หรือใช้ b1-b4")
                continue
            key = f"temp_{last_pick}"
        elif cmd in ("b1", "b2", "b3", "b4"):
            key = f"temp_{cmd[1]}"
        elif cmd in ("5", "c"):
            key = "center"
        else:
            print("❌ คำสั่งไม่ถูกต้อง")
            continue
        p = device.get_pose().position
        positions[key] = {"x": round(p.x, 2), "y": round(p.y, 2),
                          "z": round(p.z, 2), "r": round(p.r, 2)}
        print(f"✅ {key}: {positions[key]}")


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
            choice = input("\n[1]Run [2]Teach&Save [3]SetGround [4]Save&Exit [Enter]Exit > ").strip()
            if choice == "1":
                run_operation(device, settings)
            elif choice == "2":
                teach_mode(device, settings)
            elif choice == "3":
                set_ground(device, settings)
            elif choice == "4":
                save_settings(settings)
                break
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
