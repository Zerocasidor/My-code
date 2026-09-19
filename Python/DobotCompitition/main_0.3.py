import os
import sys
import json
import time
from pydobot import Dobot

# ==========================================
# ⚙️ CONFIGURATION & PERSISTENCE
# ==========================================
SETTINGS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "settings.json")

DEFAULT_SETTINGS = {
    "port": "/dev/ttyUSB0",
    "robot_speed": 100,
    "robot_accel": 100,
    "block_height": 25.0,
    "ground_z": None,
    "grip_offset": 0.0,
    "positions": {
        "pick_1": None, "temp_1": None,
        "pick_2": None, "temp_2": None,
        "pick_3": None, "temp_3": None,
        "pick_4": None, "temp_4": None,
        "center": None
    }
}

def load_settings():
    """โหลดการตั้งค่าจาก settings.json หากไม่มีจะใช้ค่าเริ่มต้น"""
    if os.path.exists(SETTINGS_FILE):
        try:
            with open(SETTINGS_FILE, "r", encoding="utf-8") as f:
                data = json.load(f)
                merged = dict(DEFAULT_SETTINGS)
                merged.update(data)
                merged["positions"] = dict(DEFAULT_SETTINGS["positions"])
                if "positions" in data and isinstance(data["positions"], dict):
                    merged["positions"].update(data["positions"])
                return merged
        except Exception as e:
            print(f"⚠️ ไม่สามารถอ่าน {SETTINGS_FILE} ได้ ({e}) ใช้ค่าเริ่มต้นแทน")
    return dict(DEFAULT_SETTINGS)

def save_settings(settings):
    """บันทึกการตั้งค่าลง settings.json"""
    try:
        with open(SETTINGS_FILE, "w", encoding="utf-8") as f:
            json.dump(settings, f, indent=4, ensure_ascii=False)
        print(f"💾 บันทึกการตั้งค่าลง {os.path.basename(SETTINGS_FILE)} เรียบร้อยแล้ว!")
        return True
    except Exception as e:
        print(f"❌ เกิดข้อผิดพลาดขณะบันทึกไฟล์: {e}")
        return False

# ==========================================
# 🤖 ROBOT CONNECTION & MOTION HELPERS
# ==========================================
def connect_robot(settings):
    port = settings.get("port", "/dev/ttyUSB0")
    speed = settings.get("robot_speed", 200)
    accel = settings.get("robot_accel", 200)
    
    try:
        port_label = port if port else "อัตโนมัติ (Auto-detect)"
        print(f"กำลังเชื่อมต่อ Dobot ที่พอร์ต {port_label}...")
        device = Dobot(port=port)
    except Exception as e:
        print(f"⚠️ ไม่สามารถเชื่อมต่อ {port} ได้ ({e}) กำลังลองค้นหาพอร์ตอัตโนมัติ...")
        try:
            device = Dobot()
        except Exception as err:
            print(f"❌ เชื่อมต่อล้มเหลว: {err}")
            return None

    try:
        time.sleep(1.5)
        if hasattr(device, "clear_alarms"):
            device.clear_alarms()
            time.sleep(0.5)
        device.speed(speed, accel)
        print("✅ เชื่อมต่อและตั้งค่าความเร็วเรียบร้อย!\n")
        return device
    except Exception as e:
        print(f"❌ เกิดข้อผิดพลาดขณะตั้งค่า Dobot: {e}")
        return None

def move_and_wait(device, x, y, z, r=0, delay=0.1):
    """สั่งการเคลื่อนที่และรอจนกว่าแขนกลจะเคลื่อนที่ถึงจุดหมายจริง (ป้องกันการตัดมุมชนสิ่งกีดขวาง)"""
    cmd_id = device.move_to(x, y, z, r)
    device.wait_for_cmd(cmd_id)
    if delay > 0:
        time.sleep(delay)

def get_dynamic_safe_z(ground_z, block_height, tower_height=0):
    """
    คำนวณ Safe Z ตามความสูงของ Tower ปัจจุบัน:
    - Tower สูง 0 หรือ 1 ก้อน : 2.5 เท่าของความสูงบล็อก (ground_z + 2.5 * block_height)
    - Tower สูง 2 ก้อน       : 3.5 เท่าของความสูงบล็อก (ground_z + 3.5 * block_height)
    - Tower สูง 3 ก้อน       : 4.5 เท่าของความสูงบล็อก (ground_z + 4.5 * block_height)
    """
    if tower_height <= 1:
        multiplier = 2.5
    elif tower_height == 2:
        multiplier = 3.5
    else:
        multiplier = 4.5
    return ground_z + (multiplier * block_height)

def execute_pick_and_place(device, source, target, safe_z, desc=""):
    """
    กระบวนการหยิบและวางแบบปลอดภัย (Dynamic Clearance):
    1. ยกแขนกลขึ้นสู่ระดับ safe_z ทันที (หากยังไม่ถึง)
    2. เคลื่อนที่แนวราบระดับ safe_z ไปอยู่เหนือจุดต้นทาง
    3. ดิ่งลงแนวตรงไปยังจุดหยิบ
    4. เปิดหัวดูด (Suck)
    5. ยกแขนกลขึ้นสู่ระดับ safe_z ในแนวดิ่งตรง
    6. เคลื่อนที่แนวราบระดับ safe_z ไปอยู่เหนือจุดวาง
    7. ดิ่งลงแนวตรงไปยังจุดวาง
    8. ปล่อยหัวดูด (Release)
    9. ยกแขนกลขึ้นสู่ระดับ safe_z ในแนวดิ่งตรง
    """
    print(f"📦 {desc}")
    
    # 1. ยกขึ้นสู่ Safe Z จากตำแหน่งปัจจุบัน
    cur_pose = device.get_pose()
    if cur_pose.position.z < (safe_z - 2.0):
        move_and_wait(device, cur_pose.position.x, cur_pose.position.y, safe_z, cur_pose.position.r)
        
    # 2. เดินทางระดับ Safe Z ไปเหนือจุดหยิบ
    move_and_wait(device, source['x'], source['y'], safe_z, source['r'])
    
    # 3. ลงแนวตรงไปหยิบ
    move_and_wait(device, source['x'], source['y'], source['z'], source['r'])
    device.suck(True)
    time.sleep(0.3)
    
    # 4. ยกขึ้นตรงๆ สู่ Safe Z
    move_and_wait(device, source['x'], source['y'], safe_z, source['r'])
    
    # 5. เดินทางระดับ Safe Z ไปเหนือจุดวาง
    move_and_wait(device, target['x'], target['y'], safe_z, target['r'])
    
    # 6. ลงแนวตรงไปวาง
    move_and_wait(device, target['x'], target['y'], target['z'], target['r'])
    device.suck(False)
    time.sleep(0.35)
    
    # 7. ยกขึ้นตรงๆ สู่ Safe Z
    move_and_wait(device, target['x'], target['y'], safe_z, target['r'])

# ==========================================
# 📐 GRID & POSITION HELPERS
# ==========================================
def calculate_3x3_grid(temp_pos, center_pos, threshold=15.0):
    dx = temp_pos['x'] - center_pos['x']
    dy = temp_pos['y'] - center_pos['y']
    
    row, col = "ตรงกลาง", "ตรงกลาง"
    if dx > threshold: row = "บน"
    elif dx < -threshold: row = "ล่าง"
    
    if dy > threshold: col = "ซ้าย"
    elif dy < -threshold: col = "ขวา"
    
    if row == "ตรงกลาง" and col == "ตรงกลาง": return "จุดกึ่งกลาง"
    if row == "ตรงกลาง": return f"ด้าน{col}"
    if col == "ตรงกลาง": return f"ด้าน{row}"
    return f"{col}{row}"

def view_settings(settings):
    """แสดงค่าคอนฟิกและตำแหน่งพิกัดทั้งหมดในระบบ"""
    positions = settings["positions"]
    ground_z = settings.get("ground_z")
    block_h = settings.get("block_height", 25.0)
    grip_off = settings.get("grip_offset", 0.0)
    
    print("\n" + "=" * 65)
    print("📋 ข้อมูลการตั้งค่าปัจจุบัน (CURRENT SETTINGS & POSITIONS)")
    print("=" * 65)
    print(f" พอร์ตที่ใช้              : {settings.get('port')}")
    print(f" ความเร็ว (Speed)         : {settings.get('robot_speed')} | อัตราเร่ง: {settings.get('robot_accel')}")
    print(f" ความสูงบล็อก (Block Height): {block_h:.2f} mm")
    print(f" ระยะกดหัวดูด (Grip Offset) : {grip_off:.2f} mm")
    if ground_z is not None:
        print(f" Smart Ground (พื้นโต๊ะ)   : ✅ {ground_z:.2f} mm")
        print(f" ระดับหยิบบนโต๊ะ (Pick Z)   : {ground_z + block_h + grip_off:.2f} mm")
        print(f" Tower ชั้น 1-4 (Target Z) : 1={ground_z + block_h + grip_off:.2f} | 2={ground_z + 2*block_h + grip_off:.2f} | 3={ground_z + 3*block_h + grip_off:.2f} | 4={ground_z + 4*block_h + grip_off:.2f} mm")
        print(f" Dynamic Safe Z           : 0-1 ชั้น={ground_z + 2.5*block_h:.2f} | 2 ชั้น={ground_z + 3.5*block_h:.2f} | 3 ชั้น={ground_z + 4.5*block_h:.2f} mm")
    else:
        print(f" Smart Ground (พื้นโต๊ะ)   : ❌ ยังไม่ได้คาลิเบรต (แนะนำเข้าเมนู [2])")
    print("-" * 65)
    print(f"{'ชื่อตำแหน่ง':<12} | {'X (mm)':<9} | {'Y (mm)':<9} | {'Z (mm)':<9} | {'R (deg)':<9} | สถานะ")
    print("-" * 65)
    
    for key in ["center", "pick_1", "temp_1", "pick_2", "temp_2", "pick_3", "temp_3", "pick_4", "temp_4"]:
        pos = positions.get(key)
        if pos:
            status = "✅ บันทึกแล้ว"
            print(f"{key:<12} | {pos['x']:<9.2f} | {pos['y']:<9.2f} | {pos['z']:<9.2f} | {pos['r']:<9.2f} | {status}")
        else:
            print(f"{key:<12} | {'-':<9} | {'-':<9} | {'-':<9} | {'-':<9} | ❌ ยังไม่มีข้อมูล")
    print("=" * 65)

# ==========================================
# 📏 SMART GROUND CALIBRATION
# ==========================================
def smart_ground_calibration(device, settings):
    """วัดและบันทึกระนาบพื้นอัจฉริยะ (Smart Ground) เพื่อคำนวณความสูงทุกจุดได้อย่างแม่นยำ"""
    print("\n" + "=" * 65)
    print("📏 SMART GROUND CALIBRATION (ตั้งค่าระนาบพื้นอัจฉริยะ)")
    print("=" * 65)
    print("ระบบจะใช้ระนาบพื้น (Ground Z) ร่วมกับความสูงของบล็อก")
    print("เพื่อคำนวณระดับการหยิบและวางบน Tower แต่ละชั้นอย่างแม่นยำ:")
    print("  - จุดหยิบ / จุดพัก (บนโต๊ะ) : Ground Z + 1.0 * Block Height")
    print("  - Tower ชั้น 1             : Ground Z + 1.0 * Block Height")
    print("  - Tower ชั้น 2             : Ground Z + 2.0 * Block Height")
    print("  - Tower ชั้น 3             : Ground Z + 3.0 * Block Height")
    print("  - Tower ชั้น 4             : Ground Z + 4.0 * Block Height")
    print("  - Safe Clearance (0-1 ชั้น): Ground Z + 2.5 * Block Height")
    print("  - Safe Clearance (2 ชั้น)  : Ground Z + 3.5 * Block Height")
    print("  - Safe Clearance (3 ชั้น)  : Ground Z + 4.5 * Block Height")
    print("=" * 65)
    
    input("\n👉 เลื่อนแขนกลให้ปลายหัวดูดแตะ 'พื้นโต๊ะ' แล้วกด [Enter] เพื่อบันทึก...")
    pose = device.get_pose()
    ground_z = round(pose.position.z, 2)
    print(f"✅ บันทึกระนาบพื้น: Ground Z = {ground_z:.2f} mm")
    
    current_h = settings.get("block_height", 25.0)
    print(f"\nความสูงบล็อกปัจจุบัน: {current_h:.2f} mm")
    h_input = input("ต้องการเปลี่ยนความสูงบล็อกหรือไม่? (กด [Enter] เพื่อใช้ค่าเดิม หรือพิมพ์ตัวเลขใหม่ mm): ").strip()
    if h_input:
        try:
            current_h = round(float(h_input), 2)
        except ValueError:
            print("⚠️ ค่าที่กรอกไม่ถูกต้อง ใช้ค่าเดิม")
            
    # Preview
    print("\n" + "-" * 55)
    print("📊 สรุปพารามิเตอร์ระดับความสูง (Height Configuration Preview):")
    print(f"   - ระนาบพื้น (Ground Z)          : {ground_z:.2f} mm")
    print(f"   - ความสูงของบล็อก (Block Height) : {current_h:.2f} mm")
    print(f"   - ระดับหยิบ/วางบนโต๊ะ (Pick/Temp): {ground_z + current_h:.2f} mm")
    print(f"   - Tower ชั้น 1 (Layer 1)        : {ground_z + (1 * current_h):.2f} mm")
    print(f"   - Tower ชั้น 2 (Layer 2)        : {ground_z + (2 * current_h):.2f} mm")
    print(f"   - Tower ชั้น 3 (Layer 3)        : {ground_z + (3 * current_h):.2f} mm")
    print(f"   - Tower ชั้น 4 (Layer 4)        : {ground_z + (4 * current_h):.2f} mm")
    print(f"   - Transit Clearance (Tower 0-1) : {ground_z + (2.5 * current_h):.2f} mm (2.5x)")
    print(f"   - Transit Clearance (Tower 2)   : {ground_z + (3.5 * current_h):.2f} mm (3.5x)")
    print(f"   - Transit Clearance (Tower 3)   : {ground_z + (4.5 * current_h):.2f} mm (4.5x)")
    print("-" * 55)
    
    confirm = input("ยืนยันการตั้งค่าหรือไม่? (y/n): ").strip().lower()
    if confirm == 'y':
        settings["ground_z"] = ground_z
        settings["block_height"] = current_h
        print("✅ อัปเดต Smart Ground ในหน่วยความจำเรียบร้อย (กด [5] ในเมนูเพื่อบันทึกลงไฟล์)")
    else:
        print("ยกเลิกการตั้งค่า")

# ==========================================
# 🎮 TEACH & SAVE POSITIONS MODE
# ==========================================
def teach_mode(device, settings):
    positions = settings["positions"]
    ground_z = settings.get("ground_z")
    block_h = settings.get("block_height", 25.0)
    
    print("\n" + "=" * 65)
    print("🎮 เข้าสู่โหมดสอนตำแหน่ง (Teach & Save Positions)")
    print("  - พิมพ์ 1-4   : บันทึก 'จุดหยิบ' (Pick 1-4)")
    print("  - พิมพ์ b     : บันทึก 'จุดวางชั่วคราว' สำหรับจุดหยิบล่าสุด")
    print("  - พิมพ์ b1-b4 : บันทึก 'จุดวางชั่วคราว' ระบุเบอร์โดยตรง (เช่น b2)")
    print("  - พิมพ์ 5 หรือ c: บันทึก 'จุดกึ่งกลางสร้าง Tower' (Center)")
    print("  - พิมพ์ p     : แสดงพิกัดที่บันทึกไว้ทั้งหมด")
    print("  - พิมพ์ s     : บันทึกลงไฟล์และเสร็จสิ้น")
    print("=" * 65)
    if ground_z is not None:
        print(f"💡 Smart Ground กำลังทำงาน: ระดับความสูง Z ในการหยิบ/วาง")
        print(f"   จะถูกคำนวณอัตโนมัติจาก Ground Z ({ground_z:.2f} mm) + Block Height ({block_h:.2f} mm)")
        print(f"   ดังนั้นคุณเพียงเลื่อนแขนกลไปเล็งพิกัด X, Y และมุมหมุน R ให้ตรงบล็อก")
        print("=" * 65)

    last_saved_pick = None

    while True:
        cmd = input("\n[Teach] ป้อนคำสั่ง (1-4, b, b1-b4, 5, p, s): ").strip().lower()
        if not cmd:
            continue
            
        if cmd == "s":
            if not positions.get("center"):
                print("⚠️ ยังไม่ได้บันทึกจุดกึ่งกลาง (5)! แนะนำให้บันทึกเพื่อใช้สร้าง Tower")
                if input("ต้องการออกโดยไม่บันทึกจุดกึ่งกลางหรือไม่? (y/n): ").strip().lower() != 'y':
                    continue
                    
            print("\n📊 --- สรุปตำแหน่ง 3x3 ---")
            center = positions.get("center")
            for i in range(1, 5):
                temp = positions.get(f"temp_{i}")
                if temp and center:
                    print(f"บล็อก {i} วางชั่วคราวที่: {calculate_3x3_grid(temp, center)}")
                elif temp:
                    print(f"บล็อก {i} วางชั่วคราวที่: บันทึกแล้ว (รอเปรียบเทียบจุด Center)")
                else:
                    print(f"บล็อก {i}: ⏩ ไม่มีจุดพัก (จะดึงจากจุดหยิบโดยตรง)")
            print("---------------------------------\n")
            
            print("✅ ตำแหน่งถูกอัปเดตในหน่วยความจำแล้ว (กด [5] ในเมนูเพื่อบันทึกลงไฟล์)")
            break
            
        elif cmd == "p":
            view_settings(settings)
            continue
            
        pose = device.get_pose()
        coord = {
            "x": round(pose.position.x, 2),
            "y": round(pose.position.y, 2),
            "z": round(pose.position.z, 2),
            "r": round(pose.position.r, 2)
        }

        if cmd in ["1", "2", "3", "4"]:
            positions[f"pick_{cmd}"] = coord
            last_saved_pick = cmd
            print(f"✅ บันทึก จุดหยิบ {cmd}: (x={coord['x']}, y={coord['y']}, z={coord['z']}, r={coord['r']})")
        elif cmd == "b":
            if last_saved_pick:
                positions[f"temp_{last_saved_pick}"] = coord
                print(f"✅ บันทึก จุดวางชั่วคราว {last_saved_pick}: (x={coord['x']}, y={coord['y']}, z={coord['z']}, r={coord['r']})")
            else:
                print("⚠️ ต้องบันทึกจุดหยิบ (1-4) ก่อนกด b หรือพิมพ์ b1, b2, b3, b4 เพื่อระบุเบอร์")
        elif cmd in ["b1", "b2", "b3", "b4"]:
            idx = cmd[1]
            positions[f"temp_{idx}"] = coord
            print(f"✅ บันทึก จุดวางชั่วคราว {idx}: (x={coord['x']}, y={coord['y']}, z={coord['z']}, r={coord['r']})")
        elif cmd in ["5", "c"]:
            positions["center"] = coord
            print(f"✅ บันทึก จุดกึ่งกลาง (Center): (x={coord['x']}, y={coord['y']}, z={coord['z']}, r={coord['r']})")
        else:
            print("❌ คำสั่งไม่ถูกต้อง (ใช้ 1-4, b, b1-b4, 5, p, s)")

# ==========================================
# ⚡ SETTINGS MENU
# ==========================================
def adjust_parameters(device, settings):
    """ปรับแต่งพารามิเตอร์ ความเร็ว, อัตราเร่ง, ระยะกด"""
    print("\n" + "-" * 50)
    print("⚡ ปรับแต่งพารามิเตอร์การทำงาน")
    print("-" * 50)
    print(f"1. ความเร็วหุ่นยนต์ (Speed)       : ปัจจุบัน {settings['robot_speed']}")
    print(f"2. อัตราเร่งหุ่นยนต์ (Accel)       : ปัจจุบัน {settings['robot_accel']}")
    print(f"3. ความสูงบล็อก (Block Height)  : ปัจจุบัน {settings['block_height']} mm")
    print(f"4. ระยะกดหัวดูด (Grip Offset)   : ปัจจุบัน {settings.get('grip_offset', 0.0)} mm")
    print("-" * 50)
    
    choice = input("เลือกค่าที่ต้องการเปลี่ยน (1-4 หรือ Enter เพื่อข้าม): ").strip()
    if choice == "1":
        val = input(f"กรอกความเร็วใหม่ (50 - 500) [เดิม: {settings['robot_speed']}]: ").strip()
        if val.isdigit():
            settings['robot_speed'] = int(val)
            device.speed(settings['robot_speed'], settings['robot_accel'])
            print("✅ อัปเดตในหน่วยความจำแล้ว (กด [5] ในเมนูเพื่อบันทึกลงไฟล์)")
    elif choice == "2":
        val = input(f"กรอกอัตราเร่งใหม่ (50 - 500) [เดิม: {settings['robot_accel']}]: ").strip()
        if val.isdigit():
            settings['robot_accel'] = int(val)
            device.speed(settings['robot_speed'], settings['robot_accel'])
            print("✅ อัปเดตในหน่วยความจำแล้ว (กด [5] ในเมนูเพื่อบันทึกลงไฟล์)")
    elif choice == "3":
        val = input(f"กรอกความสูงบล็อกใหม่ mm [เดิม: {settings['block_height']}]: ").strip()
        try:
            settings['block_height'] = float(val)
            print("✅ อัปเดตในหน่วยความจำแล้ว (กด [5] ในเมนูเพื่อบันทึกลงไฟล์)")
        except ValueError:
            print("ค่าที่ระบุไม่ถูกต้อง")
    elif choice == "4":
        val = input(f"กรอกระยะกด mm (เช่น -0.5 เพื่อให้หัวดูดแนบแน่นขึ้น) [เดิม: {settings.get('grip_offset', 0.0)}]: ").strip()
        try:
            settings['grip_offset'] = float(val)
            print("✅ อัปเดตในหน่วยความจำแล้ว (กด [5] ในเมนูเพื่อบันทึกลงไฟล์)")
        except ValueError:
            print("ค่าที่ระบุไม่ถูกต้อง")

def settings_menu(device, settings):
    """เมนูจัดการการตั้งค่าและบันทึกตำแหน่ง"""
    while True:
        print("\n" + "=" * 65)
        print("⚙️  SETTINGS & TEACH MENU (เมนูตั้งค่าและบันทึกตำแหน่ง)")
        print("=" * 65)
        print("  [1] 🎮 Teach & Save Positions (สอนพิกัดจุดหยิบ, จุดพัก, จุดกึ่งกลาง)")
        print("  [2] 📏 Smart Ground Calibration (คาลิเบรตระนาบพื้น & ตั้งความสูงบล็อก)")
        print("  [3] ⚡ Adjust Speed & Parameters (ปรับความเร็ว, อัตราเร่ง, ระยะกด)")
        print("  [4] 📋 View Current Settings & Positions (ดูการตั้งค่าและพิกัดทั้งหมด)")
        print("  [5] 💾 Save & Exit (บันทึกการตั้งค่าลง settings.json แล้วกลับสู่เมนูหลัก)")
        print("  [b] 🔙 Back to Main Menu (กลับสู่เมนูหลัก)")
        print("=" * 65)
        
        sub = input("เลือกคำสั่ง (1-5, b): ").strip().lower()
        if sub == "1":
            teach_mode(device, settings)
        elif sub == "2":
            smart_ground_calibration(device, settings)
        elif sub == "3":
            adjust_parameters(device, settings)
        elif sub == "4":
            view_settings(settings)
        elif sub == "5":
            save_settings(settings)
            break
        elif sub in ["b", "q", "back", "exit"]:
            break
        else:
            print("❌ คำสั่งไม่ถูกต้อง กรุณาเลือก 1-5 หรือ b")

# ==========================================
# 🚀 OPERATION FLOW (PHASE 1 & PHASE 2)
# ==========================================
def run_operation(device, settings):
    positions = settings["positions"]
    center = positions.get("center")
    ground_z = settings.get("ground_z")
    block_height = settings.get("block_height", 25.0)
    grip_offset = settings.get("grip_offset", 0.0)
    
    if not center:
        print("❌ ไม่พบพิกัดจุดกึ่งกลาง (Center / 5)! กรุณาเข้า Teach Mode ก่อน")
        return False
        
    available = [i for i in range(1, 5) if positions.get(f"pick_{i}") or positions.get(f"temp_{i}")]
    if not available:
        print("❌ ยังไม่มีการบันทึกตำแหน่งบล็อก (จุดหยิบ 1-4 หรือ จุดพัก temp) เลย!")
        return False
        
    if ground_z is None:
        print("\n⚠️ ยังไม่ได้คาลิเบรต Smart Ground (ระนาบพื้น)!")
        ans = input("ต้องการคาลิเบรต Smart Ground ก่อนเริ่มทำงานหรือไม่? (y/n) [แนะนำ y]: ").strip().lower()
        if ans == 'y':
            smart_ground_calibration(device, settings)
            ground_z = settings.get("ground_z")
            block_height = settings.get("block_height", 25.0)
        else:
            print("⚠️ จะใช้ค่า Z ดิบจากการสอนตำแหน่ง ซึ่งอาจทำให้ความสูงคลาดเคลื่อนได้")
            
    # คำนวณระดับความสูงหยิบบนโต๊ะ
    def get_table_pick_z(pos):
        if ground_z is not None:
            return round(ground_z + block_height + grip_offset, 2)
        return pos['z']

    # Phase 1: Move to Temp positions
    print("\n" + "=" * 65)
    print("🚀 เริ่มต้นทำงาน Phase 1: ย้ายของไปจุดพัก (Safe Clearance = 2.5x บล็อก)")
    print("=" * 65)
    
    base_ref = ground_z if ground_z is not None else center['z']
    phase1_safe_z = get_dynamic_safe_z(base_ref, block_height, tower_height=0)
    print(f"🛡️ ระดับความสูงปลอดภัย Phase 1 (Safe Z): {phase1_safe_z:.2f} mm")
    
    phase1_count = 0
    for i in range(1, 5):
        pick = positions.get(f"pick_{i}")
        temp = positions.get(f"temp_{i}")
        
        if pick and temp:
            phase1_count += 1
            src = {
                'x': pick['x'], 'y': pick['y'],
                'z': get_table_pick_z(pick),
                'r': pick['r']
            }
            tgt = {
                'x': temp['x'], 'y': temp['y'],
                'z': get_table_pick_z(temp),
                'r': temp['r']
            }
            execute_pick_and_place(
                device, 
                source=src, 
                target=tgt, 
                safe_z=phase1_safe_z, 
                desc=f"ย้ายบล็อก {i} จากจุดหยิบ -> จุดพักชั่วคราว (Z={src['z']:.2f} -> {tgt['z']:.2f})"
            )
        else:
            print(f"⏭️ ข้าม Phase 1 บล็อก {i} (ไม่มีจุดพักชั่วคราวหรือไม่มีจุดหยิบ)")
            
    if phase1_count > 0:
        print(f"✅ Phase 1 เสร็จสิ้น ({phase1_count} บล็อก)")
    else:
        print("ℹ️ ข้าม Phase 1 ทั้งหมด เนื่องจากไม่มีการระบุจุดพักชั่วคราว")

    # Phase 2: Build Tower
    print("\n" + "=" * 65)
    print("🏗️ เริ่มต้นทำงาน Phase 2: สร้าง Tower (Dynamic Clearance: 2.5x / 3.5x / 4.5x)")
    print("=" * 65)
    
    current_layer = 0  # จำนวนบล็อกที่อยู่บน Tower ในขณะนั้น
    for i in range(1, 5):
        pick = positions.get(f"pick_{i}")
        temp = positions.get(f"temp_{i}")
        
        # Priority: use temp if available, otherwise pick directly
        raw_source = temp if temp else pick
        if not raw_source:
            print(f"⏭️ ข้ามบล็อก {i} (ไม่มีทั้งจุดหยิบและจุดพัก)")
            continue
            
        layer_num = current_layer + 1
        
        # พิกัดต้นทาง (หยิบจากบนโต๊ะ)
        src = {
            'x': raw_source['x'],
            'y': raw_source['y'],
            'z': get_table_pick_z(raw_source),
            'r': raw_source['r']
        }
        
        # พิกัดปลายทางบน Tower:
        # ชั้น 1 (layer_num=1): ground_z + 1.0 * block_height
        # ชั้น 2 (layer_num=2): ground_z + 2.0 * block_height
        # ชั้น 3 (layer_num=3): ground_z + 3.0 * block_height
        # ชั้น 4 (layer_num=4): ground_z + 4.0 * block_height
        if ground_z is not None:
            tgt_z = round(ground_z + (layer_num * block_height) + grip_offset, 2)
        else:
            tgt_z = round(center['z'] + (current_layer * block_height), 2)
            
        tgt = {
            'x': center['x'],
            'y': center['y'],
            'z': tgt_z,
            'r': center['r']
        }
        
        # คำนวณ Safe Z ปรับตามความสูงของ Tower:
        # tower_height=0 หรือ 1 -> 2.5 เท่า
        # tower_height=2        -> 3.5 เท่า
        # tower_height=3        -> 4.5 เท่า
        safe_z = get_dynamic_safe_z(base_ref, block_height, tower_height=current_layer)
        
        src_label = f"จุดพัก temp_{i}" if temp else f"จุดหยิบ pick_{i}"
        execute_pick_and_place(
            device,
            source=src,
            target=tgt,
            safe_z=safe_z,
            desc=f"วางบล็อก {i} ({src_label}) -> Tower ชั้น {layer_num} (Z={tgt_z:.2f} mm | Safe Z={safe_z:.2f} mm)"
        )
        current_layer += 1
        
    print(f"\n🎉 สร้าง Tower สำเร็จเรียบร้อย ทั้งหมด {current_layer} ชั้น!")
    return True

# ==========================================
# 🎯 MAIN CONTROLLER
# ==========================================
def main():
    settings = load_settings()
    device = connect_robot(settings)
    
    if not device:
        print("❌ ไม่สามารถเชื่อมต่อ Dobot ได้ ปิดโปรแกรม")
        sys.exit(1)
        
    try:
        while True:
            positions = settings["positions"]
            center_status = "✅ บันทึกแล้ว" if positions.get("center") else "❌ ยังไม่ได้บันทึก"
            picks_count = sum(1 for i in range(1, 5) if positions.get(f"pick_{i}"))
            temps_count = sum(1 for i in range(1, 5) if positions.get(f"temp_{i}"))
            ground_status = f"✅ ({settings['ground_z']:.2f} mm)" if settings.get("ground_z") is not None else "❌ ยังไม่ได้คาลิเบรต"
            
            print("\n" + "=" * 65)
            print("🤖 DOBOT MAGICIAN - MAIN CONTROLLER (v0.3)")
            print("=" * 65)
            print(f"📌 สถานะ: จุดหยิบ ({picks_count}/4) | จุดพัก ({temps_count}/4) | Center: {center_status}")
            print(f"📏 Smart Ground: {ground_status} | ความสูงบล็อก: {settings.get('block_height'):.2f} mm")
            print(f"⚡ Dynamic Safe Clearance: 2.5x / 3.5x / 4.5x ความสูงบล็อก")
            print("-" * 65)
            print("  [1] 🚀 Run Operation (เริ่มทำงานตามตำแหน่งที่บันทึกไว้)")
            print("  [2] ⚙️ Settings Menu / Save Settings (ตั้งค่า, สอนตำแหน่ง, Smart Ground)")
            print("  [3] 🚪 Exit (ออกจากโปรแกรม)")
            print("=" * 65)
            
            choice = input("เลือกเมนูหลัก (1-3): ").strip()
            
            if choice == "1":
                if not positions.get("center"):
                    print("\n⚠️ ยังไม่ได้บันทึกตำแหน่งจุดกึ่งกลาง (Center)! กรุณาเข้าเมนูตั้งค่าเพื่อสอนตำแหน่งก่อน")
                    if input("ต้องการเข้า Settings Menu ตอนนี้เลยหรือไม่? (y/n): ").strip().lower() == 'y':
                        settings_menu(device, settings)
                    continue
                
                run_operation(device, settings)
                
                print("-" * 50)
                sub_choice = input("ตัวเลือก: [y] รันซ้ำ | [Enter] กลับสู่เมนูหลัก: ").strip().lower()
                while sub_choice == 'y':
                    run_operation(device, settings)
                    sub_choice = input("ตัวเลือก: [y] รันซ้ำ | [Enter] กลับสู่เมนูหลัก: ").strip().lower()
                    
            elif choice == "2":
                settings_menu(device, settings)
            elif choice == "3":
                print("👋 กำลังปิดโปรแกรม...")
                break
            else:
                print("❌ เมนูไม่ถูกต้อง กรุณาเลือก 1, 2 หรือ 3")
                
    except KeyboardInterrupt:
        print("\n🛑 หยุดการทำงานฉุกเฉินโดยผู้ใช้ (Ctrl+C)")
    finally:
        try:
            device.suck(False)
            device.close()
            print("✅ ปิดการเชื่อมต่อ Dobot เรียบร้อยแล้ว")
        except Exception:
            pass

if __name__ == "__main__":
    main()
