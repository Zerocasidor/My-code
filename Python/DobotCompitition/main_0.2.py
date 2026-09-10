import time
from pydobot import Dobot

# ==========================================
# ⚙️ SETTINGS (ตั้งค่าการทำงาน)
# ==========================================
PORT = "COM5"
BLOCK_HEIGHT = 25.0  # ความสูงของบล็อกแต่ละก้อน (มิลลิเมตร)
ROBOT_SPEED = 200    # ความเร็ว
ROBOT_ACCEL = 200    # อัตราเร่ง
MIN_HOP = 15         # ระยะยกพ้นพื้นเล็กน้อยสำหรับ Phase 1 (เพื่อไม่ให้ลากพื้น)
SAFE_CLEARANCE = 40  # ระยะเผื่อยกข้ามหอคอยปัจจุบัน (ป้องกันชน)

positions = {
    "pick_1": None, "temp_1": None,
    "pick_2": None, "temp_2": None,
    "pick_3": None, "temp_3": None,
    "pick_4": None, "temp_4": None,
    "center": None
}

def connect_robot():
    try:
        print(f"กำลังเชื่อมต่อ Dobot ที่พอร์ต {PORT}...")
        device = Dobot(port=PORT)
        time.sleep(1.5)
        if hasattr(device, "clear_alarms"):
            device.clear_alarms()
            time.sleep(0.5)
        device.speed(ROBOT_SPEED, ROBOT_ACCEL)
        print("✅ เชื่อมต่อและตั้งค่าความเร็วเรียบร้อย!\n")
        return device
    except Exception as e:
        print(f"❌ เชื่อมต่อล้มเหลว: {e}")
        return None

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

def teach_mode(device):
    print("=" * 65)
    print("🎮 เข้าสู่โหมดสอนตำแหน่ง (Teach & Save Mode)")
    print("  - พิมพ์ 1-4 เพื่อบันทึก 'จุดหยิบ'")
    print("  - พิมพ์ b เพื่อบันทึก 'จุดวางชั่วคราว' (ถ้าไม่กด หุ่นจะข้ามจุดพัก ไปหยิบตรงๆ)")
    print("  - พิมพ์ 5 เพื่อบันทึก 'จุดกึ่งกลาง'")
    print("  - พิมพ์ s เพื่อเริ่มทำงาน")
    print("=" * 65)

    last_saved_pick = None

    while True:
        cmd = input("\n[Teach] ป้อนคำสั่ง (1-4, b, 5, s): ").strip().lower()
        pose = device.get_pose()
        current_coord = {"x": pose.position.x, "y": pose.position.y, "z": pose.position.z, "r": pose.position.r}

        if cmd in ["1", "2", "3", "4"]:
            positions[f"pick_{cmd}"] = current_coord
            last_saved_pick = cmd
            print(f"✅ บันทึก จุดหยิบ {cmd}")
        elif cmd == "b":
            if last_saved_pick:
                positions[f"temp_{last_saved_pick}"] = current_coord
                print(f"✅ บันทึก จุดวางชั่วคราว {last_saved_pick}")
            else:
                print("⚠️ ต้องบันทึกจุดหยิบ (1-4) ก่อนกด b")
        elif cmd == "5":
            positions["center"] = current_coord
            print(f"✅ บันทึก จุดกึ่งกลาง (5)")
        elif cmd == "s":
            if not positions["center"]:
                print("⚠️ ยังไม่ได้บันทึกจุดกึ่งกลาง (5)!")
                continue
            
            print("\n📊 --- สรุปตำแหน่ง 3x3 ---")
            for i in range(1, 5):
                if positions[f"temp_{i}"]:
                    print(f"บล็อก {i} วางชั่วคราวที่: {calculate_3x3_grid(positions[f'temp_{i}'], positions['center'])}")
                else:
                    print(f"บล็อก {i}: ⏩ ไม่มีจุดพัก (จะดึงจากจุดหยิบโดยตรง)")
            print("---------------------------------\n")
            
            if input("เริ่มการทำงานเลยหรือไม่? (y/n): ").lower() == 'y':
                break

def move_and_wait(device, x, y, z, r=0, delay=0.15):
    device.move_to(x, y, z, r)
    time.sleep(delay)

def run_operation(device):
    print("\n🚀 เริ่มต้นทำงาน Phase 1: ย้ายของไปจุดพัก (ไม่หลบสิ่งกีดขวาง)...")
    
    for i in range(1, 5):
        pick = positions.get(f"pick_{i}")
        temp = positions.get(f"temp_{i}")
        
        if pick and temp:
            print(f"📦 ลากบล็อก {i} ไปจุดชั่วคราว")
            # ดึงค่า r มาใช้เพื่อรักษามุมข้อมือ
            pr = pick['r']
            tr = temp['r']
            
            move_and_wait(device, pick['x'], pick['y'], pick['z'] + 10, pr)
            move_and_wait(device, pick['x'], pick['y'], pick['z'], pr)
            device.suck(True)
            time.sleep(0.3)
            move_and_wait(device, pick['x'], pick['y'], pick['z'] + 10, pr)
            
            move_and_wait(device, temp['x'], temp['y'], temp['z'] + 10, tr)
            move_and_wait(device, temp['x'], temp['y'], temp['z'], tr)
            device.suck(False)
            time.sleep(0.3)
            move_and_wait(device, temp['x'], temp['y'], temp['z'] + 10, tr)
        else:
            print(f"⏭️ ข้ามการย้ายบล็อก {i} (ไม่มีจุดพักชั่วคราว)")

    print("\n🏗️ เริ่มต้นทำงาน Phase 2: สร้าง Tower...")
    
    center = positions["center"]
    current_layer = 0  
    cr = center['r'] # รักษามุมหมุนของจุดกึ่งกลาง
    
    safe_z = center['z'] + (4 * BLOCK_HEIGHT) + 30 
    
    for i in range(1, 5):
        pick = positions.get(f"pick_{i}")
        temp = positions.get(f"temp_{i}")
        
        source = temp if temp else pick
        
        if not source:
            print(f"⏭️ ข้ามบล็อก {i} (ไม่มีทั้งจุดหยิบและจุดพัก)")
            continue
            
        sr = source['r'] # รักษามุมหมุนของจุดต้นทาง
        target_z = center['z'] + (current_layer * BLOCK_HEIGHT)
        print(f"🗼 นำบล็อก {i} วางบน Tower ชั้นที่ {current_layer + 1} (Z={target_z:.1f})")
        
        pose = device.get_pose()
        
        # ใส่ค่า r ตาม source ในจังหวะหยิบ และ cr ในจังหวะวาง
        move_and_wait(device, pose.position.x, pose.position.y, safe_z, pose.position.r)
        move_and_wait(device, source['x'], source['y'], safe_z, sr)
        move_and_wait(device, source['x'], source['y'], source['z'], sr)
        device.suck(True)
        time.sleep(0.3)
        move_and_wait(device, source['x'], source['y'], safe_z, sr)
        
        move_and_wait(device, center['x'], center['y'], safe_z, cr)
        move_and_wait(device, center['x'], center['y'], target_z, cr)
        device.suck(False)
        time.sleep(0.3)
        move_and_wait(device, center['x'], center['y'], safe_z, cr)
        
        current_layer += 1

# ==========================================
# 🚀 MAIN LOOP (ควบคุมลูป)
# ==========================================
if __name__ == "__main__":
    device = connect_robot()
    if device:
        try:
            # เข้าสู่โหมดจูนพิกัดครั้งแรก
            teach_mode(device)
            
            while True:
                # เริ่มทำงาน
                run_operation(device)
                
                print("\n🎉 การทำงานรอบนี้เสร็จสิ้น!")
                print("-" * 50)
                choice = input("เมนู: [y] รันซ้ำพิกัดเดิม | [r] จูนพิกัดใหม่ | [n] ออกจากโปรแกรม : ").strip().lower()
                
                if choice == 'n':
                    print("👋 สิ้นสุดการทำงาน ปิดโปรแกรม...")
                    break
                elif choice == 'r':
                    # ถ้าเลือก r ให้จูนใหม่ แล้วลูป while จะพาไป run_operation อีกรอบ
                    positions = {k: None for k in positions} # Reset ค่าพิกัดเดิม
                    teach_mode(device)
                elif choice == 'y':
                    # ถ้าเลือก y ลูป while จะวนขึ้นไป run_operation อัตโนมัติ โดยใช้พิกัดเดิม
                    continue
                else:
                    print("❌ คำสั่งไม่ถูกต้อง จะรันซ้ำด้วยพิกัดเดิม...")

        except KeyboardInterrupt:
            print("\n🛑 หยุดการทำงานฉุกเฉินโดยผู้ใช้ (Ctrl+C)")
        finally:
            device.suck(False)
            device.close()
            print("✅ ปิดการเชื่อมต่อเรียบร้อยแล้ว")