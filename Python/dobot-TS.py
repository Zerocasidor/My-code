import time
from pydobot import Dobot


port = "COM5"

try:
    print(f"Connect Dobot at port {port}...")
    device = Dobot(port=port)

    time.sleep(1.5) #wait for error
    if hasattr(device, "clear_alarms"):
        print("Start clear alarms\n")
        device.clear_alarms()
        time.sleep(0.5)

    print("Ready!")

    # 1. บันทึก Home จากตำแหน่งเริ่มต้น
    current_pose = device.get_pose()
    HOME_X = current_pose.position.x
    HOME_Y = current_pose.position.y
    HOME_Z = current_pose.position.z
    HOME_R = current_pose.position.r

    print(f"🏠 บันทึกตำแหน่ง Home: X={HOME_X:.1f}, Y={HOME_Y:.1f}, Z={HOME_Z:.1f}, R={HOME_R:.1f}\n")

    target_positions = {
        "1": None,  # จุดหยิบ กล่อง 1
        "2": None,  # จุดหยิบ กล่อง 2
        "3": None,  # จุดหยิบ กล่อง 3
        "4": None,  # จุดวาง กล่อง 1
        "5": None,  # จุดวาง กล่อง 2
        "6": None,  # จุดวาง กล่อง 3
    }

    labels = {
        "1": "จุดหยิบ กล่อง 1",
        "2": "จุดหยิบ กล่อง 2",
        "3": "จุดหยิบ กล่อง 3",
        "4": "จุดวาง กล่อง 1",
        "5": "จุดวาง กล่อง 2",
        "6": "จุดวาง กล่อง 3",
    }

    def move_and_wait(x, y, z, r=0, delay=0.8):
        device.move_to(x, y, z, r)
        time.sleep(delay)

    def go_home():
        print("🏠 กลับจุด Home...")
        move_and_wait(HOME_X, HOME_Y, HOME_Z, HOME_R)

    def pick(x, y, z):
        print(f"📥 กำลังหยิบวัตถุที่ ({x:.1f}, {y:.1f}, {z:.1f})...")
        move_and_wait(x, y, z + 50)
        move_and_wait(x, y, z)
        device.suck(True)
        time.sleep(0.5)
        move_and_wait(x, y, z + 50)

    def place(x, y, z):
        print(f"📤 กำลังวางวัตถุที่ ({x:.1f}, {y:.1f}, {z:.1f})...")
        move_and_wait(x, y, z + 50)
        move_and_wait(x, y, z)
        time.sleep(0.3)
        device.suck(False)
        time.sleep(0.3)
        move_and_wait(x, y, z + 50)

    # ----------------------------------------------------
    # 🎓 โหมดบันทึกตำแหน่ง (Teach & Save Mode)
    # ----------------------------------------------------
    def teach_mode():
        print("=" * 65)
        print("🎮 เข้าสู่โหมดสอนตำแหน่ง (Teach & Save Mode)")
        print("วิธีใช้:")
        print("  1. เลื่อนแขนกล Dobot ไปยังตำแหน่งที่ต้องการ")
        print("  2. พิมพ์เลข 1, 2, 3 เพื่อ 'บันทึกจุดหยิบ'")
        print("  3. พิมพ์เลข 4, 5, 6 เพื่อ 'บันทึกจุดวาง'")
        print("  4. พิมพ์ 's' เมื่อบันทึกครบ เพื่อเริ่มทำงาน")
        print("=" * 65)

        while True:
            cmd = input("\n[Teach Mode] เลื่อนแขนกลแล้วพิมพ์คำสั่ง (1-6 / s): ").strip().lower()

            if cmd in ["1", "2", "3", "4", "5", "6"]:
                pose = device.get_pose()
                target_positions[cmd] = {
                    "x": pose.position.x,
                    "y": pose.position.y,
                    "z": pose.position.z,
                    "r": pose.position.r,
                }
                pos = target_positions[cmd]
                print(f"✅ บันทึก [{labels[cmd]}] สำเร็จ! -> X={pos['x']:.1f}, Y={pos['y']:.1f}, Z={pos['z']:.1f}")

            elif cmd == "s":
                missing = [k for k, v in target_positions.items() if v is None]
                if missing:
                    print(f"⚠️ ยังบันทึกพิกัดไม่ครบ! เหลือจุด: {', '.join(missing)}")
                    confirm = input("ต้องการเริ่มทำงานเฉพาะจุดที่บันทึกแล้วหรือไม่? (y/n): ").lower()
                    if confirm == 'y':
                        break
                else:
                    print("\n✅ บันทึกพิกัดครบทั้ง 6 จุดเรียบร้อย!\n")
                    break
            else:
                print("❌ คำสั่งไม่ถูกต้อง (กรุณาป้อนเลข 1-6 หรือ s)")

    # ----------------------------------------------------
    # 🔄 วนลูปทำงานหลัก (Main Application Loop)
    # ----------------------------------------------------
    # บังคับจูนพิกัดในรอบแรกก่อน
    teach_mode()

    loop_count = 1
    while True:
        print(f"\n=================== เริ่มรันการทำงานรอบที่ {loop_count} ===================")
        go_home()

        pairs = [("1", "4"), ("2", "5"), ("3", "6")]

        for idx, (p_pick, p_place) in enumerate(pairs, start=1):
            if target_positions[p_pick] is None or target_positions[p_place] is None:
                print(f"⏭️ ข้ามกล่องที่ {idx} เนื่องจากไม่มีการบันทึกพิกัด")
                continue

            pick_pos = target_positions[p_pick]
            place_pos = target_positions[p_place]

            print(f"\n--- กล่องที่ {idx} ---")
            pick(pick_pos["x"], pick_pos["y"], pick_pos["z"])
            place(place_pos["x"], place_pos["y"], place_pos["z"])
            go_home()

        print(f"\n🎉 ทำงานรอบที่ {loop_count} เสร็จสิ้นเรียบร้อย!")

        # สอบถามผู้ใช้ว่าจะทำงานต่อหรือไม่
        print("-" * 50)
        choice = input("ต้องการรันอีกรอบด้วยพิกัดเดิมหรือไม่? (y = รันซ้ำ / r = จูนใหม่ / n = ออก): ").strip().lower()
        
        if choice == 'y':
            loop_count += 1
            continue
        elif choice == 'r':
            loop_count = 1
            teach_mode()
        else:
            print("\n👋 สิ้นสุดการทำงาน ปิดโปรแกรม...")
            break

except Exception as e:
    print(f"❌ เกิดข้อผิดพลาด: {e}")

finally:
    if "device" in locals():
        print("\nกำลังปิดการเชื่อมต่อ...")
        device.close()
        print("✅ ปิดการเชื่อมต่อเรียบร้อยแล้ว")