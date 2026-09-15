import json
import os
import sys
import time
import cv2 as cv
import numpy as np

# ==============================================================================
# Configuration & File Paths
# ==============================================================================
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_FILE = os.path.join(BASE_DIR, "color_config.json")
CALIB_FILE = os.path.join(BASE_DIR, "cam_robot_calib.json")

CAMERA_INDEX = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
DEFAULT_MIN_AREA = 600

# Default HSV Color ranges
DEFAULT_COLORS = {
    "Green": {
        "h_min": 35, "h_max": 85,
        "s_min": 60, "s_max": 255,
        "v_min": 60, "v_max": 255,
        "bgr": [0, 255, 0],
    },
    "Blue": {
        "h_min": 95, "h_max": 135,
        "s_min": 60, "s_max": 255,
        "v_min": 60, "v_max": 255,
        "bgr": [255, 120, 0],
    },
    "Yellow": {
        "h_min": 20, "h_max": 35,
        "s_min": 60, "s_max": 255,
        "v_min": 60, "v_max": 255,
        "bgr": [0, 255, 255],
    },
    "Red": {
        "h_min1": 0, "h_max1": 10,
        "h_min2": 170, "h_max2": 180,
        "s_min": 70, "s_max": 255,
        "v_min": 70, "v_max": 255,
        "bgr": [0, 0, 255],
    },
}

COLOR_NAMES = ["Green", "Blue", "Yellow", "Red"]
MORPH_KERNEL = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
TUNER_WINDOW = "HSV Color Tuner (Press 's' to save)"
MASK_WINDOW = "Active Color Mask"


# ==============================================================================
# Config & Calibration Helpers
# ==============================================================================
def load_color_config():
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, "r") as f:
                data = json.load(f)
                print(f"📁 Loaded tuned color config from: {CONFIG_FILE}")
                return data
        except Exception as e:
            print(f"⚠️ Error reading config file: {e}. Using defaults.")
    return DEFAULT_COLORS


def save_color_config(config):
    try:
        with open(CONFIG_FILE, "w") as f:
            json.dump(config, f, indent=4)
        print(f"💾 Successfully saved tuned configuration to: {CONFIG_FILE}")
    except Exception as e:
        print(f"❌ Failed to save config: {e}")


def load_calibration_matrix():
    """Load 3x3 homography matrix mapping pixels (u, v) -> robot (X, Y) mm."""
    if os.path.exists(CALIB_FILE):
        try:
            with open(CALIB_FILE, "r") as f:
                data = json.load(f)
                H = np.array(data["homography_matrix"], dtype=np.float32)
                print(f"🎯 Loaded camera-to-robot calibration from: {CALIB_FILE}")
                return H
        except Exception as e:
            print(f"⚠️ Failed to load calibration: {e}")
    return None


def pixel_to_robot(u, v, H):
    """Transform pixel coordinate (u, v) to robot coordinate (X, Y) mm."""
    if H is None:
        return None
    pt = np.array([[[float(u), float(v)]]], dtype=np.float32)
    transformed = cv.perspectiveTransform(pt, H)
    return float(transformed[0][0][0]), float(transformed[0][0][1])


# ==============================================================================
# Image Processing & Orientation Detection
# ==============================================================================
def get_color_mask(hsv, color_name, cfg):
    c = cfg[color_name]
    if color_name == "Red":
        m1 = cv.inRange(
            hsv,
            np.array([c["h_min1"], c["s_min"], c["v_min"]], dtype=np.uint8),
            np.array([c["h_max1"], c["s_max"], c["v_max"]], dtype=np.uint8),
        )
        m2 = cv.inRange(
            hsv,
            np.array([c["h_min2"], c["s_min"], c["v_min"]], dtype=np.uint8),
            np.array([c["h_max2"], c["s_max"], c["v_max"]], dtype=np.uint8),
        )
        mask = cv.bitwise_or(m1, m2)
    else:
        mask = cv.inRange(
            hsv,
            np.array([c["h_min"], c["s_min"], c["v_min"]], dtype=np.uint8),
            np.array([c["h_max"], c["s_max"], c["v_max"]], dtype=np.uint8),
        )

    # Morphological cleaning
    mask = cv.morphologyEx(mask, cv.MORPH_OPEN, MORPH_KERNEL)
    mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, MORPH_KERNEL)
    return mask


def calculate_orientation(contour):
    """Calculate centroid, rotated bounding box, and orientation angle (theta) in degrees."""
    rect = cv.minAreaRect(contour)
    (cx, cy), (width, height), angle = rect

    # Normalize angle so theta aligns along the longer dimension of the object
    if width < height:
        theta = angle - 90.0
    else:
        theta = angle

    # Keep theta bounded in [-90, 90] degrees for Dobot rotation (r axis)
    while theta <= -90.0:
        theta += 180.0
    while theta > 90.0:
        theta -= 180.0

    box = cv.boxPoints(rect)
    # Compatible with NumPy 2.0+ (np.int0 is deprecated/removed in NumPy 2.0)
    box = np.int32(box)
    return int(cx), int(cy), theta, box


# ==============================================================================
# Interactive HSV Tuner Window (Smooth Trackbars without flickering)
# ==============================================================================
def nothing(x):
    pass


def setup_tuner_window(color_config, active_color_idx):
    cv.namedWindow(TUNER_WINDOW, cv.WINDOW_NORMAL)
    cv.resizeWindow(TUNER_WINDOW, 480, 420)

    cv.createTrackbar("Color (0:G, 1:B, 2:Y, 3:R)", TUNER_WINDOW, active_color_idx, len(COLOR_NAMES) - 1, nothing)
    cv.createTrackbar("H Min / H1 Max", TUNER_WINDOW, 0, 180, nothing)
    cv.createTrackbar("H Max / H2 Min", TUNER_WINDOW, 180, 180, nothing)
    cv.createTrackbar("S Min", TUNER_WINDOW, 0, 255, nothing)
    cv.createTrackbar("S Max", TUNER_WINDOW, 255, 255, nothing)
    cv.createTrackbar("V Min", TUNER_WINDOW, 0, 255, nothing)
    cv.createTrackbar("V Max", TUNER_WINDOW, 255, 255, nothing)
    cv.createTrackbar("Min Area", TUNER_WINDOW, DEFAULT_MIN_AREA, 5000, nothing)

    sync_trackbars_to_color(color_config, active_color_idx)


def sync_trackbars_to_color(color_config, active_color_idx):
    cname = COLOR_NAMES[active_color_idx]
    c = color_config[cname]

    if cname == "Red":
        cv.setTrackbarPos("H Min / H1 Max", TUNER_WINDOW, c["h_max1"])
        cv.setTrackbarPos("H Max / H2 Min", TUNER_WINDOW, c["h_min2"])
    else:
        cv.setTrackbarPos("H Min / H1 Max", TUNER_WINDOW, c["h_min"])
        cv.setTrackbarPos("H Max / H2 Min", TUNER_WINDOW, c["h_max"])

    cv.setTrackbarPos("S Min", TUNER_WINDOW, c["s_min"])
    cv.setTrackbarPos("S Max", TUNER_WINDOW, c["s_max"])
    cv.setTrackbarPos("V Min", TUNER_WINDOW, c["v_min"])
    cv.setTrackbarPos("V Max", TUNER_WINDOW, c["v_max"])


def update_config_from_trackbars(color_config, active_color_idx):
    cname = COLOR_NAMES[active_color_idx]
    c = color_config[cname]

    h1 = cv.getTrackbarPos("H Min / H1 Max", TUNER_WINDOW)
    h2 = cv.getTrackbarPos("H Max / H2 Min", TUNER_WINDOW)
    if cname == "Red":
        c["h_max1"] = h1
        c["h_min2"] = h2
    else:
        c["h_min"] = h1
        c["h_max"] = h2

    c["s_min"] = cv.getTrackbarPos("S Min", TUNER_WINDOW)
    c["s_max"] = cv.getTrackbarPos("S Max", TUNER_WINDOW)
    c["v_min"] = cv.getTrackbarPos("V Min", TUNER_WINDOW)
    c["v_max"] = cv.getTrackbarPos("V Max", TUNER_WINDOW)
    min_area = cv.getTrackbarPos("Min Area", TUNER_WINDOW)
    return max(min_area, 50)


# ==============================================================================
# Main Execution Loop
# ==============================================================================
def main():
    color_config = load_color_config()
    H_matrix = load_calibration_matrix()

    print(f"Opening camera index {CAMERA_INDEX}...")
    cap = cv.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print(f"❌ Error: Cannot open camera index {CAMERA_INDEX}")
        return

    cap.set(cv.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    tuner_mode = "--tune" in sys.argv
    active_color_idx = 0
    if tuner_mode:
        setup_tuner_window(color_config, active_color_idx)

    min_contour_area = DEFAULT_MIN_AREA
    last_log_time = 0.0

    print("\n" + "=" * 65)
    print("🤖 dobotImg_0.2 - Dobot Vision & Color Tracking System")
    print("Controls:")
    print("  't' : Toggle Live HSV Tuner Window")
    print("  's' : Save current tuned HSV thresholds to JSON")
    print("  'q' or ESC : Exit")
    print("=" * 65 + "\n")

    try:
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                print("❌ Stream ended or frame lost.")
                break

            # Handle tuner window updates
            if tuner_mode:
                new_col_idx = cv.getTrackbarPos("Color (0:G, 1:B, 2:Y, 3:R)", TUNER_WINDOW)
                if 0 <= new_col_idx < len(COLOR_NAMES) and new_col_idx != active_color_idx:
                    active_color_idx = new_col_idx
                    sync_trackbars_to_color(color_config, active_color_idx)
                min_contour_area = update_config_from_trackbars(color_config, active_color_idx)

            blurred = cv.GaussianBlur(frame, (5, 5), 0)
            hsv = cv.cvtColor(blurred, cv.COLOR_BGR2HSV)

            detected_objects = []

            # Detect across all colors
            for color_name in COLOR_NAMES:
                mask = get_color_mask(hsv, color_name, color_config)
                contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

                for cnt in contours:
                    area = cv.contourArea(cnt)
                    if area < min_contour_area:
                        continue

                    cx, cy, theta, box = calculate_orientation(cnt)
                    robot_coords = pixel_to_robot(cx, cy, H_matrix)

                    bgr_tuple = tuple(int(x) for x in color_config[color_name]["bgr"])
                    detected_objects.append({
                        "color": color_name,
                        "pixel": (cx, cy),
                        "angle": theta,
                        "box": box,
                        "area": area,
                        "bgr": bgr_tuple,
                        "robot_coords": robot_coords,
                    })

                # Display active mask in tuner mode
                if tuner_mode and color_name == COLOR_NAMES[active_color_idx]:
                    cv.imshow(MASK_WINDOW, mask)

            # Draw Detections
            for obj in detected_objects:
                cx, cy = obj["pixel"]
                theta = obj["angle"]
                box = obj["box"]
                bgr = obj["bgr"]
                color_name = obj["color"]

                # 1. Rotated bounding box
                cv.drawContours(frame, [box], 0, bgr, 2)

                # 2. Centroid point
                cv.circle(frame, (cx, cy), 4, (255, 255, 255), -1)

                # 3. Orientation arrow (indicates robot gripper rotation r)
                arrow_len = 35
                rad = np.deg2rad(theta)
                end_x = int(cx + arrow_len * np.cos(rad))
                end_y = int(cy + arrow_len * np.sin(rad))
                cv.arrowedLine(frame, (cx, cy), (end_x, end_y), (0, 255, 255), 2, tipLength=0.3)

                # 4. Text labels
                label_txt = f"{color_name}: ({cx},{cy}) {theta:.1f}deg"
                if obj["robot_coords"] is not None:
                    rx, ry = obj["robot_coords"]
                    robot_txt = f"Robot: X={rx:.1f}, Y={ry:.1f}mm"
                    cv.putText(frame, robot_txt, (cx - 40, cy + 26), cv.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

                cv.putText(frame, label_txt, (cx - 40, cy - 10), cv.FONT_HERSHEY_SIMPLEX, 0.48, bgr, 2)

            # HUD Display
            active_color_label = COLOR_NAMES[active_color_idx] if tuner_mode else "OFF"
            hud_status = f"Objects: {len(detected_objects)} | Tuner: {active_color_label} [T] | Calib: {'YES' if H_matrix is not None else 'NO'}"
            cv.putText(frame, hud_status, (12, 25), cv.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

            cv.imshow("Dobot Vision System v0.2", frame)

            # Periodic logging (every 1.5 seconds)
            now = time.time()
            if now - last_log_time >= 1.5:
                last_log_time = now
                if detected_objects:
                    items = [
                        f"{o['color']} px=({o['pixel'][0]},{o['pixel'][1]}), r={o['angle']:.1f}°"
                        + (f" -> Robot=({o['robot_coords'][0]:.1f}, {o['robot_coords'][1]:.1f}mm)" if o['robot_coords'] else "")
                        for o in detected_objects
                    ]
                    print(f"🎯 Objects: {' | '.join(items)}")

            # Key Controls
            key = cv.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                print("Exiting...")
                break
            elif key == ord('t'):
                tuner_mode = not tuner_mode
                if tuner_mode:
                    setup_tuner_window(color_config, active_color_idx)
                else:
                    try:
                        cv.destroyWindow(TUNER_WINDOW)
                        cv.destroyWindow(MASK_WINDOW)
                    except cv.error:
                        pass
                print(f"Tuner mode: {'ENABLED' if tuner_mode else 'DISABLED'}")
            elif key == ord('s'):
                save_color_config(color_config)

    finally:
        cap.release()
        cv.destroyAllWindows()
        print("Cleaned up camera and closed windows.")


if __name__ == "__main__":
    main()
