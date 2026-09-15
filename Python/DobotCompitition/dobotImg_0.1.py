import time
import cv2 as cv
import numpy as np

# ==============================================================================
# Configuration & Constants (Pre-allocated for performance)
# ==============================================================================
CAMERA_INDEX = 0
MIN_CONTOUR_AREA = 600  # Minimum pixel area to filter out noise specks
MORPH_KERNEL = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))

# HSV Color ranges: [Lower, Upper]
# Note: Red wraps around the 0/180 Hue boundary in OpenCV HSV, so it uses two ranges.
COLOR_RANGES = {
    "Green": {
        "ranges": [
            (np.array([35, 60, 60], dtype=np.uint8), np.array([85, 255, 255], dtype=np.uint8))
        ],
        "bgr": (0, 255, 0),
    },
    "Blue": {
        "ranges": [
            (np.array([95, 60, 60], dtype=np.uint8), np.array([135, 255, 255], dtype=np.uint8))
        ],
        "bgr": (255, 120, 0),
    },
    "Yellow": {
        "ranges": [
            (np.array([20, 60, 60], dtype=np.uint8), np.array([35, 255, 255], dtype=np.uint8))
        ],
        "bgr": (0, 255, 255),
    },
    "Red": {
        "ranges": [
            (np.array([0, 70, 70], dtype=np.uint8), np.array([10, 255, 255], dtype=np.uint8)),
            (np.array([170, 70, 70], dtype=np.uint8), np.array([180, 255, 255], dtype=np.uint8)),
        ],
        "bgr": (0, 0, 255),
    },
}


def process_color_mask(hsv_frame: np.ndarray, color_info: dict) -> np.ndarray:
    """Generate and clean binary mask for a specific color using morphology."""
    mask = None
    for lower, upper in color_info["ranges"]:
        sub_mask = cv.inRange(hsv_frame, lower, upper)
        mask = sub_mask if mask is None else cv.bitwise_or(mask, sub_mask)

    # Morphological Opening: Remove stray noise pixels
    mask = cv.morphologyEx(mask, cv.MORPH_OPEN, MORPH_KERNEL)
    # Morphological Closing: Fill internal holes inside objects
    mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, MORPH_KERNEL)
    return mask


def main():
    print(f"Opening camera index {CAMERA_INDEX}...")
    cap = cv.VideoCapture(CAMERA_INDEX)

    if not cap.isOpened():
        print(f"❌ Error: Cannot open camera index {CAMERA_INDEX}")
        return

    # Set preferred resolution
    cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)

    print("✅ Camera opened successfully. Press 'q' or 'ESC' to exit.")
    last_print_time = 0.0

    try:
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                print("❌ Error: Failed to receive frame from camera stream.")
                break

            # 1. Preprocessing: Smooth slight sensor noise and convert BGR -> HSV
            blurred = cv.GaussianBlur(frame, (5, 5), 0)
            hsv = cv.cvtColor(blurred, cv.COLOR_BGR2HSV)

            detected_objects = []

            # 2. Iterate through each target color
            for color_name, color_info in COLOR_RANGES.items():
                mask = process_color_mask(hsv, color_info)

                # Find external contours
                contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

                for cnt in contours:
                    area = cv.contourArea(cnt)
                    if area < MIN_CONTOUR_AREA:
                        continue

                    # Safe moments calculation
                    M = cv.moments(cnt)
                    if M["m00"] <= 0:
                        continue

                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])

                    detected_objects.append({
                        "color": color_name,
                        "center": (cX, cY),
                        "area": area,
                        "contour": cnt,
                        "bgr": color_info["bgr"],
                    })

            # 3. Draw detections on frame
            for obj in detected_objects:
                cX, cY = obj["center"]
                color_bgr = obj["bgr"]
                color_name = obj["color"]
                area = obj["area"]

                # Draw bounding box
                x, y, w, h = cv.boundingRect(obj["contour"])
                cv.rectangle(frame, (x, y), (x + w, y + h), color_bgr, 2)

                # Draw centroid marker
                cv.circle(frame, (cX, cY), 5, (255, 255, 255), -1)
                cv.drawMarker(frame, (cX, cY), color_bgr, cv.MARKER_CROSS, 14, 2)

                # Display label with coordinate info
                label = f"{color_name}: ({cX}, {cY})"
                cv.putText(
                    frame,
                    label,
                    (x, max(y - 10, 20)),
                    cv.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    color_bgr,
                    2,
                )

            # 4. On-screen HUD info
            hud_text = f"Detected: {len(detected_objects)} object(s) | Press 'q' to quit"
            cv.putText(
                frame,
                hud_text,
                (10, 25),
                cv.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
            )

            # 5. Throttled console logging (every 1 second) to prevent console spam
            current_time = time.time()
            if current_time - last_print_time >= 1.0:
                last_print_time = current_time
                if detected_objects:
                    summary = [
                        f"{obj['color']} at ({obj['center'][0]}, {obj['center'][1]}) area={int(obj['area'])}"
                        for obj in detected_objects
                    ]
                    print(f"🎯 Targets: {', '.join(summary)}")
                else:
                    print("🔍 No targets detected within area threshold.")

            # Show window
            cv.imshow("Dobot Color Tracking (dobotImg_0.1)", frame)

            # Exit key handling
            key = cv.waitKey(1) & 0xFF
            if key in (ord('q'), 27):  # 'q' or ESC
                print("Exit requested by user.")
                break

    finally:
        cap.release()
        cv.destroyAllWindows()
        print("Camera released and windows closed.")


if __name__ == "__main__":
    main()
