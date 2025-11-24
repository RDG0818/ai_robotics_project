import cv2
import sys
from pathlib import Path

DEVICE_INDEX = 0  # try 0 first; change to 1, 2, ... if needed
OUTPUT_PATH = Path("camera_test_frame.jpg")

def main():
    print(f"Trying to open camera at index {DEVICE_INDEX}...")
    cap = cv2.VideoCapture(DEVICE_INDEX)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

    if not cap.isOpened():
        print(f"[FAIL] Could not open camera at index {DEVICE_INDEX}")
        sys.exit(1)

    while True:
    # Try to read one frame
        ret, frame = cap.read()
        cap.release()

        if not ret or frame is None:
            print("[FAIL] Could not read a frame from the camera.")
        #sys.exit(1)
        cv2.waitKey(50)
    # Save frame to file
        cv2.imwrite(str(OUTPUT_PATH), frame)
        print(f"[OK] Captured one frame and saved to: {OUTPUT_PATH.resolve()}")

if name == "main":
    main()
