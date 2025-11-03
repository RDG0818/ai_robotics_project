import cv2
from ultralytics import YOLO

# Load YOLOv8 model (you can change to yolov8s.pt for better accuracy)
model = YOLO("yolov8n.pt")

# Open your default webcam (0 = default camera)
cap = cv2.VideoCapture("9048814-uhd_4096_2160_25fps.mp4")
if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to grab frame.")
        break

    # Run YOLO detection
    results = model(frame, verbose=False)

    # Draw results on the frame
    annotated_frame = results[0].plot()

    # Show the output
    cv2.imshow("YOLOv8 Live Detection", annotated_frame)

    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Clean up
cap.release()
cv2.destroyAllWindows()

