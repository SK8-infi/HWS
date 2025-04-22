import cv2
import time
from ultralytics import YOLO
import torch

# Constants
VIDEO_STREAM_URL = "http://192.168.4.1:81/stream"
DISPLAY_WIDTH = 640
DISPLAY_HEIGHT = 480

# Check for CUDA availability
device = 'cuda' if torch.cuda.is_available() else 'cpu'
print(f"Using device: {device}")

# Load YOLOv8 model and move to GPU if available
model = YOLO("yolov8l.pt")  # Using large model for better accuracy
if device == 'cuda':
    model.to(device)
    print("Model moved to GPU")

def run_camera_detector():
    cap = cv2.VideoCapture(VIDEO_STREAM_URL)
    if not cap.isOpened():
        print(f"❌ Failed to open video stream: {VIDEO_STREAM_URL}")
        return

    # Set buffer size to 1 to minimize latency
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    
    print(f"📹 Streaming from {VIDEO_STREAM_URL}")
    win_name = "ESP32-CAM Live Stream"
    cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)

    fullscreen = False
    prev_time = time.time()
    frame_count = 0
    last_frame_time = time.time()

    while True:
        # Clear buffer by reading frames until we get the latest one
        for _ in range(2):  # Skip 2 frames to ensure we get the latest
            cap.grab()
        
        ret, frame = cap.read()
        if not ret:
            print("Stream disconnected.")
            break

        # Calculate actual FPS
        current_time = time.time()
        frame_count += 1
        if current_time - last_frame_time >= 1.0:  # Update FPS every second
            actual_fps = frame_count / (current_time - last_frame_time)
            frame_count = 0
            last_frame_time = current_time
            print(f"Actual FPS: {actual_fps:.1f}")

        # Resize to fixed size
        frame = cv2.resize(frame, (DISPLAY_WIDTH, DISPLAY_HEIGHT))

        # Run YOLOv8 object detection with GPU if available
        results = model.predict(source=frame, imgsz=640, conf=0.4, verbose=False, device=device)
        annotated_frame = results[0].plot()

        # Calculate processing FPS
        curr_time = time.time()
        fps = 1 / (curr_time - prev_time)
        prev_time = curr_time

        # Overlay FPS and latency info
        cv2.putText(annotated_frame, f"FPS: {fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        cv2.putText(annotated_frame, f"Latency: {(time.time() - current_time)*1000:.1f}ms", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

        cv2.imshow(win_name, annotated_frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('f'):
            fullscreen = not fullscreen
            flag = cv2.WINDOW_FULLSCREEN if fullscreen else cv2.WINDOW_NORMAL
            cv2.setWindowProperty(win_name, cv2.WND_PROP_FULLSCREEN, flag)
        elif key == 27:  # ESC to exit fullscreen
            fullscreen = False
            cv2.setWindowProperty(win_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    run_camera_detector() 