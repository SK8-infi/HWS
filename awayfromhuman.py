import cv2
import time
from ultralytics import YOLO
import torch
import asyncio
import json
import multiprocessing
import queue
import keyboard

VIDEO_STREAM_URL = "http://192.168.4.1:81/stream"
DISPLAY_WIDTH = 640
DISPLAY_HEIGHT = 480
CAR_SPEED = 150  # Default speed for car movement
PAN_SPEED = 2    # Speed of camera panning
MIN_PAN = 0      # Minimum pan angle
MAX_PAN = 180    # Maximum pan angle
PAN_CENTER = 90  # Center pan position

# Check for CUDA availability
device = 'cuda' if torch.cuda.is_available() else 'cpu'
print(f"Using device: {device}")

# Load YOLOv8 model and move to GPU if available
model = YOLO("yolov8l.pt")  # Using large model for better accuracy
if device == 'cuda':
    model.to(device)
    print("Model moved to GPU")

def analyze_position(x_center, width, img_width):
    third = img_width // 3
    if x_center < third:
        return "close left"
    elif x_center > 2 * third:
        return "close right"
    else:
        return "too close"

class CarController:
    def __init__(self):
        self.current_pan = PAN_CENTER
        self.current_tilt = 90
        self.active_keys = set()  # Keep track of currently pressed keys

    def release_all_keys(self):
        for key in self.active_keys:
            keyboard.release(key)
        self.active_keys.clear()

    def send_command(self, left, right, pan=None, tilt=None):
        # Release all previously pressed keys
        self.release_all_keys()

        if pan is not None:
            self.current_pan = max(MIN_PAN, min(MAX_PAN, pan))
            # Pan control
            if pan > self.current_pan:
                keyboard.press('j')  # Pan left
                self.active_keys.add('j')
            elif pan < self.current_pan:
                keyboard.press('l')  # Pan right
                self.active_keys.add('l')

        if tilt is not None:
            self.current_tilt = max(0, min(180, tilt))
            # Tilt control
            if tilt > self.current_tilt:
                keyboard.press('i')  # Tilt up
                self.active_keys.add('i')
            elif tilt < self.current_tilt:
                keyboard.press('k')  # Tilt down
                self.active_keys.add('k')

        # Movement control
        if left > 0 and right > 0:  # Forward
            keyboard.press('d')
            self.active_keys.add('d')
        elif left < 0 and right < 0:  # Backward
            keyboard.press('a')
            self.active_keys.add('a')
        elif left < 0 and right > 0:  # Left turn
            keyboard.press('s')
            self.active_keys.add('s')
        elif left > 0 and right < 0:  # Right turn
            keyboard.press('w')
            self.active_keys.add('w')

    def get_pan_position(self):
        return self.current_pan

    def cleanup(self):
        self.release_all_keys()

async def run_camera_detector():
    car = CarController()

    cap = cv2.VideoCapture(VIDEO_STREAM_URL)
    if not cap.isOpened():
        print(f"❌ Failed to open video stream: {VIDEO_STREAM_URL}")
        return

    # Set buffer size to 1 to minimize latency
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    
    print(f"📹 Streaming from {VIDEO_STREAM_URL}")
    win_name = "Human Position Detection"
    cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)

    fullscreen = False
    prev_time = time.time()
    frame_count = 0
    last_frame_time = time.time()
    last_pan_update = time.time()

    try:
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

            # Draw thirds lines
            third = DISPLAY_WIDTH // 3
            cv2.line(frame, (third, 0), (third, DISPLAY_HEIGHT), (0, 255, 0), 1)
            cv2.line(frame, (2*third, 0), (2*third, DISPLAY_HEIGHT), (0, 255, 0), 1)

            # Run YOLOv8 object detection with person class only
            results = model.predict(
                source=frame,
                classes=[0],  # Only detect persons (class 0)
                conf=0.5,     # Confidence threshold
                imgsz=640,
                verbose=False,
                device=device
            )
            
            # Get detection results
            boxes = results[0].boxes
            human_detected = False
            closest_person = None
            max_area = 0

            for box in boxes:
                # Get bounding box coordinates
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                width = x2 - x1
                height = y2 - y1
                center_x = x1 + width // 2
                
                # Calculate relative area (percentage of frame)
                frame_area = DISPLAY_WIDTH * DISPLAY_HEIGHT
                box_area = width * height
                relative_area = (box_area / frame_area) * 100
                
                # Track the closest person (largest bounding box)
                if relative_area > max_area:
                    max_area = relative_area
                    closest_person = (center_x, width, relative_area)
                
                # Check if person is close based on relative area (e.g., takes up more than 20% of frame)
                if relative_area > 20:  # Adjust this percentage based on your needs
                    human_detected = True
                    status = analyze_position(center_x, width, DISPLAY_WIDTH)
                    
                    # Draw bounding box and status
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    cv2.putText(frame, f"Human: {status}", (x1, y1 - 10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    cv2.putText(frame, f"Area: {relative_area:.1f}%", (x1, y1 - 40), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # Update camera pan position to follow human
            if closest_person and time.time() - last_pan_update > 0.1:  # Update every 100ms
                center_x, _, _ = closest_person
                current_pan = car.get_pan_position()
                
                # Calculate target pan position based on human position
                # Map center_x (0 to DISPLAY_WIDTH) to pan angle (MIN_PAN to MAX_PAN)
                target_pan = MIN_PAN + (center_x / DISPLAY_WIDTH) * (MAX_PAN - MIN_PAN)
                
                # Smooth panning by moving gradually towards target
                if abs(target_pan - current_pan) > PAN_SPEED:
                    if target_pan > current_pan:
                        new_pan = current_pan + PAN_SPEED
                    else:
                        new_pan = current_pan - PAN_SPEED
                    car.send_command(0, 0, pan=new_pan)
                last_pan_update = time.time()

            if not human_detected:
                cv2.putText(frame, "No human detected", (20, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                # No human detected, stop the car
                car.send_command(0, 0)
            else:
                # Control car based on human position
                if closest_person:
                    center_x, width, area = closest_person
                    third = DISPLAY_WIDTH // 3
                    current_pan = car.get_pan_position()
                    
                    # Calculate relative position considering camera pan
                    pan_offset = (current_pan - PAN_CENTER) / (MAX_PAN - MIN_PAN) * DISPLAY_WIDTH
                    adjusted_center = center_x + pan_offset
                    
                    if area > 20:  # If person is too close
                        # Move backward to maintain distance
                        car.send_command(-CAR_SPEED, -CAR_SPEED)
                    elif area < 10:  # If person is too far
                        # Move forward to get closer
                        car.send_command(CAR_SPEED, CAR_SPEED)
                    else:
                        # Person is at good distance, adjust position to center
                        if adjusted_center < DISPLAY_WIDTH//2 - 50:  # Person is on the left
                            car.send_command(-CAR_SPEED//2, CAR_SPEED//2)  # Turn left
                        elif adjusted_center > DISPLAY_WIDTH//2 + 50:  # Person is on the right
                            car.send_command(CAR_SPEED//2, -CAR_SPEED//2)  # Turn right
                        else:
                            # Person is centered, stop
                            car.send_command(0, 0)

            # Calculate processing FPS
            curr_time = time.time()
            fps = 1 / (curr_time - prev_time)
            prev_time = curr_time

            # Overlay FPS, latency, and pan info
            cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            cv2.putText(frame, f"Latency: {(time.time() - current_time)*1000:.1f}ms", (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            cv2.putText(frame, f"Pan: {car.get_pan_position():.1f}°", (10, 90), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

            cv2.imshow(win_name, frame)

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

    finally:
        cap.release()
        cv2.destroyAllWindows()
        car.cleanup()  # Release all keys when done

def main():
    # Run the camera detector
    asyncio.run(run_camera_detector())

if __name__ == "__main__":
    main() 