import asyncio
import websockets
import json
import keyboard
import time
import platform
import subprocess
import cv2
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
model = YOLO("yolov8l.pt")  # Using nano model for better performance
if device == 'cuda':
    model.to(device)
    print("Model moved to GPU")

class CarController:
    def __init__(self):
        self.speed = 150
        self.current_left = 0
        self.current_right = 0
        self.pan_angle = 90
        self.tilt_angle = 90
        self.last_pan_time = 0
        self.last_tilt_time = 0
        self.pan_delay = 0.1
        self.tilt_delay = 0.1
        self.websocket = None
        self.connected = False
        self.current_ip = None
        self.ip_range = ["192.168.4.2", "192.168.4.3", "192.168.4.4", "192.168.4.5"]
        self.key_state = {k: False for k in 'wasdijkl'}

    async def ensure_connection(self):
        while True:
            for ip in self.ip_range:
                try:
                    self.current_ip = ip
                    uri = f"ws://{ip}:81"
                    print(f"Connecting to {uri}...")
                    self.websocket = await websockets.connect(uri)
                    self.connected = True
                    print(f"✅ Connected to car at {ip}")
                    asyncio.create_task(self.receive_data())
                    return
                except Exception as e:
                    print(f"❌ Connection to {ip} failed: {e}")
                    self.connected = False
                    await asyncio.sleep(1)
            await asyncio.sleep(5)

    async def receive_data(self):
        while True:
            try:
                message = await self.websocket.recv()
                data = json.loads(message)
                self.pan_angle = data.get("pan", self.pan_angle)
                self.tilt_angle = data.get("tilt", self.tilt_angle)
            except Exception as e:
                print(f"Receive error: {e}")
                self.connected = False
                await self.ensure_connection()
                break

    async def send_speeds(self):
        while True:
            try:
                if self.websocket and self.connected:
                    await self.websocket.send(json.dumps({
                        "left": self.current_left,
                        "right": self.current_right,
                        "pan": self.pan_angle,
                        "tilt": self.tilt_angle
                    }))
                await asyncio.sleep(0.1)
            except Exception as e:
                print(f"Send error: {e}")
                self.connected = False
                await self.ensure_connection()

    async def handle_keys(self):
        while True:
            for k in self.key_state:
                self.key_state[k] = keyboard.is_pressed(k)

            if keyboard.is_pressed('+'):
                self.speed = min(self.speed + 10, 255)
                print(f"Speed increased to {self.speed}")
                await asyncio.sleep(0.2)
            if keyboard.is_pressed('-'):
                self.speed = max(self.speed - 10, 0)
                print(f"Speed decreased to {self.speed}")
                await asyncio.sleep(0.2)

            left = right = 0
            if self.key_state['w'] and not self.key_state['s']:
                left = right = self.speed
            elif self.key_state['s'] and not self.key_state['w']:
                left = right = -self.speed

            if self.key_state['a']:
                left = -self.speed
                right = self.speed
            if self.key_state['d']:
                left = self.speed
                right = -self.speed

            now = time.time()
            if self.key_state['i'] and now - self.last_tilt_time > self.tilt_delay:
                self.tilt_angle = max(0, self.tilt_angle - 10)
                self.last_tilt_time = now
            if self.key_state['k'] and now - self.last_tilt_time > self.tilt_delay:
                self.tilt_angle = min(180, self.tilt_angle + 10)
                self.last_tilt_time = now
            if self.key_state['j'] and now - self.last_pan_time > self.pan_delay:
                self.pan_angle = max(0, self.pan_angle - 10)
                self.last_pan_time = now
            if self.key_state['l'] and now - self.last_pan_time > self.pan_delay:
                self.pan_angle = min(180, self.pan_angle + 10)
                self.last_pan_time = now

            self.current_left = left
            self.current_right = right

            await asyncio.sleep(0.05)

    def stop_car(self):
        self.current_left = 0
        self.current_right = 0
        print("🛑 Car stopped.")

async def main():
    car = CarController()
    await car.ensure_connection()

    asyncio.create_task(car.send_speeds())
    asyncio.create_task(car.handle_keys())

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

        # Overlay info
        overlay_color = (255, 255, 255)
        cv2.putText(annotated_frame, f"Pan: {car.pan_angle}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, overlay_color, 2)
        cv2.putText(annotated_frame, f"Tilt: {car.tilt_angle}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, overlay_color, 2)
        cv2.putText(annotated_frame, f"Speed: {car.speed}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, overlay_color, 2)

        status_text = "Connected" if car.connected else "Disconnected"
        status_color = (0, 255, 0) if car.connected else (0, 0, 255)
        cv2.putText(annotated_frame, f"Status: {status_text}", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)

        cv2.putText(annotated_frame, f"FPS: {fps:.1f}", (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        cv2.putText(annotated_frame, f"Latency: {(time.time() - current_time)*1000:.1f}ms", (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

        cv2.imshow(win_name, annotated_frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            car.stop_car()
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
    asyncio.run(main())
