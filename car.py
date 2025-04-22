import asyncio
import websockets
import json
import multiprocessing
import queue
import threading
import time
import keyboard
import os

class CarController:
    def __init__(self, command_queue=None):
        self.websocket = None
        self.connected = False
        self.current_ip = None
        self.ip_range = ["192.168.4.2", "192.168.4.3", "192.168.4.4", "192.168.4.5"]
        self.current_left = 0
        self.current_right = 0
        self.current_pan = 90
        self.current_tilt = 90
        self.command_queue = command_queue
        self.running = True
        self.speed = 150
        self.key_state = {k: False for k in 'wasdijkl'}

    async def ensure_connection(self):
        while self.running:
            for ip in self.ip_range:
                try:
                    self.current_ip = ip
                    uri = f"ws://{ip}:81"
                    print(f"Connecting to {uri}...")
                    self.websocket = await websockets.connect(uri)
                    self.connected = True
                    print(f"✅ Connected to car at {ip}")
                    return
                except Exception as e:
                    print(f"❌ Connection to {ip} failed: {e}")
                    self.connected = False
                    await asyncio.sleep(1)
            await asyncio.sleep(5)

    async def process_commands(self):
        while self.running:
            try:
                if self.websocket and self.connected:
                    # Process keyboard commands
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

                    # Calculate movement based on keyboard controls
                    left = right = 0
                    if self.key_state['d']:
                        left = right = self.speed  # Forward
                    elif self.key_state['a']:
                        left = right = -self.speed  # Backward
                    if self.key_state['s']:
                        left = -self.speed  # Left turn
                        right = self.speed
                    if self.key_state['w']:
                        left = self.speed  # Right turn
                        right = -self.speed

                    # Camera pan
                    if self.key_state['j']:
                        self.current_pan = min(180, self.current_pan + 10)
                    if self.key_state['l']:
                        self.current_pan = max(0, self.current_pan - 10)

                    # Camera tilt
                    if self.key_state['i']:
                        self.current_tilt = max(0, self.current_tilt - 10)
                    if self.key_state['k']:
                        self.current_tilt = min(180, self.current_tilt + 10)

                    # Read commands from file
                    try:
                        if os.path.exists('car_command.txt'):
                            with open('car_command.txt', 'r') as f:
                                command = f.read().strip()
                            os.remove('car_command.txt')  # Remove the file after reading
                            
                            if command.startswith("CONTROL "):
                                cmd = command[8:]  # Remove "CONTROL " prefix
                                if cmd == "FORWARD":
                                    left = right = self.speed
                                elif cmd == "BACKWARD":
                                    left = right = -self.speed
                                elif cmd == "LEFT":
                                    left = -self.speed
                                    right = self.speed
                                elif cmd == "RIGHT":
                                    left = self.speed
                                    right = -self.speed
                                elif cmd.startswith("SPEED "):
                                    try:
                                        speed = int(cmd[6:])
                                        self.speed = max(0, min(255, speed))
                                        print(f"Speed set to {self.speed}")
                                    except ValueError:
                                        print("Invalid speed value")
                                elif cmd == "PAN LEFT":
                                    self.current_pan = max(0, self.current_pan - 10)
                                elif cmd == "PAN RIGHT":
                                    self.current_pan = min(180, self.current_pan + 10)
                                elif cmd == "TILT UP":
                                    self.current_tilt = max(0, self.current_tilt - 10)
                                elif cmd == "TILT DOWN":
                                    self.current_tilt = min(180, self.current_tilt + 10)
                                elif cmd == "STOP":
                                    left = right = 0
                    except Exception as e:
                        print(f"Error reading command file: {e}")

                    # Send movement command
                    await self.websocket.send(json.dumps({
                        "left": left,
                        "right": right,
                        "pan": self.current_pan,
                        "tilt": self.current_tilt
                    }))
            except Exception as e:
                print(f"Send error: {e}")
                self.connected = False
                await self.ensure_connection()
            await asyncio.sleep(0.1)

    def stop(self):
        self.running = False
        if self.websocket:
            asyncio.run(self.websocket.close())

async def main():
    # Create a queue for communication between processes
    command_queue = multiprocessing.Queue(maxsize=100)
    
    # Initialize car controller
    car = CarController(command_queue)
    
    # Connect to car
    await car.ensure_connection()
    
    # Start command processing in a separate task
    command_task = asyncio.create_task(car.process_commands())
    
    try:
        # Keep the main task running
        while car.running:
            await asyncio.sleep(1)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        car.running = False
        await command_task
        car.stop()

if __name__ == "__main__":
    asyncio.run(main())
