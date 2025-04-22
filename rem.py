import subprocess
import sys
import multiprocessing
import time
import socket
import netifaces
import threading
import psutil
import keyboard

def get_local_ip():
    try:
        # Get all network interfaces
        interfaces = netifaces.interfaces()
        for interface in interfaces:
            # Skip loopback interface
            if interface == 'lo':
                continue
            # Get addresses for the interface
            addrs = netifaces.ifaddresses(interface)
            # Get IPv4 addresses
            if netifaces.AF_INET in addrs:
                for addr in addrs[netifaces.AF_INET]:
                    ip = addr['addr']
                    # Skip localhost
                    if not ip.startswith('127.'):
                        return ip
    except:
        pass
    return "127.0.0.1"  # Fallback to localhost

def terminate_program(program_name):
    try:
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                # Check if this is a Python process running the target program
                if proc.info['cmdline'] and program_name in ' '.join(proc.info['cmdline']):
                    print(f"Terminating {program_name} (PID: {proc.info['pid']})")
                    proc.terminate()
                    proc.wait(timeout=3)  # Wait for process to terminate
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.TimeoutExpired):
                continue
    except Exception as e:
        print(f"Error terminating {program_name}: {e}")

def run_program(program):
    try:
        subprocess.run([sys.executable, program], check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error running {program}: {e}")
    except KeyboardInterrupt:
        print(f"\n{program} terminated by user.")

def handle_client(client_socket, client_address):
    print(f"Connection established with {client_address}")
    current_program = None
    active_keys = set()  # Keep track of currently pressed keys
    last_command_time = time.time()  # Track when the last command was received
    
    try:
        while True:
            # Set a timeout for receiving commands
            client_socket.settimeout(0.1)  # 100ms timeout
            try:
                command = client_socket.recv(1024).decode().strip()
                if not command:
                    break
                last_command_time = time.time()  # Update last command time
            except socket.timeout:
                # If no command received for 200ms, release all keys
                if time.time() - last_command_time > 0.2:
                    for key in active_keys:
                        keyboard.release(key)
                    active_keys.clear()
                continue

            print(f"Received command: {command}")

            # Check if it's a control command
            if command.startswith("CONTROL "):
                cmd = command[8:]  # Remove "CONTROL " prefix
                
                # Release all previously pressed keys
                for key in active_keys:
                    keyboard.release(key)
                active_keys.clear()
                
                if cmd == "FORWARD":
                    keyboard.press('w')
                    active_keys.add('w')
                elif cmd == "BACKWARD":
                    keyboard.press('s')
                    active_keys.add('s')
                elif cmd == "LEFT":
                    keyboard.press('a')
                    active_keys.add('a')
                elif cmd == "RIGHT":
                    keyboard.press('d')
                    active_keys.add('d')
                elif cmd.startswith("SPEED "):
                    try:
                        speed = int(cmd[6:])
                        if speed > 150:  # If speed is higher than default
                            keyboard.press('+')
                            active_keys.add('+')
                        elif speed < 150:  # If speed is lower than default
                            keyboard.press('-')
                            active_keys.add('-')
                    except ValueError:
                        print("Invalid speed value")
                elif cmd == "PAN LEFT":
                    keyboard.press('j')
                    active_keys.add('j')
                elif cmd == "PAN RIGHT":
                    keyboard.press('l')
                    active_keys.add('l')
                elif cmd == "TILT UP":
                    keyboard.press('i')
                    active_keys.add('i')
                elif cmd == "TILT DOWN":
                    keyboard.press('k')
                    active_keys.add('k')
                elif cmd == "STOP":
                    # Release all keys
                    for key in active_keys:
                        keyboard.release(key)
                    active_keys.clear()
                continue

            # Terminate any currently running program
            if current_program:
                print(f"Terminating current program: {current_program}")
                terminate_program(current_program)
                time.sleep(1)  # Give time for process to terminate

            if command == "1":
                print("Executing Camera Detector")
                current_program = 'camera_detector.py'
                try:
                    subprocess.run([sys.executable, current_program], check=True)
                except Exception as e:
                    print(f"Error running camera_detector.py: {e}")
            elif command == "2":
                print("Executing Stream")
                current_program = 'stream.py'
                try:
                    subprocess.run([sys.executable, current_program], check=True)
                except Exception as e:
                    print(f"Error running stream.py: {e}")
            elif command == "3":
                print("Executing Away from Human")
                current_program = 'awayfromhuman.py'
                try:
                    subprocess.run([sys.executable, current_program], check=True)
                except Exception as e:
                    print(f"Error running awayfromhuman.py: {e}")
            else:
                print("Unknown command")
                current_program = None
    except Exception as e:
        print(f"Error handling client: {e}")
    finally:
        # Release all keys when client disconnects
        for key in active_keys:
            keyboard.release(key)
        # Terminate any running program when client disconnects
        if current_program:
            terminate_program(current_program)
        client_socket.close()

def socket_server():
    # Get and print local IP address
    local_ip = get_local_ip()
    print(f"\nServer IP Address: {local_ip}")
    
    # Set up the server
    HOST = '0.0.0.0'  # Listen on all interfaces
    PORT = 9999       # Port number
    
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((HOST, PORT))
    server_socket.listen(1)
    
    print(f"Server listening on port {PORT}...")
    print(f"App should connect to: {local_ip}:{PORT}")
    
    try:
        while True:
            # Wait for a client (mobile) to connect
            client_socket, client_address = server_socket.accept()
            print(f"New connection from {client_address}")
            # Handle client in a separate thread
            client_thread = threading.Thread(target=handle_client, args=(client_socket, client_address))
            client_thread.daemon = True
            client_thread.start()
            
    except KeyboardInterrupt:
        print("\nShutting down server...")
    except Exception as e:
        print(f"Server error: {e}")
    finally:
        server_socket.close()

def main():
    print("\nWelcome to the Remote Control Interface!")
    print("Starting Car Control in the background...")
    
    # Start car.py in a separate process
    car_process = multiprocessing.Process(target=run_program, args=('car.py',))
    car_process.start()
    
    # Start socket server in a separate thread
    server_thread = threading.Thread(target=socket_server)
    server_thread.daemon = True
    server_thread.start()
    
    try:
        # Keep the main thread alive
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nTerminating all processes...")
        # Terminate any running programs
        terminate_program('camera_detector.py')
        terminate_program('stream.py')
        terminate_program('awayfromhuman.py')
        car_process.terminate()
        car_process.join()
    except Exception as e:
        print(f"An error occurred: {e}")
        # Terminate any running programs
        terminate_program('camera_detector.py')
        terminate_program('stream.py')
        terminate_program('awayfromhuman.py')
        car_process.terminate()
        car_process.join()

if __name__ == "__main__":
    main() 