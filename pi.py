import serial
import socket
import threading
import time

ARDUINO_PORT = '/dev/ttyUSB0'
BAUD_RATE = 9600
HOST = '0.0.0.0'
PORT = 9000

SERVER_IP = "10.42.0.1"   
SERVER_PORT = 5000         
PING_INTERVAL = 3.0        
PING_TIMEOUT = 1.5         
MAX_FAILURES = 3           
MAX_HISTORY = 50           

latest_distance = "999"
latest_temp = "--"
latest_hum = "--"

command_history = []        
history_lock = threading.Lock()
retracing = False           
was_ever_connected = False  

def reverse_command(cmd):

    parts = cmd.strip().split()
    if len(parts) != 2:
        return None
    direction, duration = parts[0], parts[1]
    rev = {'F': 'B', 'B': 'F', 'L': 'R', 'R': 'L'}
    if direction in rev:
        return f"{rev[direction]} {duration}"
    return None

def check_server():

    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(PING_TIMEOUT)
        s.connect((SERVER_IP, SERVER_PORT))
        s.close()
        return True
    except:
        return False

def retrace_steps(ser):

    global retracing
    retracing = True

    with history_lock:
        steps = list(reversed(command_history))
        command_history.clear()

    print(f"\n{'!'*50}")
    print(f"  WIFI LOST — RETRACING {len(steps)} STEPS")
    print(f"{'!'*50}")

    for i, cmd in enumerate(steps):
        rev = reverse_command(cmd)
        if not rev:
            continue

        print(f"  [{i+1}/{len(steps)}] {cmd} -> {rev}")
        ser.write((rev + '\n').encode('utf-8'))

        duration_ms = int(rev.split()[1])
        time.sleep((duration_ms / 1000.0) + 0.3)

        if check_server():
            print(f"  WiFi RESTORED after {i+1} retrace steps!")
            break
    else:
        print("  Fully retraced. Waiting for WiFi...")

    retracing = False
    print("Retrace complete. Resuming normal operation.\n")

def connectivity_monitor(ser):

    global was_ever_connected, retracing
    fail_count = 0

    print("Waiting for server connection...")
    while not was_ever_connected:
        if check_server():
            was_ever_connected = True
            print(f"Server connection confirmed ({SERVER_IP}:{SERVER_PORT})")
        time.sleep(2)

    while True:
        time.sleep(PING_INTERVAL)

        if retracing:
            continue

        if check_server():
            if fail_count > 0:
                print(f"WiFi OK (recovered after {fail_count} failures)")
            fail_count = 0
        else:
            fail_count += 1
            print(f"WiFi check FAILED ({fail_count}/{MAX_FAILURES})")

            if fail_count >= MAX_FAILURES:
                with history_lock:
                    has_history = len(command_history) > 0
                if has_history:
                    retrace_steps(ser)
                else:
                    print("  No command history to retrace. Waiting...")
                fail_count = 0

def arduino_reader(ser):
    global latest_distance, latest_temp, latest_hum
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').strip()
            if line.startswith("DIST:"):
                latest_distance = line.split(":")[1]
            elif line.startswith("TEMP:"):
                latest_temp = line.split(":")[1]
            elif line.startswith("HUM:"):
                latest_hum = line.split(":")[1]

def start_server():
    global latest_distance, was_ever_connected
    try:
        ser = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=1)
        print(f"Connected to Arduino on {ARDUINO_PORT}")
    except Exception as e:
        print(f"Arduino connection error: {e}")
        return

    threading.Thread(target=arduino_reader, args=(ser,), daemon=True).start()
    threading.Thread(target=connectivity_monitor, args=(ser,), daemon=True).start()

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen(5)
    print(f"Relay server listening on port {PORT}")

    while True:
        conn, addr = server.accept()
        try:
            data = conn.recv(1024).decode('utf-8').strip()
            if data == "GET_DIST":
                conn.sendall(latest_distance.encode('utf-8'))
            elif data == "GET_ENV":
                conn.sendall(f"{latest_temp},{latest_hum}".encode('utf-8'))
            elif data:
                
                was_ever_connected = True

                if retracing:
                    
                    print(f"  Ignoring command during retrace: {data}")
                else:
                    
                    ser.write((data + '\n').encode('utf-8'))
                    
                    parts = data.strip().split()
                    if len(parts) == 2 and parts[0] in ('F', 'B', 'L', 'R'):
                        with history_lock:
                            command_history.append(data)
                            
                            if len(command_history) > MAX_HISTORY:
                                command_history.pop(0)
                        print(f"  CMD: {data} (history: {len(command_history)})")
        finally:
            conn.close()

if __name__ == "__main__":
    start_server()
