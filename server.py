
from flask import Flask, Response, render_template, jsonify, send_from_directory, request
import cv2
import threading
import time
import base64
import requests
import socket
import json
import sys
import os
import math
import re
from collections import deque

STREAM_URL = "http://10.42.0.56:8080/?action=stream"
PI_IP = "10.42.0.56"
PI_PORT = 9000
OLLAMA_URL = "http://localhost:11434/api/generate"
OLLAMA_MODEL = "gemma3:12b"
EVAL_INTERVAL = 1.5
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
MISSION_FILE = "mission.json"
WEB_PORT = 5000
FRAMES_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "captured_frames")
SHOW_CV_WINDOW = False
DEBUG = True

os.makedirs(FRAMES_DIR, exist_ok=True)

app = Flask(__name__, template_folder=os.path.join(os.path.dirname(os.path.abspath(__file__)), "templates"))

stream = None
mission_state = None
route_tracker = None
event_log = None
pi_connected = False
returning = False
mission_started = False


class EventLog:
    def __init__(self, maxlen=100):
        self.events = deque(maxlen=maxlen)
        self.lock = threading.Lock()

    def add(self, event_type, **kwargs):
        with self.lock:
            self.events.append({
                "type": event_type,
                "timestamp": time.strftime("%H:%M:%S"),
                **kwargs
            })

    def get_all(self):
        with self.lock:
            return list(self.events)


class VideoStream:
    def __init__(self, src):
        self.cap = cv2.VideoCapture(src)
        self.ret, self.frame = self.cap.read()
        self.stopped = False
        self.lock = threading.Lock()
        threading.Thread(target=self.update, daemon=True).start()

    def update(self):
        while not self.stopped:
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.1)
                continue
            frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))
            with self.lock:
                self.ret, self.frame = ret, frame

    def read(self):
        with self.lock:
            if self.ret and self.frame is not None:
                return True, self.frame.copy()
            return False, None

    def stop(self):
        self.stopped = True
        self.cap.release()


class RouteTracker:
    def __init__(self):
        self.x, self.y, self.heading = 0.0, 0.0, 0.0
        self.points = [{"x": 0, "y": 0, "heading": 0, "command": "START", "frame": None}]
        self.frame_counter = 0
        self.lock = threading.Lock()

    def update(self, command, frame=None):
        parts = command.split()
        if len(parts) != 2:
            return
        direction, duration = parts[0], int(parts[1])

        frame_path = None
        if frame is not None:
            self.frame_counter += 1
            filename = f"frame_{self.frame_counter:04d}.jpg"
            cv2.imwrite(os.path.join(FRAMES_DIR, filename), frame)
            frame_path = filename

        dist = duration * 0.08
        with self.lock:
            if direction == 'F':
                self.x += dist * math.sin(math.radians(self.heading))
                self.y -= dist * math.cos(math.radians(self.heading))
            elif direction == 'B':
                self.x -= dist * math.sin(math.radians(self.heading))
                self.y += dist * math.cos(math.radians(self.heading))
            elif direction == 'L':
                self.heading = (self.heading - (duration / 400.0) * 90) % 360
            elif direction == 'R':
                self.heading = (self.heading + (duration / 400.0) * 90) % 360

            self.points.append({
                "x": round(self.x, 2), "y": round(self.y, 2),
                "heading": round(self.heading, 1),
                "command": command, "frame": frame_path
            })

    def get_points(self):
        with self.lock:
            return list(self.points)


class RouteMetrics:
    def __init__(self):
        self.lock = threading.Lock()
        self.start_time = time.time()
        self.total_commands = 0
        self.forward_commands = 0
        self.turn_commands = 0
        self.backward_commands = 0
        self.obstacle_deviations = 0
        self.total_duration_ms = 0
        self.steps_completed = 0
        self.steps_skipped = 0

    def record(self, command, planned_action):
        with self.lock:
            self.total_commands += 1
            parts = command.split()
            if len(parts) == 2:
                d, dur = parts[0], int(parts[1])
                self.total_duration_ms += dur
                if d == 'F': self.forward_commands += 1
                elif d == 'B': self.backward_commands += 1
                elif d in ('L','R'): self.turn_commands += 1
                pd = planned_action.split()[0] if planned_action else None
                if pd and d != pd:
                    self.obstacle_deviations += 1

    def step_done(self, skipped=False):
        with self.lock:
            if skipped: self.steps_skipped += 1
            else: self.steps_completed += 1

    def get_summary(self):
        with self.lock:
            elapsed = time.time() - self.start_time
            eff = (self.forward_commands / max(self.total_commands, 1)) * 100
            return {
                "elapsed_time": round(elapsed, 1),
                "total_commands": self.total_commands,
                "forward": self.forward_commands,
                "turns": self.turn_commands,
                "backward": self.backward_commands,
                "deviations": self.obstacle_deviations,
                "motor_time_ms": self.total_duration_ms,
                "steps_completed": self.steps_completed,
                "steps_skipped": self.steps_skipped,
                "path_efficiency": round(eff, 1),
            }


class MissionState:
    def __init__(self, mission_data, metrics):
        self.name = mission_data["name"]
        self.description = mission_data.get("description", "")
        self.terrain_notes = mission_data.get("terrain_notes", "")
        self.steps = mission_data["steps"]
        self.current_step_index = 0
        self.step_attempts = 0
        self.command_history = deque(maxlen=10)
        self.completed = False
        self.lock = threading.Lock()
        self.metrics = metrics

    def get_current_step(self):
        with self.lock:
            if self.current_step_index >= len(self.steps):
                self.completed = True
                return None
            return self.steps[self.current_step_index]

    def record_command(self, command):
        with self.lock:
            planned = self.steps[self.current_step_index]["action"]
            self.command_history.append({
                "command": command, "planned": planned,
                "step_index": self.current_step_index,
                "attempt": self.step_attempts,
                "timestamp": time.strftime("%H:%M:%S")
            })
            self.step_attempts += 1
            self.metrics.record(command, planned)

    def advance_step(self, skipped=False):
        with self.lock:
            self.metrics.step_done(skipped=skipped)
            self.current_step_index += 1
            self.step_attempts = 0
            if self.current_step_index >= len(self.steps):
                self.completed = True

    def get_history_summary(self):
        with self.lock:
            return list(self.command_history)

    def get_state_dict(self):
        with self.lock:
            steps_out = []
            for i, s in enumerate(self.steps):
                if i < self.current_step_index: status = "completed"
                elif i == self.current_step_index and not self.completed: status = "current"
                else: status = "pending"
                steps_out.append({
                    "action": s["action"], "description": s["description"],
                    "max_attempts": s["max_attempts"], "status": status,
                    "attempts": self.step_attempts if status == "current" else 0
                })
            return {
                "name": self.name, "description": self.description,
                "current_step": self.current_step_index,
                "total_steps": len(self.steps),
                "completed": self.completed, "steps": steps_out
            }


def load_mission(path):
    try:
        with open(path, 'r') as f:
            data = json.load(f)
        for i, step in enumerate(data["steps"]):
            if "max_attempts" not in step:
                step["max_attempts"] = 10
        print(f"Loaded mission: '{data['name']}' ({len(data['steps'])} steps)")
        return data
    except Exception as e:
        print(f"Mission load error: {e}")
        sys.exit(1)

def get_distance_from_pi():
    global pi_connected
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(0.5)
        s.connect((PI_IP, PI_PORT))
        s.sendall(b"GET_DIST")
        dist = s.recv(1024).decode('utf-8')
        s.close()
        pi_connected = True
        return dist
    except:
        pi_connected = False
        return "unknown"

def send_to_pi(command):
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((PI_IP, PI_PORT))
        s.sendall(command.encode('utf-8'))
        s.close()
    except Exception as e:
        print(f"Pi send error: {e}")

def get_env_from_pi():

    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(0.5)
        s.connect((PI_IP, PI_PORT))
        s.sendall(b"GET_ENV")
        data = s.recv(1024).decode('utf-8')
        s.close()
        parts = data.split(',')
        if len(parts) == 2:
            return {"temp": parts[0], "hum": parts[1]}
    except:
        pass
    return {"temp": "--", "hum": "--"}

def reverse_command(cmd):

    parts = cmd.strip().split()
    if len(parts) != 2:
        return None
    d, dur = parts[0], parts[1]
    rev = {'F': 'B', 'B': 'F', 'L': 'R', 'R': 'L'}
    return f"{rev[d]} {dur}" if d in rev else None

def execute_return():

    global returning
    returning = True
    event_log.add("system", message="Return to base initiated")

    points = route_tracker.get_points()
    
    commands = [p['command'] for p in points if p['command'] != 'START']

    if not commands:
        event_log.add("system", message="No route to retrace")
        returning = False
        return

    total = len(commands)
    event_log.add("system", message=f"Retracing {total} movement commands...")
    print(f"\n>>> RETURN TO BASE: Retracing {total} commands")

    for i, cmd in enumerate(reversed(commands)):
        if not returning:  
            break

        rev = reverse_command(cmd)
        if not rev:
            continue

        print(f"  [{i+1}/{total}] {cmd} -> {rev}")
        event_log.add("return", step=i+1, total=total, original=cmd, reversed=rev)
        send_to_pi(rev)

        if stream:
            ret, frame = stream.read()
            route_tracker.update(rev, frame if ret else None)
        else:
            route_tracker.update(rev)

        duration_ms = int(rev.split()[1])
        time.sleep((duration_ms / 1000.0) + 0.3)

    event_log.add("system", message="Return to base complete")
    print(">>> Return to base complete\n")
    returning = False


def build_prompt(ms, dist):
    step = ms.get_current_step()
    if not step:
        return None
    history = ms.get_history_summary()
    hist_text = "None yet." if not history else "\n".join(
        f"  [{h['timestamp']}] Step {h['step_index']+1} att {h['attempt']+1}: "
        f"plan={h['planned']} sent={h['command']}" for h in history)
    met = ms.metrics.get_summary()
    return (
        f"You are the navigation AI for a lunar rover prototype.\n"
        f"Objective: complete the mission via SHORTEST, SAFEST path.\n"
        f"Scored on ROUTE OPTIMIZATION — minimize commands, deviations, time.\n\n"
        f"=== TERRAIN ===\n{ms.terrain_notes}\n\n"
        f"=== CURRENT STEP {ms.current_step_index+1}/{len(ms.steps)} ===\n"
        f"Planned: {step['action']}\nGoal: {step['description']}\n"
        f"Attempts: {ms.step_attempts}/{step['max_attempts']}\n\n"
        f"=== SENSOR ===\nDistance: {dist} cm\n\n"
        f"=== METRICS ===\nElapsed: {met['elapsed_time']}s | Cmds: {met['total_commands']} | "
        f"Devs: {met['deviations']} | Eff: {met['path_efficiency']}%\n\n"
        f"=== LAST {len(history)} COMMANDS ===\n{hist_text}\n\n"
        f"=== RULES ===\n"
        f"Reply EXACTLY ONE line:\n"
        f"  Movement: F 600, B 400, L 400, R 400\n"
        f"  NEXT = step accomplished\n"
        f"  SKIP = step impossible\n"
        f"If the goal involves going near an object:\n"
        f"  - Keep moving forward (e.g., your planned action) until you are close.\n"
        f"  - If Distance is between 20cm and 40cm, use short forward movements (e.g., F 150) to cautiously get closer.\n"
        f"  - If Distance <= 20cm, reply NEXT to complete the step.\n"
        f"  - Do NOT reply NEXT if distance is > 20cm.\n"
        f"If distance < 20cm, do NOT go forward.\n"
        f"Prefer planned action when clear. Dodge obstacles then resume plan.\n"
        f"After {step['max_attempts']} attempts, SKIP.\n"
    )

def ask_ai(frame, ms):
    global route_tracker, event_log
    if ms.completed:
        return
    dist = get_distance_from_pi()
    prompt = build_prompt(ms, dist)
    if not prompt:
        return

    _, buf = cv2.imencode('.jpg', frame)
    img_b64 = base64.b64encode(buf).decode('utf-8')

    try:
        resp = requests.post(OLLAMA_URL, json={
            "model": OLLAMA_MODEL, "prompt": prompt,
            "stream": False, "images": [img_b64]
        }).json()
        raw = resp.get('response', '').strip()
        if DEBUG:
            print(f"[AI] {raw}")
        if not raw:
            return

        decision = raw.split('\n')[0].strip().upper()
        step = ms.get_current_step()

        if decision == "NEXT":
            event_log.add("step_complete", step=ms.current_step_index + 1)
            ms.advance_step(skipped=False)
        elif decision == "SKIP":
            event_log.add("step_skip", step=ms.current_step_index + 1)
            ms.advance_step(skipped=True)
        else:
            parts = decision.split()
            cmd = None
            if len(parts) == 2 and parts[0] in 'FBLR':
                try:
                    int(parts[1])
                    cmd = decision
                except ValueError:
                    pass
            if not cmd:
                m = re.search(r'\b([FBLR])\s+(\d+)\b', raw.upper())
                if m:
                    cmd = f"{m.group(1)} {m.group(2)}"

            if cmd:
                planned = step['action'] if step else ""
                is_dev = cmd.split()[0] != planned.split()[0] if planned else False
                send_to_pi(cmd)
                ms.record_command(cmd)
                route_tracker.update(cmd, frame)
                event_log.add("deviation" if is_dev else "command",
                              step=ms.current_step_index + 1,
                              planned=planned, sent=cmd)

                step = ms.get_current_step()
                if step and ms.step_attempts >= step['max_attempts']:
                    event_log.add("step_skip", step=ms.current_step_index + 1,
                                  reason="max_attempts")
                    ms.advance_step(skipped=True)

        if ms.completed:
            event_log.add("mission_complete")

    except Exception as e:
        print(f"AI Error: {e}")
        event_log.add("error", message=str(e))


@app.route('/')
def index():
    return render_template('dashboard.html')

@app.route('/optimizer')
def optimizer():
    return render_template('optimizer.html')

@app.route('/heightmap.bin')
def serve_heightmap():
    return send_from_directory('sim', 'heightmap.bin')

@app.route('/api/state')
def api_state():
    env = get_env_from_pi()
    data = {
        "mission": mission_state.get_state_dict() if mission_state else {},
        "metrics": mission_state.metrics.get_summary() if mission_state else {},
        "route": route_tracker.get_points() if route_tracker else [],
        "events": event_log.get_all() if event_log else [],
        "distance": get_distance_from_pi(),
        "env": env,
        "pi_connected": pi_connected,
        "returning": returning,
        "started": mission_started,
    }
    return jsonify(data)

@app.route('/video_feed')
def video_feed():
    def gen():
        while True:
            if stream is None:
                time.sleep(0.5)
                continue
            ret, frame = stream.read()
            if not ret:
                time.sleep(0.1)
                continue
            _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
                   + buf.tobytes() + b'\r\n')
            time.sleep(0.033)
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/frames/<path:filename>')
def serve_frame(filename):
    return send_from_directory(FRAMES_DIR, filename)

@app.route('/api/terrain_map')
def terrain_map():

    points = route_tracker.get_points() if route_tracker else []
    
    forward_frames = [p['frame'] for p in points
                      if p.get('frame') and p['command'].startswith('F')]

    if not forward_frames:
        return Response(status=204)

    strips = []
    target_w = FRAME_WIDTH
    for fname in forward_frames:
        img = cv2.imread(os.path.join(FRAMES_DIR, fname))
        if img is None:
            continue
        h, w = img.shape[:2]
        
        crop = img[int(h * 0.65):, :]
        
        if crop.shape[1] != target_w:
            crop = cv2.resize(crop, (target_w, crop.shape[0]))
        strips.append(crop)

    if not strips:
        return Response(status=204)

    terrain = cv2.vconcat(strips)

    max_h = 3000
    if terrain.shape[0] > max_h:
        terrain = terrain[terrain.shape[0] - max_h:, :]

    _, buf = cv2.imencode('.jpg', terrain, [cv2.IMWRITE_JPEG_QUALITY, 85])
    return Response(buf.tobytes(), mimetype='image/jpeg',
                    headers={'Cache-Control': 'no-cache'})

@app.route('/api/translate', methods=['POST'])
def api_translate():

    data = request.get_json()
    texts = data.get('texts', [])
    if not texts:
        return jsonify({"translations": []})

    numbered = "\n".join(f"{i+1}. {t}" for i, t in enumerate(texts))
    prompt = (
        "Translate each numbered line below to Turkish. "
        "Return ONLY the translations, one per line, keeping the same numbering. "
        "Do not add any explanation.\n\n" + numbered
    )
    try:
        resp = requests.post(OLLAMA_URL, json={
            "model": OLLAMA_MODEL, "prompt": prompt, "stream": False
        }).json()
        raw = resp.get('response', '').strip()
        lines = []
        for line in raw.split('\n'):
            line = line.strip()
            if not line:
                continue
            
            if len(line) > 2 and line[0].isdigit():
                idx = line.find('.')
                if idx != -1 and idx < 4:
                    line = line[idx+1:].strip()
            lines.append(line)
        return jsonify({"translations": lines})
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route('/api/return', methods=['POST'])
def api_return():

    global returning
    if returning:
        return jsonify({"status": "already_returning"})
    if not pi_connected:
        return jsonify({"error": "Pi not connected"}), 503
    threading.Thread(target=execute_return, daemon=True).start()
    return jsonify({"status": "started"})

@app.route('/api/return/cancel', methods=['POST'])
def api_return_cancel():

    global returning
    returning = False
    event_log.add("system", message="Return cancelled")
    return jsonify({"status": "cancelled"})

@app.route('/api/start', methods=['POST'])
def api_start():
    global mission_started
    mission_started = True
    event_log.add("system", message="Mission started manually")
    return jsonify({"status": "started"})


if __name__ == "__main__":
    mission_path = sys.argv[1] if len(sys.argv) > 1 else MISSION_FILE
    mission_data = load_mission(mission_path)
    metrics = RouteMetrics()
    mission_state = MissionState(mission_data, metrics)
    route_tracker = RouteTracker()
    event_log = EventLog()

    event_log.add("system", message=f"Mission loaded: {mission_state.name}")

    print(f"\n{'='*55}")
    print(f"  LUNAR ROVER: {mission_state.name}")
    for i, s in enumerate(mission_state.steps):
        print(f"  Step {i+1}: [{s['action']}] {s['description']}")
    print(f"{'='*55}")

    stream = VideoStream(STREAM_URL)

    threading.Thread(
        target=lambda: app.run(host='0.0.0.0', port=WEB_PORT, debug=False,
                               use_reloader=False, threaded=True),
        daemon=True
    ).start()
    print(f"\n  Dashboard: http://localhost:{WEB_PORT}\n")

    last_eval = time.time()
    ai_busy = threading.Event()

    try:
        while True:
            ret, frame = stream.read()
            if not ret:
                time.sleep(0.1)
                continue

            if SHOW_CV_WINDOW:
                cv2.imshow("Rover", frame)

            if (mission_started and not mission_state.completed and not ai_busy.is_set()
                    and pi_connected and not returning
                    and time.time() - last_eval > EVAL_INTERVAL):
                def _ai(f, ms, flag):
                    flag.set()
                    try: ask_ai(f, ms)
                    finally: flag.clear()
                threading.Thread(target=_ai, args=(frame.copy(), mission_state, ai_busy),
                                 daemon=True).start()
                last_eval = time.time()

            if SHOW_CV_WINDOW:
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            else:
                time.sleep(0.03)

    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        stream.stop()
        if SHOW_CV_WINDOW:
            cv2.destroyAllWindows()
        if mission_state.metrics.total_commands > 0:
            print("\n--- FINAL METRICS ---")
            for k, v in mission_state.metrics.get_summary().items():
                print(f"  {k}: {v}")
