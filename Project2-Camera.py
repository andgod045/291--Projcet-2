import cv2
import mediapipe as mp
import time
import threading
import socket
import requests
import numpy as np
from collections import deque

ESP32_IP = '172.20.10.3'   # replace if needed
UDP_PORT = 1234
STREAM_URL = f'http://{ESP32_IP}:81/stream'

udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# ---------------- UDP command helpers ----------------

CMD_TRACKING_ENABLE = 0x03
CMD_LEFT            = 0x10
CMD_RIGHT           = 0x11
CMD_FORWARD         = 0x12
CMD_STOP            = 0x13

def send_raw(cmd: int) -> None:
    try:
        udp_sock.sendto(bytes([cmd]), (ESP32_IP, UDP_PORT))
    except Exception as e:
        print(f"UDP send failed: {e}")

def send_tracking_enable() -> None:
    send_raw(CMD_TRACKING_ENABLE)

def send_left() -> None:
    send_raw(CMD_LEFT)

def send_right() -> None:
    send_raw(CMD_RIGHT)

def send_forward() -> None:
    send_raw(CMD_FORWARD)

def send_stop() -> None:
    send_raw(CMD_STOP)

# ---------------- MediaPipe setup ----------------

mp_pose = mp.solutions.pose
mp_draw = mp.solutions.drawing_utils

pose = mp_pose.Pose(
    min_detection_confidence=0.7,
    min_tracking_confidence=0.5
)

# ---------------- Tracking tuning ----------------

TARGET_SHOULDER_WIDTH = 0.30
ARMS_HISTORY_LENGTH = 8
NO_TARGET_STOP_DELAY = 0.20
TRACKING_ENABLE_REPEAT = 2.0

# EMA smoothing for nose x
CX_SMOOTHING = 0.72

# Proportional steering using pulse blending
CENTER_X = 0.50
CENTER_DEADBAND = 0.05
FULL_TURN_OFFSET = 0.30

CONTROL_PERIOD = 0.14
MIN_TURN_FRACTION = 0.10
MAX_TURN_FRACTION = 0.92
TURN_CURVE = 1.35

# Refresh discrete commands so ESP32 keeps getting updates
COMMAND_REFRESH = 0.08
FORWARD_REFRESH = 0.18

# Inference frame size
INFER_W = 160
INFER_H = 120

# ---------------- Tracking helpers ----------------

def check_arms_raised(landmarks) -> bool:
    left_wrist = landmarks[mp_pose.PoseLandmark.LEFT_WRIST]
    right_wrist = landmarks[mp_pose.PoseLandmark.RIGHT_WRIST]
    left_shoulder = landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER]
    right_shoulder = landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER]
    return left_wrist.y < left_shoulder.y and right_wrist.y < right_shoulder.y

def get_shoulder_width(landmarks) -> float:
    left_shoulder = landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER]
    right_shoulder = landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER]
    return abs(right_shoulder.x - left_shoulder.x)

def get_distance_label(shoulder_width: float) -> str:
    diff = shoulder_width - TARGET_SHOULDER_WIDTH
    if diff > 0.05:
        return 'TOO CLOSE'
    elif diff < -0.05:
        return 'TOO FAR'
    else:
        return 'GOOD'

def compute_turn_mix(cx: float):
    """
    Returns:
      desired_turn: 'LEFT', 'RIGHT', or 'FORWARD'
      turn_fraction: 0.0 to 1.0

    Because your robot's turning direction is swapped relative to image motion,
    a person on the LEFT side of frame needs command RIGHT, and vice versa.
    """
    offset = cx - CENTER_X

    if abs(offset) <= CENTER_DEADBAND:
        return 'FORWARD', 0.0

    mag = (abs(offset) - CENTER_DEADBAND) / (FULL_TURN_OFFSET - CENTER_DEADBAND)
    mag = max(0.0, min(1.0, mag))
    mag = mag ** TURN_CURVE

    turn_fraction = MIN_TURN_FRACTION + mag * (MAX_TURN_FRACTION - MIN_TURN_FRACTION)

    # swapped to match your robot
    if offset < 0:
        desired_turn = 'RIGHT'
    else:
        desired_turn = 'LEFT'

    return desired_turn, turn_fraction

# ---------------- Threaded MJPEG frame capture ----------------

class FrameGrabber(threading.Thread):
    def __init__(self, url: str):
        super().__init__(daemon=True)
        self.url = url
        self.frame = None
        self.lock = threading.Lock()
        self.running = True
        self.last_frame_time = 0.0

    def run(self):
        while self.running:
            try:
                print(f"Connecting to stream: {self.url}")
                with requests.get(self.url, stream=True, timeout=(2, 2)) as response:
                    response.raise_for_status()
                    byte_buffer = b""
                    self.last_frame_time = time.time()

                    for chunk in response.iter_content(chunk_size=4096):
                        if not self.running:
                            return

                        now = time.time()

                        if now - self.last_frame_time > 1.5:
                            print("Frame watchdog timeout, reconnecting...")
                            break

                        if not chunk:
                            continue

                        byte_buffer += chunk

                        while True:
                            start = byte_buffer.find(b'\xff\xd8')
                            end = byte_buffer.find(b'\xff\xd9')

                            if start == -1 or end == -1 or end <= start:
                                if len(byte_buffer) > 65536:
                                    byte_buffer = byte_buffer[-32768:]
                                break

                            jpg = byte_buffer[start:end + 2]
                            byte_buffer = byte_buffer[end + 2:]

                            img = cv2.imdecode(
                                np.frombuffer(jpg, dtype=np.uint8),
                                cv2.IMREAD_COLOR
                            )

                            if img is not None:
                                with self.lock:
                                    self.frame = img
                                self.last_frame_time = time.time()

            except Exception as e:
                if self.running:
                    print(f"Stream error: {e}")

            if self.running:
                print("Reconnecting stream...")
                time.sleep(0.2)

    def get_frame(self):
        with self.lock:
            return None if self.frame is None else self.frame.copy()

    def stop(self):
        self.running = False

# ---------------- Threaded inference ----------------

class InferenceEngine(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.input_frame = None
        self.pose_results = None
        self.input_lock = threading.Lock()
        self.output_lock = threading.Lock()
        self.running = True

    def run(self):
        while self.running:
            with self.input_lock:
                frame = None if self.input_frame is None else self.input_frame.copy()

            if frame is None:
                time.sleep(0.005)
                continue

            small = cv2.resize(frame, (INFER_W, INFER_H))
            rgb = cv2.cvtColor(small, cv2.COLOR_BGR2RGB)
            pose_res = pose.process(rgb)

            with self.output_lock:
                self.pose_results = pose_res

            time.sleep(0.001)

    def set_frame(self, frame):
        with self.input_lock:
            self.input_frame = frame.copy()

    def get_results(self):
        with self.output_lock:
            return self.pose_results

    def stop(self):
        self.running = False

# ---------------- Main ----------------

grabber = FrameGrabber(STREAM_URL)
grabber.start()
print("Camera grabber started")

engine = InferenceEngine()
engine.start()
print("Inference engine started")

tracking_locked = False
arms_raised_history = []
last_seen_target_time = 0.0
last_tracking_enable_time = 0.0
last_good_frame_time = time.time()

last_sent_command = None
last_command_send_time = 0.0
last_forward_send_time = 0.0

smoothed_cx = 0.5
control_cycle_start = time.time()

print("Tracking controller ready")
print("Robot should already be in TRACKING mode on the EFM8.")

def send_command(command: str, force: bool = False) -> None:
    global last_sent_command, last_command_send_time, last_forward_send_time

    now = time.time()

    if not force and command == last_sent_command:
        if command == 'FORWARD':
            if (now - last_forward_send_time) < FORWARD_REFRESH:
                return
        else:
            if (now - last_command_send_time) < COMMAND_REFRESH:
                return

    if command == 'LEFT':
        send_left()
    elif command == 'RIGHT':
        send_right()
    elif command == 'FORWARD':
        send_forward()
        last_forward_send_time = now
    else:
        send_stop()

    last_sent_command = command
    last_command_send_time = now
    print(f"Sent: {command}")

def send_stop_if_needed() -> None:
    global last_sent_command, last_command_send_time
    if last_sent_command != 'STOP':
        send_stop()
        last_sent_command = 'STOP'
        last_command_send_time = time.time()
        print("Sent: STOP")

try:
    while True:
        frame = grabber.get_frame()
        if frame is None:
            if time.time() - last_good_frame_time > 1.5:
                print("No frame available")
            time.sleep(0.005)
            continue

        last_good_frame_time = time.time()

        frame = cv2.flip(frame, 1)
        height, width = frame.shape[:2]

        engine.set_frame(frame)
        pose_results = engine.get_results()

        now = time.time()

        if now - last_tracking_enable_time > TRACKING_ENABLE_REPEAT:
            send_tracking_enable()
            last_tracking_enable_time = now

        found_pose = False
        shoulder_width = None
        raw_cx_norm = None
        cx_norm = None
        desired_turn = 'FORWARD'
        turn_fraction = 0.0
        blended_command = 'STOP'

        if pose_results and pose_results.pose_landmarks:
            found_pose = True
            landmarks = pose_results.pose_landmarks.landmark

            nose = landmarks[mp_pose.PoseLandmark.NOSE]
            raw_cx_norm = nose.x
            smoothed_cx = (CX_SMOOTHING * smoothed_cx) + ((1.0 - CX_SMOOTHING) * raw_cx_norm)

            cx_norm = smoothed_cx
            cx = int(raw_cx_norm * width)
            cy = int(nose.y * height)
            scx = int(smoothed_cx * width)

            if not tracking_locked:
                arms_up = check_arms_raised(landmarks)
                arms_raised_history.append(arms_up)
                if len(arms_raised_history) > ARMS_HISTORY_LENGTH:
                    arms_raised_history.pop(0)

                if len(arms_raised_history) == ARMS_HISTORY_LENGTH and all(arms_raised_history):
                    tracking_locked = True
                    last_seen_target_time = now
                    arms_raised_history = []
                    smoothed_cx = raw_cx_norm
                    control_cycle_start = now
                    print("Target locked")

                cv2.circle(frame, (cx, cy), 10, (0, 255, 255), -1)
                cv2.putText(frame, 'Raise both arms to lock on', (10, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

                if arms_raised_history:
                    filled = int((len([x for x in arms_raised_history if x]) / ARMS_HISTORY_LENGTH) * 100)
                    cv2.putText(frame, f'Lock-on: {filled}%', (10, 70),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

                send_stop_if_needed()

            else:
                last_seen_target_time = now
                shoulder_width = get_shoulder_width(landmarks)

                desired_turn, turn_fraction = compute_turn_mix(cx_norm)

                # Time-sliced proportional steering:
                # small offset => mostly forward, brief turn
                # big offset   => mostly turn
                elapsed = now - control_cycle_start
                if elapsed >= CONTROL_PERIOD:
                    control_cycle_start = now
                    elapsed = 0.0

                turn_time = turn_fraction * CONTROL_PERIOD

                if desired_turn == 'FORWARD':
                    blended_command = 'FORWARD'
                else:
                    if elapsed < turn_time:
                        blended_command = desired_turn
                    else:
                        blended_command = 'FORWARD'

                send_command(blended_command)

                cv2.circle(frame, (cx, cy), 5, (160, 160, 160), -1)
                cv2.circle(frame, (scx, cy), 8, (0, 255, 0), -1)

                cv2.line(frame, (int((CENTER_X - CENTER_DEADBAND) * width), 0),
                         (int((CENTER_X - CENTER_DEADBAND) * width), height), (255, 255, 0), 1)
                cv2.line(frame, (int((CENTER_X + CENTER_DEADBAND) * width), 0),
                         (int((CENTER_X + CENTER_DEADBAND) * width), height), (255, 255, 0), 1)

                distance_label = get_distance_label(shoulder_width)

                cv2.putText(frame, 'Track: LOCKED', (10, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(frame, f'x raw: {raw_cx_norm:.2f}', (10, 75),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.putText(frame, f'x smooth: {cx_norm:.2f}', (10, 100),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.putText(frame, f'turn cmd: {desired_turn}', (10, 125),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.putText(frame, f'turn frac: {turn_fraction:.2f}', (10, 150),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.putText(frame, f'sent: {last_sent_command}', (10, 175),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.putText(frame, f'dist: {distance_label}', (10, 200),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        if not found_pose:
            arms_raised_history = []

            if tracking_locked and (now - last_seen_target_time > NO_TARGET_STOP_DELAY):
                send_stop_if_needed()
                cv2.putText(frame, 'Target lost', (10, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
            elif not tracking_locked:
                send_stop_if_needed()

        if tracking_locked:
            cv2.putText(frame, 'Press L to unlock', (10, 230),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        cv2.imshow('Robot Tracking Controller', frame)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break
        elif key == ord('l'):
            tracking_locked = False
            arms_raised_history = []
            smoothed_cx = 0.5
            send_stop_if_needed()
            print("Tracking unlocked")

finally:
    try:
        send_stop()
    except:
        pass
    grabber.stop()
    engine.stop()
    udp_sock.close()
    cv2.destroyAllWindows()
