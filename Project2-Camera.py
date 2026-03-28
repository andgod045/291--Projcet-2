import cv2
import mediapipe as mp
import serial
import serial.tools.list_ports
import time
import threading
import socket
import numpy as np

def connect_serial():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        print(f'Found port: {port.device}')
    try:
        ser = serial.Serial('COM3', 9600, timeout=1)
        print(f'Connected to COM3')
        return ser
    except:
        print('No serial connection, running in demo mode')
        return None

ser = connect_serial()

ESP32_IP = 'xxxxx' #depends on given esp32 ip address once initialized, will vary from network to network
UDP_PORT = 1234
udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_cmd(val):
    try:
        cmd_byte = bytes([int(val, 16)])
        udp_sock.sendto(cmd_byte, (ESP32_IP, UDP_PORT))
        if ser:
            ser.write(cmd_byte)
    except:
        pass

def send_mode(mode):
    mode_map = {
        'RC':         '0x01',
        'GUIDE_WIRE': '0x02',
        'TRACKING':   '0x03',
        'STOP':       '0x04',
        'SCAN':       '0x05'
    }
    if mode in mode_map:
        send_cmd(mode_map[mode])
        print(f'Sent mode: {mode}')

def send_steering(command):
    steering_map = {
        'LEFT':     '0x10',
        'RIGHT':    '0x11',
        'STRAIGHT': '0x12',
        'STOP':     '0x13'
    }
    if command in steering_map:
        send_cmd(steering_map[command])

def send_speed(command):
    speed_map = {
        'FORWARD':  '0x20',
        'BACKWARD': '0x21',
        'NORMAL':   '0x22'
    }
    if command in speed_map:
        send_cmd(speed_map[command])

# hands setup
mp_hands = mp.solutions.hands
mp_draw = mp.solutions.drawing_utils
hands = mp_hands.Hands(
    max_num_hands=1,
    min_detection_confidence=0.8,
    min_tracking_confidence=0.6
)

# pose setup
mp_pose = mp.solutions.pose
pose = mp_pose.Pose(
    min_detection_confidence=0.7,
    min_tracking_confidence=0.5
)

FINGER_TIPS = [8, 12, 16, 20]
FINGER_BASES = [6, 10, 14, 18]
THUMB_TIP = 4
THUMB_BASE = 2

COOLDOWN = 2
TARGET_SHOULDER_WIDTH = 0.3
DISTANCE_TOLERANCE = 0.05

MODE_CYCLE = ['RC', 'TRACKING', 'GUIDE_WIRE']

def next_mode(current):
    idx = MODE_CYCLE.index(current) if current in MODE_CYCLE else 0
    return MODE_CYCLE[(idx + 1) % len(MODE_CYCLE)]

def count_fingers(landmarks):
    count = 0
    for tip, base in zip(FINGER_TIPS, FINGER_BASES):
        if landmarks[tip].y < landmarks[base].y:
            count += 1
    if landmarks[THUMB_TIP].x < landmarks[THUMB_BASE].x:
        count += 1
    return count

def check_arms_raised(landmarks):
    left_wrist = landmarks[mp_pose.PoseLandmark.LEFT_WRIST]
    right_wrist = landmarks[mp_pose.PoseLandmark.RIGHT_WRIST]
    left_shoulder = landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER]
    right_shoulder = landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER]
    return left_wrist.y < left_shoulder.y and right_wrist.y < right_shoulder.y

def get_shoulder_width(landmarks):
    left_shoulder = landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER]
    right_shoulder = landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER]
    return abs(right_shoulder.x - left_shoulder.x)

def get_tracking_command(landmarks):
    nose = landmarks[mp_pose.PoseLandmark.NOSE]
    cx = nose.x
    if cx < 0.4:
        return 'LEFT'
    elif cx > 0.6:
        return 'RIGHT'
    else:
        return 'STRAIGHT'

def get_distance_command(shoulder_width):
    diff = shoulder_width - TARGET_SHOULDER_WIDTH
    if diff > DISTANCE_TOLERANCE:
        return 'BACKWARD'
    elif diff < -DISTANCE_TOLERANCE:
        return 'FORWARD'
    else:
        return 'NORMAL'

# --- THREADED FRAME CAPTURE ---
class FrameGrabber(threading.Thread):
    def __init__(self, url):
        super().__init__(daemon=True)
        self.url = url
        self.frame = None
        self.lock = threading.Lock()
        self.cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.running = True

    def run(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                with self.lock:
                    self.frame = frame
            else:
                print('Frame failed, reconnecting...')
                time.sleep(0.5)
                self.cap.open(self.url, cv2.CAP_FFMPEG)
                self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    def get_frame(self):
        with self.lock:
            return self.frame.copy() if self.frame is not None else None

    def stop(self):
        self.running = False
        self.cap.release()

# --- THREADED MEDIAPIPE INFERENCE ---
class InferenceEngine(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.input_frame = None
        self.hand_results = None
        self.pose_results = None
        self.input_lock = threading.Lock()
        self.output_lock = threading.Lock()
        self.running = True

    def run(self):
        while self.running:
            with self.input_lock:
                frame = self.input_frame
            if frame is None:
                time.sleep(0.01)
                continue
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            hand_res = hands.process(rgb)
            pose_res = pose.process(rgb)
            with self.output_lock:
                self.hand_results = hand_res
                self.pose_results = pose_res

    def set_frame(self, frame):
        with self.input_lock:
            self.input_frame = frame.copy()

    def get_results(self):
        with self.output_lock:
            return self.hand_results, self.pose_results

    def stop(self):
        self.running = False

grabber = FrameGrabber(f'http://{ESP32_IP}:81/stream')
grabber.start()
print(f"Camera grabber started")

engine = InferenceEngine()
engine.start()
print("Inference engine started")

finger_history = []
arms_raised_history = []
HISTORY_LENGTH = 5
ARMS_HISTORY_LENGTH = 8
current_mode = 'RC'
previous_mode = 'RC'
tracking_locked = False
last_distance_command = 'NORMAL'
last_steering_command = 'STRAIGHT'
last_switch_time = 0

print("Robot controller ready. Mode: RC")

while True:
    frame = grabber.get_frame()
    if frame is None:
        time.sleep(0.01)
        continue

    frame = cv2.flip(frame, 1)
    height, width = frame.shape[:2]

    # feed latest frame to inference engine
    engine.set_frame(frame)

    # get latest inference results (may be from previous frame, that's fine)
    hand_results, pose_results = engine.get_results()

    fingers = 0

    # --- HAND GESTURE LOGIC ---
    if hand_results and hand_results.multi_hand_landmarks:
        for hand_landmarks in hand_results.multi_hand_landmarks:
            mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
            fingers = count_fingers(hand_landmarks.landmark)

            if current_mode == 'TRACKING' and tracking_locked:
                finger_history.append(fingers)
                if len(finger_history) > HISTORY_LENGTH:
                    finger_history.pop(0)
                if len(finger_history) == HISTORY_LENGTH and len(set(finger_history)) == 1:
                    if finger_history[0] == 2:
                        now = time.time()
                        if now - last_switch_time > COOLDOWN:
                            tracking_locked = False
                            print("Target lost")
                            finger_history = []
                            last_switch_time = now
                continue

            finger_history.append(fingers)
            if len(finger_history) > HISTORY_LENGTH:
                finger_history.pop(0)

            if len(finger_history) == HISTORY_LENGTH and len(set(finger_history)) == 1:
                stable_count = finger_history[0]
                now = time.time()

                if now - last_switch_time > COOLDOWN:
                    if stable_count == 5:
                        previous_mode = current_mode
                        current_mode = next_mode(current_mode)
                        tracking_locked = False
                        send_mode(current_mode)
                        print(f"Mode: {current_mode}")
                        last_switch_time = now
                        finger_history = []

                    elif stable_count == 3:
                        if current_mode == 'STOP':
                            current_mode = previous_mode
                            print(f"Resuming {current_mode}")
                        else:
                            previous_mode = current_mode
                            current_mode = 'STOP'
                            print("Stopped")
                        tracking_locked = False
                        send_mode(current_mode)
                        last_switch_time = now
                        finger_history = []
                else:
                    remaining = COOLDOWN - (now - last_switch_time)
                    cv2.putText(frame, f'Cooldown: {remaining:.1f}s', (10, 210),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)

        cv2.putText(frame, f'Fingers: {fingers}', (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    else:
        finger_history = []

    # --- POSE / TRACKING LOGIC ---
    if pose_results and pose_results.pose_landmarks:
        landmarks = pose_results.pose_landmarks.landmark

        nose = landmarks[mp_pose.PoseLandmark.NOSE]
        cx = int(nose.x * width)
        cy = int(nose.y * height)

        if current_mode == 'TRACKING':
            arms_up = check_arms_raised(landmarks)
            arms_raised_history.append(arms_up)
            if len(arms_raised_history) > ARMS_HISTORY_LENGTH:
                arms_raised_history.pop(0)

            if (len(arms_raised_history) == ARMS_HISTORY_LENGTH and
                    all(arms_raised_history)):
                if not tracking_locked:
                    print("Target locked")
                tracking_locked = True
                arms_raised_history = []

            if tracking_locked:
                mp_draw.draw_landmarks(
                    frame,
                    pose_results.pose_landmarks,
                    mp_pose.POSE_CONNECTIONS
                )
                cv2.circle(frame, (cx, cy), 10, (0, 255, 0), -1)
                cv2.rectangle(frame,
                    (cx - 50, cy - 50),
                    (cx + 50, cy + 50),
                    (0, 255, 0), 2)

                steering = get_tracking_command(landmarks)
                if steering != last_steering_command:
                    send_steering(steering)
                    last_steering_command = steering

                shoulder_width = get_shoulder_width(landmarks)
                distance_cmd = get_distance_command(shoulder_width)
                if distance_cmd != last_distance_command:
                    send_speed(distance_cmd)
                    last_distance_command = distance_cmd

                bar_width = int(shoulder_width * width)
                target_x = int(TARGET_SHOULDER_WIDTH * width)
                cv2.rectangle(frame, (10, height - 30),
                              (bar_width, height - 10), (0, 255, 0), -1)
                cv2.line(frame, (target_x, height - 35),
                         (target_x, height - 5), (0, 0, 255), 2)
                cv2.putText(frame, f'Distance: {distance_cmd}', (10, height - 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.putText(frame, f'Steering: {steering}', (10, 170),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                cv2.circle(frame, (cx, cy), 10, (0, 255, 255), -1)
                cv2.putText(frame, 'Raise both arms to lock on', (10, 170),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                if arms_raised_history and arms_raised_history[-1]:
                    filled = int((len([x for x in arms_raised_history if x]) /
                                  ARMS_HISTORY_LENGTH) * 100)
                    cv2.putText(frame, f'Lock-on: {filled}%', (10, 200),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        else:
            arms_raised_history = []

    else:
        arms_raised_history = []
        if current_mode == 'TRACKING' and tracking_locked:
            send_steering('STOP')
            send_speed('NORMAL')
            last_steering_command = 'STOP'
            last_distance_command = 'NORMAL'

    # --- MODE DISPLAY ---
    mode_color = (0, 0, 255) if current_mode == 'STOP' else (0, 255, 0)
    cv2.putText(frame, f'Mode: {current_mode}', (10, 90),
                cv2.FONT_HERSHEY_SIMPLEX, 1, mode_color, 2)
    if current_mode == 'STOP':
        cv2.putText(frame, f'Resumes: {previous_mode}', (10, 130),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
    if current_mode == 'TRACKING':
        lock_color = (0, 255, 0) if tracking_locked else (0, 255, 255)
        lock_text = 'LOCKED' if tracking_locked else 'SEARCHING'
        cv2.putText(frame, f'Track: {lock_text}', (10, 130),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, lock_color, 2)

    cv2.imshow('Robot Controller', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

grabber.stop()
engine.stop()
udp_sock.close()
if ser:
    ser.close()
cv2.destroyAllWindows()
