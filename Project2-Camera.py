import cv2
import mediapipe as mp
import serial
import serial.tools.list_ports
import time

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

# hands setup
mp_hands = mp.solutions.hands
mp_draw = mp.solutions.drawing_utils
hands = mp_hands.Hands(
    max_num_hands=1,
    min_detection_confidence=0.7,
    min_tracking_confidence=0.5
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

COOLDOWN = 2  # seconds between mode switches
last_switch_time = 0

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

def get_tracking_command(landmarks, width):
    nose = landmarks[mp_pose.PoseLandmark.NOSE]
    cx = nose.x

    left_dead = 0.4
    right_dead = 0.6

    if cx < left_dead:
        return 'LEFT'
    elif cx > right_dead:
        return 'RIGHT'
    else:
        return 'STRAIGHT'

def send_mode(ser, mode):
    mode_bytes = {
        'RC':         b'\x01',
        'GUIDE_WIRE': b'\x02',
        'TAPE':       b'\x03',
        'TRACKING':   b'\x04',
        'STOP':       b'\x05'
    }
    if ser and mode in mode_bytes:
        ser.write(mode_bytes[mode])
        print(f'Sent mode: {mode}')

def send_steering(ser, command):
    steering_bytes = {
        'LEFT':     b'\x10',
        'RIGHT':    b'\x11',
        'STRAIGHT': b'\x12',
        'STOP':     b'\x13'
    }
    if ser and command in steering_bytes:
        ser.write(steering_bytes[command])

cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

finger_history = []
arms_raised_history = []
HISTORY_LENGTH = 10
ARMS_HISTORY_LENGTH = 15
current_mode = 'RC'
previous_mode = 'RC'
tracking_locked = False

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.flip(frame, 1)
    height, width = frame.shape[:2]

    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    hand_results = hands.process(rgb)
    pose_results = pose.process(rgb)

    fingers = 0

    # --- HAND GESTURE LOGIC ---
    if hand_results.multi_hand_landmarks:
        for hand_landmarks in hand_results.multi_hand_landmarks:
            mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
            fingers = count_fingers(hand_landmarks.landmark)

            # if tracking locked, only process deliberate gestures
            if current_mode == 'TRACKING' and tracking_locked and fingers not in [3, 4, 5]:
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
                        if current_mode in ['GUIDE_WIRE', 'TAPE']:
                            current_mode = 'RC'
                        else:
                            current_mode = 'GUIDE_WIRE'
                        tracking_locked = False
                        send_mode(ser, current_mode)
                        last_switch_time = now
                        finger_history = []

                    elif stable_count == 4:
                        if current_mode != 'STOP':
                            previous_mode = current_mode
                            if current_mode == 'GUIDE_WIRE':
                                current_mode = 'TAPE'
                            elif current_mode == 'TAPE':
                                current_mode = 'GUIDE_WIRE'
                            elif current_mode == 'RC':
                                current_mode = 'TRACKING'
                            elif current_mode == 'TRACKING':
                                current_mode = 'RC'
                            tracking_locked = False
                            send_mode(ser, current_mode)
                            last_switch_time = now
                        finger_history = []

                    elif stable_count == 3:
                        if current_mode == 'STOP':
                            current_mode = previous_mode
                        else:
                            previous_mode = current_mode
                            current_mode = 'STOP'
                        tracking_locked = False
                        send_mode(ser, current_mode)
                        last_switch_time = now
                        finger_history = []
                else:
                    # show cooldown remaining
                    remaining = COOLDOWN - (now - last_switch_time)
                    cv2.putText(frame, f'Cooldown: {remaining:.1f}s', (10, 210),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)

        cv2.putText(frame, f'Fingers: {fingers}', (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    else:
        finger_history = []

    # --- POSE / TRACKING LOGIC ---
    if pose_results.pose_landmarks:
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
                tracking_locked = True
                arms_raised_history = []
                print('Target locked!')

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

                steering = get_tracking_command(landmarks, width)
                send_steering(ser, steering)
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
            send_steering(ser, 'STOP')

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

cap.release()
if ser:
    ser.close()
cv2.destroyAllWindows()
