import cv2
import mediapipe as mp
import serial
import serial.tools.list_ports

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

mp_hands = mp.solutions.hands
mp_draw = mp.solutions.drawing_utils
hands = mp_hands.Hands(
    max_num_hands=1,
    min_detection_confidence=0.7,
    min_tracking_confidence=0.5
)

FINGER_TIPS = [8, 12, 16, 20]  
FINGER_BASES = [6, 10, 14, 18] 
THUMB_TIP = 4
THUMB_BASE = 2

def count_fingers(landmarks):
    count = 0

    for tip, base in zip(FINGER_TIPS, FINGER_BASES):
        if landmarks[tip].y < landmarks[base].y:
            count += 1

    if landmarks[THUMB_TIP].x < landmarks[THUMB_BASE].x:
        count += 1
    
    return count

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

# use webcam for now, swap to ESP32 stream later
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

finger_history = []
HISTORY_LENGTH = 10
current_mode = 'RC'
previous_mode = 'RC'

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.flip(frame, 1)
    height, width = frame.shape[:2]

    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(rgb)
    
    fingers = 0
    
    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
            
            fingers = count_fingers(hand_landmarks.landmark)

            finger_history.append(fingers)
            if len(finger_history) > HISTORY_LENGTH:
                finger_history.pop(0)
            
            if len(finger_history) == HISTORY_LENGTH and len(set(finger_history)) == 1:
                stable_count = finger_history[0]
                
                if stable_count == 5:
                    previous_mode = current_mode
                    if current_mode in ['GUIDE_WIRE', 'TAPE']:
                        current_mode = 'RC'
                    else:
                        current_mode = 'GUIDE_WIRE'
                    send_mode(ser, current_mode)
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
                        send_mode(ser, current_mode)
                    finger_history = []
                
                elif stable_count == 3:
                    if current_mode == 'STOP':
                        current_mode = previous_mode
                    else:
                        previous_mode = current_mode
                        current_mode = 'STOP'
                    send_mode(ser, current_mode)
                    finger_history = []
            
            cv2.putText(frame, f'Fingers: {fingers}', (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    else:
        finger_history = []
    
    # mode display
    mode_color = (0, 0, 255) if current_mode == 'STOP' else (0, 255, 0)
    cv2.putText(frame, f'Mode: {current_mode}', (10, 90),
                cv2.FONT_HERSHEY_SIMPLEX, 1, mode_color, 2)
    if current_mode == 'STOP':
        cv2.putText(frame, f'Resumes: {previous_mode}', (10, 130),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
    
    cv2.imshow('Robot Controller', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
if ser:
    ser.close()
cv2.destroyAllWindows()