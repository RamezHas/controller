import vgamepad as vg
import time
import cv2
import HTModule as htm
import numpy as np
import math

# Webcam setup
Wcam, Hcam = 640, 480
cap = cv2.VideoCapture(0)
cap.set(3, Wcam)
cap.set(4, Hcam)

# Detector and gamepad
detector = htm.handDetector(detectionCon=0.75, maxHands=2)
gamepad = vg.VX360Gamepad()

# Track state
right_hand_action = None

# === HELPER FUNCTIONS ===
def is_touching(p1, p2, threshold=30):
    return math.hypot(p1[1] - p2[1], p1[2] - p2[2]) < threshold

def press_button(action):
    if action == "short_pass":
        gamepad.press_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_A)
    elif action == "shoot":
        gamepad.press_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_B)
    elif action == "long_pass":
        gamepad.press_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_X)
    elif action == "change_player":
        gamepad.press_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_Y)
    elif action == "sprint":
        gamepad.left_trigger(value=255)
    gamepad.update()

def release_all():
    gamepad.left_trigger(value=0)
    gamepad.release_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_A)
    gamepad.release_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_B)
    gamepad.release_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_X)
    gamepad.release_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_Y)
    gamepad.update()

def recognize_button_touch(lmList):
    thumb = lmList[4]
    if is_touching(thumb, lmList[8]):
        return "short_pass"
    elif is_touching(thumb, lmList[12]):
        return "shoot"
    elif is_touching(thumb, lmList[16]):
        return "long_pass"
    elif is_touching(thumb, lmList[20]):
        return "change_player"
    return "idle"

def update_analog_if_touching(thumb, index, center, dead_zone=10, max_radius=60):
    if is_touching(thumb, index):
        dx = center[0] - index[1]  # invert x for mirrored webcam
        dy = center[1] - index[2]  # y is inverted to match gamepad up/down

        distance = math.hypot(dx, dy)
        if distance < dead_zone:
            x_val = y_val = 32767
        else:
            angle = math.atan2(dy, dx)
            radius = min(distance, max_radius)
            magnitude = radius / max_radius

            y_norm = math.cos(angle) * magnitude
            x_norm = math.sin(angle) * magnitude

            x_val = int(32767 - x_norm * 32767)
            y_val = int(32767 + y_norm * 32767)


            x_val = max(0, min(65535, x_val))
            y_val = max(0, min(65535, y_val))

        gamepad.left_joystick(x_value=y_val, y_value=x_val)
        gamepad.update()

        cv2.circle(img, center, max_radius, (0, 255, 255), 2)
        cv2.circle(img, center, 5, (255, 0, 255), -1)
        cv2.line(img, center, (index[1], index[2]), (0, 255, 0), 2)
    else:
        gamepad.left_joystick(x_value=32767, y_value=32767)
        gamepad.update()

# === MAIN LOOP ===
print("[INFO] Analog Stick Controller Running. Press 'Q' to quit.")

while True:
    success, img = cap.read()
    if not success:
        break

    img = cv2.flip(img, 1)
    img = detector.findHands(img)
    hands = detector.findPositionMultiple(img)

    left_hand = None
    right_hand = None

    for hand in hands:
        if hand["type"] == "Left":
            left_hand = hand
        elif hand["type"] == "Right":
            right_hand = hand

    # === LEFT HAND — ANALOG STICK from 4 → 8 movement ===
    if left_hand:
        lm = left_hand["lmList"]
        thumb = lm[4]
        index = lm[8]
        analog_center = (Wcam // 3, Hcam // 2)
        update_analog_if_touching(thumb, index, analog_center)
    else:
        gamepad.left_joystick(y_value=32767, x_value=32767)
        gamepad.update()

    # === RIGHT HAND — BUTTONS ===
    if right_hand:
        action = recognize_button_touch(right_hand["lmList"])
        if action != right_hand_action:
            release_all()
            if action != "idle":
                press_button(action)
            right_hand_action = action
    else:
        release_all()
        right_hand_action = None

    # === DISPLAY ===
    cv2.imshow("Analog Controller (Touch to Move)", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

