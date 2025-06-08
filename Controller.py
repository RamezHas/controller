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

# Detector and controller
detector = htm.handDetector(detectionCon=0.75, maxHands=2)
gamepad = vg.VX360Gamepad()

# === GAMEPAD CONTROL ===
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

def release_dpad():
    gamepad.release_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_UP)
    gamepad.release_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_DOWN)
    gamepad.release_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_LEFT)
    gamepad.release_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_RIGHT)
    gamepad.update()

def is_touching(p1, p2, threshold=30):
    return math.hypot(p1[1] - p2[1], p1[2] - p2[2]) < threshold

def detect_dpad_gesture(lmList):
    thumb = lmList[4]
    index = lmList[8]
    middle = lmList[12]
    ring = lmList[16]
    pinky = lmList[20]

    if is_touching(thumb, index):
        return "right"
    elif is_touching(thumb, middle):
        return "up"
    elif is_touching(thumb, ring):
        return "down"
    elif is_touching(thumb, pinky):
        return "left"
    return None

def recognize_button_touch(lmList):
    thumb = lmList[4]
    index = lmList[8]
    middle = lmList[12]
    ring = lmList[16]
    pinky = lmList[20]

    if is_touching(thumb, index):
        return "short_pass"
    elif is_touching(thumb, middle):
        return "shoot"
    elif is_touching(thumb, ring):
        return "long_pass"
    elif is_touching(thumb, pinky):
        return "change_player"
    return "idle"

# === MAIN LOOP ===
print("[INFO] Controller running. Thumb + finger = D-Pad or action. Press 'Q' to quit.")

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

    # === LEFT HAND: D-PAD by Thumb + Finger Touch ===
    if left_hand:
        lm = left_hand["lmList"]
        direction = detect_dpad_gesture(lm)

        release_dpad()  # always release before applying new
        if direction == "up":
            gamepad.press_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_UP)
        elif direction == "down":
            gamepad.press_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_DOWN)
        elif direction == "left":
            gamepad.press_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_LEFT)
        elif direction == "right":
            gamepad.press_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_RIGHT)
        gamepad.update()
    else:
        release_dpad()

    # === RIGHT HAND: Button Touch ===
    if right_hand:
        action = recognize_button_touch(right_hand["lmList"])
        if action in ["short_pass", "shoot", "long_pass", "change_player"]:
            press_button(action)
            time.sleep(0.2)
            release_all()
            cv2.putText(img, f'{action.upper()} TRIGGERED', (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
    else:
        release_all()

    # === DISPLAY ===
    cv2.imshow("PES Hand Controller (Touch D-Pad)", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
