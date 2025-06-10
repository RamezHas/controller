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


def update_joystick_pinch(lmList, center, dead_zone=10, max_radius=60):
    thumb = lmList[4]
    index = lmList[8]

    if is_touching(thumb, index):
        pinch_x = (thumb[1] + index[1]) // 2
        pinch_y = (thumb[2] + index[2]) // 2

        # Calculate offset from center
        dx = pinch_x - center[0]
        dy = pinch_y - center[1]

        distance = math.hypot(dx, dy)
        if distance < dead_zone:
            # Center position (neutral)
            x_val = y_val = 32767
        else:
            # Normalize and apply to joystick range
            # Clamp the distance to max_radius
            radius = min(distance, max_radius)
            magnitude = radius / max_radius

            # Calculate normalized coordinates
            x_norm = dx / distance * magnitude  # Remove angle calculation, use direct ratio
            y_norm = dy / distance * magnitude

            # FIXED: Invert Y axis and correct the mapping
            # For gamepad: right = positive X, down = positive Y
            # For screen: right = positive X, down = positive Y
            # But gamepad expects: center = 32767, max = 65535, min = 0
            x_val = int(32767 - x_norm * 32767)
            y_val = int(32767 - y_norm * 32767)  # INVERTED Y axis

            # Clamp values to valid range
            x_val = max(0, min(65535, x_val))
            y_val = max(0, min(65535, y_val))

        gamepad.left_joystick(x_value=x_val, y_value=-1*y_val)
        gamepad.update()

        # Visual feedback
        cv2.circle(img, center, max_radius, (0, 255, 255), 2)
        cv2.circle(img, center, dead_zone, (128, 128, 128), 1)  # Dead zone circle
        cv2.circle(img, center, 5, (255, 0, 255), -1)
        cv2.line(img, center, (pinch_x, pinch_y), (0, 255, 0), 2)

        # Show current position indicator
        cv2.circle(img, (pinch_x, pinch_y), 8, (0, 255, 0), -1)

    else:
        # Return to center when not pinching
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

    # === LEFT HAND — ANALOG STICK via Pinch and Drag ===
    if left_hand:
        lm = left_hand["lmList"]
        analog_center = (Wcam // 3, Hcam // 2)
        update_joystick_pinch(lm, analog_center)
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
    cv2.imshow("Analog Controller (Pinch to Move)", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()