import cv2
import numpy as np
import time
import RPi.GPIO as GPIO

# --- GPIO SETUP ---
TRIGGER_PIN = 18         # Input from ESP32
FPGA_FLAG_OUT = 19       # Output to FPGA

GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIGGER_PIN, GPIO.IN)
GPIO.setup(FPGA_FLAG_OUT, GPIO.OUT)
GPIO.output(FPGA_FLAG_OUT, GPIO.LOW)

# --- Camera Functions ---
def start_camera():
    pipeline = (
        "libcamerasrc ! "
        "video/x-raw,width=640,height=480,framerate=30/1,format=BGR ! "
        "videoconvert ! "
        "appsink"
    )
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print("Cannot open camera")
        return None
    return cap

def stop_camera(cap):
    cap.release()
    print("Camera stopped")

# --- Main Loop ---
NO_FIRE_TIMEOUT = 10  # seconds

while True:
    print("Waiting for ESP32 TSL trigger...")
    while GPIO.input(TRIGGER_PIN) == 0:
        time.sleep(0.05)
    print("Trigger received. Starting camera fire detection...")

    cap = start_camera()
    if cap is None:
        continue

    prev_Y = None
    no_fire_start = None

    while cap:
        ret, frame = cap.read()
        if not ret:
            print("No frame. Restarting camera...")
            break

        frame = cv2.GaussianBlur(frame, (3, 3), 0)
        frame = cv2.convertScaleAbs(frame, alpha=1.3, beta=10)

        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_hsv = np.array([0, 150, 150])
        upper_hsv = np.array([50, 255, 255])
        hsv_mask = cv2.inRange(hsv_frame, lower_hsv, upper_hsv)

        ycbcr_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2YCrCb)
        Y, Cr, Cb = cv2.split(ycbcr_frame)

        if prev_Y is not None:
            flicker = cv2.absdiff(Y, prev_Y)
        else:
            flicker = np.zeros_like(Y)
        prev_Y = Y.copy()

        ycc_mask = (Y > 180) & (Cr > 140) & ((Cr - Cb) > 20) & (flicker > 10)
        ycc_mask = ycc_mask.astype(np.uint8) * 255

        combined_mask = ycc_mask.copy()
        confidence_boost = cv2.bitwise_and(ycc_mask, hsv_mask)
        combined_mask = cv2.addWeighted(combined_mask, 1.0, confidence_boost, 0.5, 0)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        fire_detected = False
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 100:
                fire_detected = True
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                cv2.putText(frame, "FIRE DETECTED", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        GPIO.output(FPGA_FLAG_OUT, GPIO.HIGH if fire_detected else GPIO.LOW)

        #cv2.imshow('Combined Fire Mask', combined_mask)
        #cv2.imshow('Fire Detection', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        if fire_detected:
            no_fire_start = None
        else:
            if no_fire_start is None:
                no_fire_start = time.time()
            elif time.time() - no_fire_start > NO_FIRE_TIMEOUT:
                print("No fire for a while. Going idle.")
                GPIO.output(FPGA_FLAG_OUT, GPIO.LOW)
                stop_camera(cap)
                cap = None
                break

    #cv2.destroyAllWindows()
GPIO.cleanup()
