import cv2
from picamera2 import Picamera2
import smbus
import RPi.GPIO as GPIO
import time
import numpy as np
import sys
from threading import Thread
import threading
sys.path.append('/home/aslan/Python')
from custom_servo import Servo
from Telegram_Bot import Telegram
import LCD1602  # Import LCD1602 module for LCD integration
import os

# Initialize Picamera2 instance for camera handling
picam2 = Picamera2()

# Initialize Servo motors for pan and tilt functionality
pan = Servo(pin=13)
tilt = Servo(pin=12)

# Initial angles for pan and tilt servos
pan_angle = 0
tilt_angle = 0

# Set initial angles
pan.set_angle(pan_angle)
tilt.set_angle(tilt_angle)

# Initialize LCD1602 display with I2C address 0x27 and enable backlight
LCD1602.init(0x27, 1)

# Set GPIO mode to BCM
GPIO.setmode(GPIO.BCM)

# Define GPIO pins for relays and set them as outputs
RELAY_PIN = 27
RELAY_PIN2 = 26
GPIO.setup(RELAY_PIN, GPIO.OUT)
GPIO.setup(RELAY_PIN2, GPIO.OUT)

# Initialize I2C bus for ADC
bus = smbus.SMBus(1)  # Bus 1 indicates /dev/i2c-1
ADC_ADDRESS = 0x4b  # I2C address of the ADS7830 ADC

# Display resolution and preview configuration for camera
dispW = 400
dispH = 400
picam2.preview_configuration.main.size = (dispW, dispH)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.controls.FrameRate = 60
picam2.preview_configuration.align()
picam2.configure("preview")

# Start the camera
picam2.start()

# Initialize FPS and text properties for display
fps = 0
pos = (10, 30)
font = cv2.FONT_HERSHEY_SIMPLEX
height = 0.5
weight = 1
myColor = (0, 0, 255)

# HSV filter initial values for object tracking
hueLow = 25
hueHigh = 65
satLow = 154
satHigh = 255
valLow = 88
valHigh = 241
track = 0
myCount = 0
lcd = 3
last_speech_time = 0
last_lcd_update = 0

# Trackbar callback functions for adjusting HSV values
def onTrack1(val):
    global hueLow
    hueLow = val

def onTrack2(val):
    global hueHigh
    hueHigh = val

def onTrack3(val):
    global satLow
    satLow = val

def onTrack4(val):
    global satHigh
    satHigh = val

def onTrack5(val):
    global valLow
    valLow = val

def onTrack6(val):
    global valHigh
    valHigh = val

def onTrack7(val):
    global track
    track = val

# Create a window for the trackbars and display adjustments
cv2.namedWindow('Camera and Mask')

# Create trackbars inside the 'Camera and Mask' window
cv2.createTrackbar('Hue Low', 'Camera and Mask', hueLow, 360, onTrack1)
cv2.createTrackbar('Hue High', 'Camera and Mask', hueHigh, 360, onTrack2)
cv2.createTrackbar('Sat Low', 'Camera and Mask', satLow, 255, onTrack3)
cv2.createTrackbar('Sat High', 'Camera and Mask', satHigh, 255, onTrack4)
cv2.createTrackbar('Val Low', 'Camera and Mask', valLow, 255, onTrack5)
cv2.createTrackbar('Val High', 'Camera and Mask', valHigh, 255, onTrack6)
cv2.createTrackbar('Train-0 Track-1', 'Camera and Mask', track, 1, onTrack7)

# Function to make the Pi speak phrases in a separate thread
def say_target_in_thread(status):
    def say_target():
        if status == 1:
            os.system("espeak 'Tracking target'")
        if status == 2:
            os.system("espeak 'target destroyed'")
        if status == 3:
            os.system("espeak 'searching target'")

    # Start the speech thread
    thread = threading.Thread(target=say_target)
    thread.start()

# Function to update the LCD with a message in a separate thread
def update_lcd_in_thread(message):
    def update_lcd():
        LCD1602.clear()  # Clear previous message
        LCD1602.write(0, 0, message)

    # Start the thread
    thread = threading.Thread(target=update_lcd)
    thread.start()

# Function to send Telegram message in a separate thread
class TelegramMessageThread(Thread):
    def _init_(self):
        Thread._init_(self)

    def run(self):
        Telegram().send_message()

# Function to process frames for object detection
def process_frame(frame):
    global pan_angle, tilt_angle, track, myCount, contours, lcd, last_speech_time, last_lcd_update
    frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lowerBound = np.array([hueLow, satLow, valLow])
    upperBound = np.array([hueHigh, satHigh, valHigh])
    myMask = cv2.inRange(frameHSV, lowerBound, upperBound)

    # Remove noise using morphological operations
    kernel = np.ones((3, 3), np.uint8)
    myMask = cv2.dilate(myMask, kernel, iterations=1)
    myMask = cv2.erode(myMask, kernel, iterations=1)

    # Find contours in the mask
    contours, _ = cv2.findContours(myMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)

        if track == 1:
            GPIO.output(RELAY_PIN, GPIO.LOW)
            if myCount == 0:
                TelegramMessageThread().start()
                myCount = 1

            # Calculate pan and tilt errors for servo control
            tilt_error = (y + h / 2) - dispH / 2
            pan_error = (x + w / 2) - dispW / 2

            # Adjust tilt and pan angles with limits
            if abs(tilt_error) > 10:
                tilt_angle = max(min(tilt_angle + tilt_error / 25, 4), -90)
                tilt.set_angle(tilt_angle)

            if abs(pan_error) > 10:
                pan_angle = max(min(pan_angle - pan_error / 25, 90), -90)
                pan.set_angle(pan_angle)

            # Update LCD and speech based on tracking conditions
            if abs(pan_error) < 10 and abs(tilt_error) < 10 and lcd != 1:
                current_time = time.time()
                if current_time - last_lcd_update > 2:
                    update_lcd_in_thread("Target destroyed")
                    say_target_in_thread(2)
                    last_lcd_update = current_time
                    last_speech_time = current_time
                    lcd = 1

            if abs(pan_error) > 10 and abs(tilt_error) > 10 and lcd != 0:
                current_time = time.time()
                if current_time - last_lcd_update > 2:
                    update_lcd_in_thread("Tracking target")
                    say_target_in_thread(1)
                    last_lcd_update = current_time
                    last_speech_time = current_time
                    lcd = 0

        if track == 0:
            LCD1602.clear()
            GPIO.output(RELAY_PIN, GPIO.HIGH)

    else:
        if track == 1:
            GPIO.output(RELAY_PIN, GPIO.HIGH)
            if lcd == 3 or lcd==1 or lcd==0:
                current_time = time.time()
                if current_time - last_lcd_update > 5:
                    update_lcd_in_thread("Searching target")
                    say_target_in_thread(3)
                    last_lcd_update = current_time
                    last_speech_time = current_time

        myCount = 0

    return frame, myMask

# Threading class to capture frames continuously
class FrameCaptureThread(Thread):
    def _init_(self):
        Thread._init_(self)
        self.frame = None
        self.running = True

    def run(self):
        global picam2
        while self.running:
            self.frame = picam2.capture_array()

    def stop(self):
        self.running = False

# Function to read ADC value from a specified channel
def read_adc(channel):
    if channel < 0 or channel > 7:
        return -1
    command = 0x8b | (channel << 4)
    bus.write_byte(ADC_ADDRESS, command)
    adc_value = bus.read_byte(ADC_ADDRESS)
    return adc_value

# Main loop for capturing and processing frames
def main_loop():
    global fps
    frame_capture = FrameCaptureThread()
    frame_capture.start()

    last_saved_time = time.time()
    frame_counter = 1
    GPIO.output(RELAY_PIN, GPIO.HIGH)

    while True:
        tStart = time.time()
        frame = frame_capture.frame
        light_level = read_adc(0)

        if light_level < 110:
            GPIO.output(RELAY_PIN2, GPIO.HIGH)
        else:
            GPIO.output(RELAY_PIN2, GPIO.LOW)

        if frame is None:
            continue

        frame = cv2.flip(frame, -1)
        frame, myMask = process_frame(frame)

        # Resize and combine frame and mask for display
        myMask = cv2.cvtColor(myMask, cv2.COLOR_GRAY2BGR)
        combined_frame = cv2.hconcat([frame, myMask])

        # Draw a cross at the center of the frame
        centerX, centerY = dispW // 2, dispH // 2
        cv2.line(combined_frame, (centerX - 10, centerY), (centerX + 10, centerY), (0, 255, 0), 1)
        cv2.line(combined_frame, (centerX, centerY - 10), (centerX, centerY + 10), (0, 255, 0), 1)

        cv2.putText(combined_frame, str(int(fps)) + ' FPS', pos, font, height, myColor, weight)

        cv2.imshow("Camera and Mask", combined_frame)

        # Save frames every 5 seconds if tracking
        if track == 1 and contours and frame_counter < 4:
            current_time = time.time()
            if current_time - last_saved_time >= 5:
                frame_name = f"/home/aslan/Python/frame_save/frame_{frame_counter}.jpg"
                cv2.imwrite(frame_name, frame)
                print(f"Frame saved: {frame_name}")
                last_saved_time = current_time
                frame_counter += 1

        if cv2.waitKey(1) == 27 or cv2.getWindowProperty("Camera and Mask", cv2.WND_PROP_VISIBLE) < 1:
            GPIO.output(RELAY_PIN, GPIO.HIGH)
            GPIO.output(RELAY_PIN2, GPIO.LOW)
            LCD1602.clear()
            break

        tEnd = time.time()
        loopTime = tEnd - tStart
        fps = 0.9 * fps + 0.1 * (1 / loopTime)

    frame_capture.stop()
    cv2.destroyAllWindows()

# Start the main loop when the script is executed
if _name_ == "_main_":
    main_loop()