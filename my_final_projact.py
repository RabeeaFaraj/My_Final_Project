import cv2
from picamera2 import Picamera2
import time
import numpy as np
import sys
from threading import Thread
sys.path.append('/home/aslan/Python')
from custom_servo import Servo
from Telegram_Bot import Telegram 

# Initialize Picamera2 instance
picam2 = Picamera2()

pan = Servo(pin=13)
tilt = Servo(pin=12)

pan_angle = 0
tilt_angle = 0

pan.set_angle(pan_angle)
tilt.set_angle(tilt_angle)

# Set the display width and height
dispW = 400  
dispH = 400 

# Configure the preview settings for Picamera2
picam2.preview_configuration.main.size = (dispW, dispH)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.controls.FrameRate = 60
picam2.preview_configuration.align()
picam2.configure("preview")

# Start the camera
picam2.start()

fps = 0
pos = (10, 30)
font = cv2.FONT_HERSHEY_SIMPLEX
height = 0.5
weight = 1
myColor = (0, 0, 255)

# Global variables for trackbars
hueLow = 25
hueHigh = 65
satLow = 154
satHigh = 255
valLow = 88
valHigh = 241
track = 0
myCount=0

# Trackbar callback functions
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

# Create a window for the trackbars and display
cv2.namedWindow('Camera and Mask')

# Create trackbars inside the 'Camera and Mask' window
cv2.createTrackbar('Hue Low', 'Camera and Mask', hueLow, 360, onTrack1)
cv2.createTrackbar('Hue High', 'Camera and Mask', hueHigh, 360, onTrack2)
cv2.createTrackbar('Sat Low', 'Camera and Mask', satLow, 255, onTrack3)
cv2.createTrackbar('Sat High', 'Camera and Mask', satHigh, 255, onTrack4)
cv2.createTrackbar('Val Low', 'Camera and Mask', valLow, 255, onTrack5)
cv2.createTrackbar('Val High', 'Camera and Mask', valHigh, 255, onTrack6)
cv2.createTrackbar('Train-0 Track-1', 'Camera and Mask', track, 1, onTrack7)

# Function to send Telegram message in a separate thread
class TelegramMessageThread(Thread):
    def __init__(self):
        Thread.__init__(self)

    def run(self):
        Telegram().send_message()

# Function to process the frame
def process_frame(frame):
    global pan_angle, tilt_angle, track, myCount, contours 
    frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lowerBound = np.array([hueLow, satLow, valLow])
    upperBound = np.array([hueHigh, satHigh, valHigh])
    myMask = cv2.inRange(frameHSV, lowerBound, upperBound)

    # Apply morphological operations to remove noise
    kernel = np.ones((3, 3), np.uint8)  # Smaller kernel for faster processing
    myMask = cv2.dilate(myMask, kernel, iterations=1)
    myMask = cv2.erode(myMask, kernel, iterations=1)
    
    contours, _ = cv2.findContours(myMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)

        if track == 1:
            if myCount == 0:
                # Start the Telegram message in a separate thread
                TelegramMessageThread().start()
                myCount = 1

            tilt_error = (y + h / 2) - dispH / 2
            pan_error = (x + w / 2) - dispW / 2

            if abs(tilt_error) > 10:  # Threshold to reduce jitter
                tilt_angle = max(min(tilt_angle + tilt_error / 25, 90), -90)
                if tilt_angle > 4:
                    tilt_angle = 4
                tilt.set_angle(tilt_angle)

            if abs(pan_error) > 10:  # Threshold to reduce jitter
                pan_angle = max(min(pan_angle - pan_error / 25, 90), -90)
                pan.set_angle(pan_angle)
                
    else:
        myCount = 0

    return frame, myMask

# Threading for capturing frames
class FrameCaptureThread(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.frame = None
        self.running = True

    def run(self):
        global picam2
        while self.running:
            self.frame = picam2.capture_array()

    def stop(self):
        self.running = False

# Main loop to capture frames and process them
def main_loop():
    global fps
    frame_capture = FrameCaptureThread()
    frame_capture.start()
    
    last_saved_time = time.time()  
    frame_counter = 1

    while True:
        tStart = time.time()
        frame = frame_capture.frame
        if frame is None:
            continue

        frame = cv2.flip(frame, -1)
        frame, myMask = process_frame(frame)

        # Resize the mask for easier viewing
        myMask = cv2.cvtColor(myMask, cv2.COLOR_GRAY2BGR)  # Convert to BGR for concatenation
        combined_frame = cv2.hconcat([frame, myMask])  # Combine horizontally

        # Draw a green cross in the middle of the frame
        centerX, centerY = dispW // 2, dispH // 2
        cv2.line(combined_frame, (centerX - 10, centerY), (centerX + 10, centerY), (0, 255, 0), 1)
        cv2.line(combined_frame, (centerX, centerY - 10), (centerX, centerY + 10), (0, 255, 0), 1)

        cv2.putText(combined_frame, str(int(fps)) + ' FPS', pos, font, height, myColor, weight)

        # Show the combined frame and mask
        cv2.imshow("Camera and Mask", combined_frame)

        # Save the frame every 5 seconds if tracking
        if track == 1 and contours and frame_counter<4 :
            current_time = time.time()
            if current_time - last_saved_time >= 5:
                frame_name = f"/home/aslan/Python/frame_save/frame_{frame_counter}.jpg"
                centerX, centerY = dispW // 2, dispH // 2
                cv2.line(frame, (centerX - 10, centerY), (centerX + 10, centerY), (0, 255, 0), 1)
                cv2.line(frame, (centerX, centerY - 10), (centerX, centerY + 10), (0, 255, 0), 1)
                cv2.imwrite(frame_name, frame)  # Save the frame
                print(f"Frame saved: {frame_name}")
                last_saved_time = current_time  # Update the time of the last saved frame
                frame_counter += 1

        # Exit on 'Esc' or X press
        if cv2.waitKey(1) == 27 or cv2.getWindowProperty("Camera and Mask", cv2.WND_PROP_VISIBLE) < 1:
            break

        tEnd = time.time()
        loopTime = tEnd - tStart
        fps = 0.9 * fps + 0.1 * (1 / loopTime)

    frame_capture.stop()
    cv2.destroyAllWindows()

# Start the main loop
if __name__ == "__main__":
    main_loop()
