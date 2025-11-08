import cv2
import mediapipe as mp
import time
import os

class HandTracker:
    def __init__(self, camera_index=0):
        
        self.bankOpen = False

        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )
        
        # Verify device exists
        device_path = f"/dev/video{camera_index}"
        if not os.path.exists(device_path):
            raise RuntimeError(f"Camera device {device_path} not found")
        
        # Set camera
        self.cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
        
        if not self.cap.isOpened():
            raise RuntimeError(f"Failed to open camera {camera_index}")
        
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        
        # Give camera time to startup
        print("Starting Camera")
        time.sleep(2)
        
        # Test final read
        ret, test_frame = self.cap.read()
        if not ret or test_frame is None:
            self.cap.release()
            raise RuntimeError("Camera opened but cannot read frames")
        
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
    def process_frame(self):
        ret, frame = self.cap.read()
        
        if not ret or frame is None:
            print("Warning: Failed to read frame")
            return None
        
        # Convert BGR to RGB for MediaPipe
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Process with MediaPipe
        results = self.hands.process(rgb_frame)
        
        if not results.multi_hand_landmarks or self.bankOpen:
            action = False
        else: 
            action = True
                    
        return action
    
    def toggle_bank(self):
        self.bankOpen = not self.bankOpen
    
    def release(self):
        if self.cap is not None:
            self.cap.release()
    
    def test(self):
        return "Hello World"
