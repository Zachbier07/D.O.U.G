import cv2
import mediapipe as mp
import time
import os

class HandTracker:
    def __init__(self, camera_index=0):
        self.elapsed = 0
        self.last_updated = time.time()
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
        
        print(f"Attempting to open {device_path}...")
        
        # Use V4L2 backend explicitly
        self.cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
        
        if not self.cap.isOpened():
            raise RuntimeError(f"Failed to open camera {camera_index}")
        
        # Set properties BEFORE checking if they work
        # Use MJPEG codec (most compatible)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        # Give camera time to initialize
        print("Warming up camera...")
        time.sleep(2)
        
        # Discard first few frames (often corrupted)
        for i in range(5):
            ret, _ = self.cap.read()
            if ret:
                print(f"Frame {i+1} captured successfully")
            else:
                print(f"Frame {i+1} failed")
            time.sleep(0.1)
        
        # Test final read
        ret, test_frame = self.cap.read()
        if not ret or test_frame is None:
            self.cap.release()
            raise RuntimeError("Camera opened but cannot read frames")
        
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = int(self.cap.get(cv2.CAP_PROP_FPS))
        backend = self.cap.getBackendName()
        
        print(f"Camera initialized successfully!")
        print(f"  Resolution: {self.width}x{self.height}")
        print(f"  FPS: {fps}")
        print(f"  Backend: {backend}")
    
    def process_frame(self):
        """Capture a frame, detect hand position, return action string or None"""
        ret, frame = self.cap.read()
        
        if not ret or frame is None:
            print("Warning: Failed to read frame")
            return None
        
        # Flip for mirror effect
        frame = cv2.flip(frame, 1)
        
        # Convert BGR to RGB for MediaPipe
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Process with MediaPipe
        results = self.hands.process(rgb_frame)
        
        action = False
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                for lm in hand_landmarks.landmark:
                    cx, cy = int(lm.x * self.width), int(lm.y * self.height)
                    
                    # Draw circle on frame (optional)
                    cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)
                    
                    action = True
        
        self.elapsed = time.time() - self.last_updated
        return action
    
    def toggle_bank(self):
        self.bankOpen = not self.bankOpen
    
    def release(self):
        if self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()
    
    def test(self):
        return "Hello World"