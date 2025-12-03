## QUTVC intelligent Volleyball Identification System ##


import cv2
import numpy as np
from ultralytics import YOLO

# =============================================================================
# --- COMPONENT 1: MOMENTUM TRACKER (Physics) ---
# =============================================================================
class MomentumTracker:
    def __init__(self, max_history=5, max_ghost_frames=3):
        self.history = [] 
        self.max_history = max_history
        self.max_ghost_frames = max_ghost_frames
        self.ghost_counter = 0 # Tracks how many frames we've predicted without real data
        self.last_dims = (50, 50) # Fallback dimensions

    def update(self, box):
        """Called when a REAL sensor (YOLO/Color) finds the ball."""
        cx = (box[0] + box[2]) / 2
        cy = (box[1] + box[3]) / 2
        self.history.append((cx, cy))
        
        # Update known size
        w = box[2] - box[0]
        h = box[3] - box[1]
        self.last_dims = (w, h)
        
        if len(self.history) > self.max_history:
            self.history.pop(0)
        
        # Reset ghost counter because we found the object
        self.ghost_counter = 0

    def predict(self, frame_shape):
        """
        Attempts to predict location. 
        Returns None if history is insufficient OR if we've exceeded max_ghost_frames.
        """
        # 1. Safety Check: Don't hallucinate forever
        if self.ghost_counter >= self.max_ghost_frames:
            return None

        if len(self.history) < 2:
            return None
        
        # 2. Physics Calculation (Linear Extrapolation)
        (x1, y1) = self.history[-2]
        (x2, y2) = self.history[-1]
        dx = x2 - x1
        dy = y2 - y1
        
        pred_x = int(x2 + dx)
        pred_y = int(y2 + dy)
        
        # 3. Construct Box
        w_half = self.last_dims[0] // 2
        h_half = self.last_dims[1] // 2
        h_img, w_img = frame_shape[:2]
        
        p_x1 = max(0, pred_x - w_half)
        p_y1 = max(0, pred_y - h_half)
        p_x2 = min(w_img, pred_x + w_half)
        p_y2 = min(h_img, pred_y + h_half)
        
        # 4. Increment ghost counter (we used a prediction, not a measurement)
        self.ghost_counter += 1
        
        # Update history with the prediction so the NEXT prediction continues the arc
        self.history.append((pred_x, pred_y))
        if len(self.history) > self.max_history:
            self.history.pop(0)

        return (p_x1, p_y1, p_x2, p_y2)

# =============================================================================
# --- COMPONENT 2: COLOR DETECTOR (Smart Fallback) ---
# =============================================================================
class ColorBlobDetector:
    def __init__(self):
        self.backSub = cv2.createBackgroundSubtractorMOG2(history=500, varThreshold=50, detectShadows=False)
        self.lower_yellow = np.array([18, 100, 100])
        self.upper_yellow = np.array([40, 255, 255])
        self.warmup = 50
        self.count = 0
        self.box_padding = 0.2

    def detect(self, frame):
        self.count += 1
        fgMask = self.backSub.apply(frame)
        if self.count < self.warmup: return None

        h_img, w_img = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.bitwise_and(fgMask, cv2.inRange(hsv, self.lower_yellow, self.upper_yellow))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((11,11), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best_box = None
        max_area = 0

        for c in contours:
            area = cv2.contourArea(c)
            if area < 100 or area > 20000: continue
            
            # Roundness check
            ((cx, cy), radius) = cv2.minEnclosingCircle(c)
            if area / (np.pi * radius**2) < 0.60: continue

            # If passed, calculate square box with padding
            x, y, w, h = cv2.boundingRect(c)
            center_x, center_y = x + w//2, y + h//2
            side = max(w, h) * (1 + self.box_padding)
            r = int(side / 2)
            
            if area > max_area:
                max_area = area
                best_box = (
                    max(0, center_x - r),
                    max(0, center_y - r),
                    min(w_img, center_x + r),
                    min(h_img, center_y + r)
                )
        return best_box

# =============================================================================
# --- MAIN PACKAGE: IVIS DETECTOR ---
# =============================================================================
class IVISDetector:
    def __init__(self, model_path, conf=0.5):
        print(f"[IVIS] Initializing. Loading YOLO from {model_path}...")
        try:
            self.model = YOLO(model_path)
        except Exception as e:
            print(f"[IVIS] Error loading YOLO: {e}")
            self.model = None
            
        self.conf = conf
        self.color_det = ColorBlobDetector()
        self.tracker = MomentumTracker(max_ghost_frames=3) # Stops after 3 frames of no detection
        
        # Colors for visualization (BGR)
        self.colors = {
            'yolo': (255, 0, 0),    # Blue
            'color': (0, 255, 255), # Yellow
            'physics': (255, 0, 255)# Purple
        }

    def predict(self, frame):
        """
        Main pipeline: YOLO -> Color -> Physics
        Returns: dict {'box': (x1,y1,x2,y2), 'source': str, 'center': (x,y)} or None
        """
        box = None
        source = None

        # 1. Try YOLO
        if self.model:
            results = self.model.predict(frame, conf=self.conf, verbose=False, classes=[0])
            if results[0].boxes:
                # Take highest confidence box
                b = results[0].boxes[0]
                coords = [int(i) for i in b.xyxy[0]]
                box = tuple(coords)
                source = 'yolo'

        # 2. Try Color (if YOLO failed)
        if box is None:
            box = self.color_det.detect(frame)
            if box: source = 'color'

        # 3. Physics Update logic
        if box:
            # We have a real detection, update the tracker
            self.tracker.update(box)
        else:
            # No detection, try physics prediction
            box = self.tracker.predict(frame.shape)
            if box: source = 'physics'

        if box:
            cx = (box[0] + box[2]) // 2
            cy = (box[1] + box[3]) // 2
            return {'box': box, 'source': source, 'center': (cx, cy)}
        
        return None

    def visualize(self, frame, result, mode=1):
        """
        Draws the result on the frame based on the selected mode.
        Mode 1: Box + Label
        Mode 2: Box Only
        Mode 3: Circle
        Mode 4: None (Data only)
        """
        if result is None or mode == 4:
            return frame

        x1, y1, x2, y2 = result['box']
        color = self.colors.get(result['source'], (0, 255, 0))
        
        if mode == 1: # Box + Label
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            label = f"IVIS:{result['source'].upper()}"
            (w, h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            cv2.rectangle(frame, (x1, y1 - 20), (x1 + w, y1), color, -1)
            cv2.putText(frame, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
        elif mode == 2: # Box Only
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            
        elif mode == 3: # Circle
            cx, cy = result['center']
            radius = max(x2-x1, y2-y1) // 2
            cv2.circle(frame, (cx, cy), radius, color, 2)
            cv2.circle(frame, (cx, cy), 3, (0, 0, 255), -1)

        return frame