# detect_volleyball_final.py

# =============================================================================
# --- Imports ---
# =============================================================================
import os
import cv2
import numpy as np
import tensorflow as tf
from ultralytics import YOLO
from collections import deque, Counter
import time
import threading
import queue

# =============================================================================
# --- Configuration ---
# =============================================================================
MODEL_DIR = 'drive/MyDrive/Colab Notebooks/numbers_detection'
CLASSIFICATION_MODEL_PATH = os.path.join(MODEL_DIR, 'numbers_classifier_multidigit_model.tflite')
BBOX_MODEL_PATH = os.path.join(MODEL_DIR, 'number_detection_model.tflite')

INPUT_VIDEO_PATH = os.path.join(MODEL_DIR, 'video_samples/input/IMG_9086.mp4')
OUTPUT_VIDEO_PATH = os.path.join(MODEL_DIR, 'video_samples/output/volleyball_output_final.mp4')

YOLO_MODEL = 'yolov8m.pt'
YOLO_CONF_THRESHOLD = 0.3
NUMBER_CONF_THRESHOLD = 0.5

PROCESSING_WIDTH = 1280
PROCESSING_HEIGHT = 720
IOU_THRESHOLD = 0.4
BALL_IOU_THRESHOLD = 0.1
MAX_FRAMES_TO_SKIP = 10
BALL_MAX_FRAMES_TO_SKIP = 1

PLAYER_NUM_HISTORY = 15

# --- Boolean toggles ---
DETECT_PLAYERS = False
DETECT_BALL = True
RECOGNIZE_PLAYER_NUMBERS = False

# --- Ball Tracking Configuration ---
USE_KALMAN_FILTER = True
BALL_TRAIL_LENGTH = 16

# --- HSV Color Range (UPDATED with your calibrated values) ---
LOWER_BALL_COLOR = np.array([18, 152, 71])
UPPER_BALL_COLOR = np.array([23, 255, 207])

# --- Contour Detection Settings ---
MIN_BALL_AREA = 100
MAX_BALL_AREA = 8000
MOTION_COLOR_MATCH_RATIO = 0.3

# =============================================================================
# --- Kalman Filter Class (Tuned for Motion) ---
# =============================================================================
class KalmanFilter:
    def __init__(self):
        self.kalman = cv2.KalmanFilter(6, 4)
        self.kalman.measurementMatrix = np.array([[1, 0, 0, 0, 0, 0],[0, 1, 0, 0, 0, 0],[0, 0, 0, 0, 1, 0],[0, 0, 0, 0, 0, 1]], np.float32)
        self.kalman.transitionMatrix = np.array([[1, 0, 1, 0, 0, 0],[0, 1, 0, 1, 0, 0],[0, 0, 1, 0, 0, 0],[0, 0, 0, 1, 0, 0],[0, 0, 0, 0, 1, 0],[0, 0, 0, 0, 0, 1]], np.float32)
        self.kalman.processNoiseCov = np.eye(6, dtype=np.float32) * 0.1
        self.kalman.measurementNoiseCov = np.eye(4, dtype=np.float32) * 0.05
    def predict(self):
        pred_state = self.kalman.predict()
        x, y, w, h = int(pred_state[0][0]), int(pred_state[1][0]), int(pred_state[4][0]), int(pred_state[5][0])
        return (x - w // 2, y - h // 2, x + w // 2, y + h // 2)
    def update(self, bbox):
        x1, y1, x2, y2 = bbox
        w, h = x2 - x1, y2 - y1; x, y = x1 + w // 2, y1 + h // 2
        measurement = np.array([x, y, w, h], dtype=np.float32)
        self.kalman.correct(measurement)
    def initialize(self, bbox):
        x1, y1, x2, y2 = bbox
        w, h = x2 - x1, y2 - y1; x, y = x1 + w // 2, y1 + h // 2
        self.kalman.statePost = np.array([x, y, 0, 0, w, h], dtype=np.float32)

# =============================================================================
# --- Helper Functions ---
# =============================================================================
def load_tflite_model(model_path):
    interpreter = tf.lite.Interpreter(model_path=model_path)
    interpreter.allocate_tensors()
    return interpreter
def tflite_predict(interpreter, image):
    input_details, output_details = interpreter.get_input_details(), interpreter.get_output_details()
    interpreter.set_tensor(input_details[0]['index'], image)
    interpreter.invoke()
    return interpreter.get_tensor(output_details[0]['index'])
def detect_number_bbox(player_image, bbox_interpreter):
    image = cv2.resize(player_image, (224, 224)); image = image.astype("float32") / 255.0; image = np.expand_dims(image, axis=0)
    return tuple(tflite_predict(bbox_interpreter, image)[0])
def identify_number(number_image, classifier_interpreter):
    if number_image.size == 0: return -1, 0.0
    image = cv2.resize(number_image, (224, 224)); image = image.astype("float32") / 255.0; image = np.expand_dims(image, axis=0)
    preds = tflite_predict(classifier_interpreter, image)[0]
    i = np.argmax(preds); confidence = preds[i]
    return (i, confidence) if confidence > NUMBER_CONF_THRESHOLD else (-1, confidence)
def calculate_iou(boxA, boxB):
    if not boxA or not boxB: return 0.0
    xA, yA, xB, yB = max(boxA[0], boxB[0]), max(boxA[1], boxB[1]), min(boxA[2], boxB[2]), min(boxA[3], boxB[3])
    interArea = max(0, xB - xA) * max(0, yB - yA)
    boxAArea, boxBArea = (boxA[2] - boxA[0]) * (boxA[3] - boxA[1]), (boxB[2] - boxB[0]) * (boxB[3] - boxB[1])
    return interArea / float(boxAArea + boxBArea - interArea) if (boxAArea + boxBArea - interArea) > 0 else 0.0

def detect_ball_from_contours_motion_first(current_frame, prev_frame, lower_hsv, upper_hsv, min_area, max_area):
    if prev_frame is None: return []

    diff = cv2.absdiff(prev_frame, current_frame)
    gray_diff = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
    _, motion_mask = cv2.threshold(gray_diff, 25, 255, cv2.THRESH_BINARY)
    motion_mask = cv2.dilate(motion_mask, np.ones((5,5),np.uint8), iterations=3)
    motion_mask = cv2.erode(motion_mask, np.ones((5,5),np.uint8), iterations=1)

    contours, _ = cv2.findContours(motion_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    hsv_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)
    detections = []
    
    color_mask = cv2.inRange(hsv_frame, lower_hsv, upper_hsv)

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if min_area < area < max_area:
            x, y, w, h = cv2.boundingRect(cnt)
            roi_mask = color_mask[y:y+h, x:x+w]
            color_match_ratio = cv2.countNonZero(roi_mask) / ((w * h) + 1e-6)
            if color_match_ratio > MOTION_COLOR_MATCH_RATIO:
                detections.append([x, y, x + w, y + h])
            
    return detections

# =============================================================================
# --- Main Processing Thread ---
# =============================================================================
def process_frames(frame_queue, result_queue, models):
    yolo_model, bbox_interpreter, classifier_interpreter = models['yolo'], models['bbox'], models['classifier']
    next_player_id, tracked_players = 0, {}
    ball_kf = KalmanFilter() if USE_KALMAN_FILTER else None
    ball_trail, frames_since_ball_seen, ball_initialized = deque(maxlen=BALL_TRAIL_LENGTH), 0, False
    prev_frame = None

    yolo_classes_to_detect = [cls for flag, cls in [(DETECT_PLAYERS, 0), (DETECT_BALL, 32)] if flag]

    while True:
        frame = frame_queue.get()
        if frame is None: break

        yolo_player_detections, yolo_ball_detections = [], []
        if yolo_classes_to_detect:
            results = yolo_model.predict(frame, conf=YOLO_CONF_THRESHOLD, classes=yolo_classes_to_detect, verbose=False)
            for box in results[0].boxes:
                coords, cls_id = [int(i) for i in box.xyxy[0]], int(box.cls[0])
                if cls_id == 0: yolo_player_detections.append(coords)
                elif cls_id == 32: yolo_ball_detections.append(coords)
        
        if DETECT_PLAYERS:
            matched_player_ids = set()
            for det_box in yolo_player_detections:
                best_iou, best_id = 0, -1
                for pid, data in tracked_players.items():
                    iou = calculate_iou(det_box, data["bbox"])
                    if iou > IOU_THRESHOLD and iou > best_iou:
                        best_iou, best_id = iou, pid
                if best_id != -1:
                    tracked_players[best_id].update({"bbox": det_box, "last_seen": 0})
                    matched_player_ids.add(best_id)
                else:
                    tracked_players[next_player_id] = {"bbox": det_box, "history": deque(maxlen=PLAYER_NUM_HISTORY), "last_seen": 0}
                    matched_player_ids.add(next_player_id)
                    next_player_id += 1
            for pid, data in list(tracked_players.items()):
                if pid in matched_player_ids:
                    if RECOGNIZE_PLAYER_NUMBERS:
                        x1, y1, x2, y2 = data["bbox"]
                        player_image = frame[y1:y2, x1:x2]
                        if player_image.size > 0:
                            (startX_rel, startY_rel, endX_rel, endY_rel) = detect_number_bbox(player_image, bbox_interpreter)
                            h, w, _ = player_image.shape
                            num_box = (int(startX_rel*w), int(startY_rel*h), int(endX_rel*w), int(endY_rel*h))
                            number_crop = player_image[num_box[1]:num_box[3], num_box[0]:num_box[2]]
                            number_pred, confidence = identify_number(number_crop, classifier_interpreter)
                            data["history"].append(number_pred)
                else:
                    data["last_seen"] += 1
                    if data["last_seen"] > MAX_FRAMES_TO_SKIP: del tracked_players[pid]
        
        if DETECT_BALL:
            contour_detections = detect_ball_from_contours_motion_first(frame, prev_frame, LOWER_BALL_COLOR, UPPER_BALL_COLOR, MIN_BALL_AREA, MAX_BALL_AREA)
            all_candidates = yolo_ball_detections + contour_detections
            best_ball_bbox, predicted_bbox = None, None

            if ball_initialized and ball_kf:
                predicted_bbox = ball_kf.predict()
                best_iou, best_candidate = -1, None
                for bbox in all_candidates:
                    iou = calculate_iou(predicted_bbox, bbox)
                    if iou > best_iou:
                        best_iou, best_candidate = iou, bbox
                if best_iou > BALL_IOU_THRESHOLD: best_ball_bbox = best_candidate
            elif all_candidates:
                best_ball_bbox = yolo_ball_detections[0] if yolo_ball_detections else all_candidates[0]

            if best_ball_bbox:
                if ball_kf:
                    if not ball_initialized: ball_kf.initialize(best_ball_bbox); ball_initialized = True
                    else: ball_kf.update(best_ball_bbox)
                frames_since_ball_seen = 0
            else:
                frames_since_ball_seen += 1
                if frames_since_ball_seen > BALL_MAX_FRAMES_TO_SKIP: ball_initialized = False

        if DETECT_PLAYERS:
            for pid, data in tracked_players.items():
                x1, y1, x2, y2 = data["bbox"]
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                label = f"PlayerID: {pid}"
                if RECOGNIZE_PLAYER_NUMBERS and data.get("history"):
                    high_conf_numbers = [num for num in data["history"] if num != -1]
                    if high_conf_numbers:
                        stable_num = Counter(high_conf_numbers).most_common(1)[0][0]
                        label = f"Player {stable_num}"
                    else: label = "Player ?"
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 0, 0), 3)

        if DETECT_BALL:
            final_ball_box = None
            if best_ball_bbox: final_ball_box = best_ball_bbox
            elif ball_initialized and ball_kf and frames_since_ball_seen < BALL_MAX_FRAMES_TO_SKIP:
                final_ball_box = predicted_bbox if predicted_bbox else ball_kf.predict()

            if final_ball_box:
                x1, y1, x2, y2 = final_ball_box
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 3)
                cv2.putText(frame, "Ball", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 3)
                ball_trail.append(((x1 + x2) // 2, (y1 + y2) // 2))

            for i in range(1, len(ball_trail)):
                if ball_trail[i-1] and ball_trail[i]:
                    thickness = int(np.sqrt(BALL_TRAIL_LENGTH / float(i + 1)) * 2.5)
                    cv2.line(frame, ball_trail[i - 1], ball_trail[i], (0, 165, 255), thickness)
        
        prev_frame = frame.copy()
        result_queue.put(frame)

# =============================================================================
# --- Main Pipeline ---
# =============================================================================
def main():
    global RECOGNIZE_PLAYER_NUMBERS, DETECT_PLAYERS
    if RECOGNIZE_PLAYER_NUMBERS and not DETECT_PLAYERS:
        print("[WARNING] Number recognition disabled because player detection is off.")
        RECOGNIZE_PLAYER_NUMBERS = False

    print("[INFO] Loading models...")
    models = {'yolo': YOLO(YOLO_MODEL), 'bbox': load_tflite_model(BBOX_MODEL_PATH), 'classifier': load_tflite_model(CLASSIFICATION_MODEL_PATH)}

    print(f"[INFO] Opening video file: {INPUT_VIDEO_PATH}")
    video = cv2.VideoCapture(INPUT_VIDEO_PATH)
    if not video.isOpened():
        print(f"[ERROR] Could not open video file: {INPUT_VIDEO_PATH}"); return

    os.makedirs(os.path.dirname(OUTPUT_VIDEO_PATH), exist_ok=True)
    writer = cv2.VideoWriter(OUTPUT_VIDEO_PATH, cv2.VideoWriter_fourcc(*'mp4v'), video.get(cv2.CAP_PROP_FPS), (PROCESSING_WIDTH, PROCESSING_HEIGHT))
    
    frame_queue, result_queue = queue.Queue(maxsize=8), queue.Queue()
    
    print("[INFO] Starting processing thread...")
    processing_thread = threading.Thread(target=process_frames, args=(frame_queue, result_queue, models))
    processing_thread.start()

    frame_count, start_time = 0, time.time()
    
    while True:
        ret, frame = video.read()
        if not ret: break
        
        frame_queue.put(cv2.resize(frame, (PROCESSING_WIDTH, PROCESSING_HEIGHT)))
        
        if not result_queue.empty():
            processed_frame = result_queue.get()
            writer.write(processed_frame)
            frame_count += 1
            if frame_count > 0 and frame_count % 50 == 0:
                print(f"[INFO] Processed {frame_count} frames...")

    print("[INFO] Cleaning up...")
    frame_queue.put(None)
    processing_thread.join()

    while not result_queue.empty():
        writer.write(result_queue.get())
        frame_count += 1
    
    end_time = time.time()
    processing_time = end_time - start_time
    fps = frame_count / processing_time if processing_time > 0 else 0

    print(f"\n[INFO] Processing complete.")
    print(f"Total frames processed: {frame_count}, Total time: {processing_time:.2f}s, FPS: {fps:.2f}")

    video.release()
    writer.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()