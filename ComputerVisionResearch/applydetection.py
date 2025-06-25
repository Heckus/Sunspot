# detect_volleyball_revised.py

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
# MODIFIED: Paths now point to the optimized .tflite models
MODEL_DIR = 'drive/MyDrive/Colab Notebooks/numbers_detection' 
CLASSIFICATION_MODEL_PATH = os.path.join(MODEL_DIR, 'numbers_classifier_multidigit_model.tflite')
BBOX_MODEL_PATH = os.path.join(MODEL_DIR, 'number_detection_model.tflite')

INPUT_VIDEO_PATH = os.path.join(MODEL_DIR, 'video_samples/input/IMG_9088.mp4')
OUTPUT_VIDEO_PATH = os.path.join(MODEL_DIR, 'video_samples/output/volleyball_output_revised.mp4')

# MODIFIED: Tuned thresholds and model selection
YOLO_MODEL = 'yolov8s.pt'  # Switched to a more accurate model
YOLO_CONF_THRESHOLD = 0.3   # Lowered for better recall
NUMBER_CONF_THRESHOLD = 0.5

# NEW: Configuration for performance and tracking
PROCESSING_WIDTH = 1920  # Resize frames to 1080p for faster processing
PROCESSING_HEIGHT = 1080
IOU_THRESHOLD = 0.4  # Threshold for matching objects between frames
MAX_FRAMES_TO_SKIP = 10  # How many frames an object can be missed before being delisted
PLAYER_NUM_HISTORY = 15 # Number of recent number detections to store per player

# =============================================================================
# --- TFLite & Tracking Helper Functions ---
# =============================================================================

def load_tflite_model(model_path):
    """Loads a TFLite model and allocates tensors."""
    interpreter = tf.lite.Interpreter(model_path=model_path)
    interpreter.allocate_tensors()
    return interpreter

def tflite_predict(interpreter, image):
    """Performs inference using a TFLite interpreter."""
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()
    
    interpreter.set_tensor(input_details[0]['index'], image)
    interpreter.invoke()
    return interpreter.get_tensor(output_details[0]['index'])

# MODIFIED: Functions now use the TFLite interpreter
def detect_number_bbox(player_image, bbox_interpreter):
    image = cv2.resize(player_image, (224, 224))
    image = image.astype("float32") / 255.0
    image = np.expand_dims(image, axis=0)
    preds_bbox = tflite_predict(bbox_interpreter, image)[0]
    return tuple(preds_bbox)

def identify_number(number_image, classifier_interpreter):
    if number_image.shape[0] == 0 or number_image.shape[1] == 0:
        return -1, 0.0
    image = cv2.resize(number_image, (224, 224))
    image = image.astype("float32") / 255.0
    image = np.expand_dims(image, axis=0)
    preds = tflite_predict(classifier_interpreter, image)[0]
    
    i = np.argmax(preds)
    confidence = preds[i]
    return (i, confidence) if confidence > NUMBER_CONF_THRESHOLD else (-1, confidence)

# NEW: IOU calculation for tracking
def calculate_iou(boxA, boxB):
    xA = max(boxA[0], boxB[0])
    yA = max(boxA[1], boxB[1])
    xB = min(boxA[2], boxB[2])
    yB = min(boxA[3], boxB[3])
    interArea = max(0, xB - xA) * max(0, yB - yA)
    boxAArea = (boxA[2] - boxA[0]) * (boxA[3] - boxA[1])
    boxBArea = (boxB[2] - boxB[0]) * (boxB[3] - boxB[1])
    return interArea / float(boxAArea + boxBArea - interArea)

# =============================================================================
# --- Main Processing Thread ---
# =============================================================================
def process_frames(frame_queue, result_queue, models):
    """Worker thread function to process frames from the queue."""
    yolo_model = models['yolo']
    bbox_interpreter = models['bbox']
    classifier_interpreter = models['classifier']

    # NEW: Tracking state variables
    next_player_id = 0
    tracked_players = {} # {id: {"bbox": [], "history": deque(), "last_seen": 0}}
    tracked_ball = {} # {id: {"bbox": [], "last_seen": 0}}
    frame_num = 0

    while True:
        frame = frame_queue.get()
        if frame is None: # Sentinel value to stop the thread
            break
        
        frame_num += 1
        
        # --- YOLO Detection ---
        results = yolo_model.predict(frame, conf=YOLO_CONF_THRESHOLD, classes=[0, 32], verbose=False)
        
        current_detections = {"players": [], "ball": []}
        for box in results[0].boxes:
            coords = [int(i) for i in box.xyxy[0]]
            cls_id = int(box.cls[0])
            if cls_id == 0:
                current_detections["players"].append(coords)
            elif cls_id == 32:
                current_detections["ball"].append(coords)
        
        # --- Player Tracking & Number Recognition ---
        matched_player_ids = set()
        for det_box in current_detections["players"]:
            best_iou = 0
            best_id = -1
            for pid, data in tracked_players.items():
                iou = calculate_iou(det_box, data["bbox"])
                if iou > IOU_THRESHOLD and iou > best_iou:
                    best_iou = iou
                    best_id = pid

            if best_id != -1:
                tracked_players[best_id]["bbox"] = det_box
                tracked_players[best_id]["last_seen"] = 0
                matched_player_ids.add(best_id)
            else:
                tracked_players[next_player_id] = {"bbox": det_box, "history": deque(maxlen=PLAYER_NUM_HISTORY), "last_seen": 0}
                matched_player_ids.add(next_player_id)
                next_player_id += 1
        
        # Update tracked players
        for pid, data in list(tracked_players.items()):
            if pid in matched_player_ids:
                x1, y1, x2, y2 = data["bbox"]
                player_image = frame[y1:y2, x1:x2]
                if player_image.size > 0:
                    (startX_rel, startY_rel, endX_rel, endY_rel) = detect_number_bbox(player_image, bbox_interpreter)
                    h, w, _ = player_image.shape
                    num_box = (int(startX_rel*w), int(startY_rel*h), int(endX_rel*w), int(endY_rel*h))
                    number_crop = player_image[num_box[1]:num_box[3], num_box[0]:num_box[2]]
                    
                    number_pred, confidence = identify_number(number_crop, classifier_interpreter)
                    data["history"].append((number_pred, confidence))
            else:
                data["last_seen"] += 1
                if data["last_seen"] > MAX_FRAMES_TO_SKIP:
                    del tracked_players[pid]
        
        # --- Ball Tracking ---
        # (Simplified tracking for the single ball)
        if current_detections["ball"]:
            ball_box = current_detections["ball"][0]
            if not tracked_ball:
                tracked_ball[0] = {"bbox": ball_box, "last_seen": 0}
            else:
                tracked_ball[0]["bbox"] = ball_box
                tracked_ball[0]["last_seen"] = 0
        elif tracked_ball:
            tracked_ball[0]["last_seen"] += 1
            if tracked_ball[0]["last_seen"] > MAX_FRAMES_TO_SKIP:
                del tracked_ball[0]

        # --- Drawing Logic ---
        # Draw players
        for pid, data in tracked_players.items():
            x1, y1, x2, y2 = data["bbox"]
            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
            
            # Get stable number from history
            high_conf_numbers = [num for num, conf in data["history"] if num != -1 and conf > NUMBER_CONF_THRESHOLD + 0.2]
            if high_conf_numbers:
                stable_num = Counter(high_conf_numbers).most_common(1)[0][0]
                label = f"Player {stable_num}"
            else:
                label = "Player ?"

            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 0, 0), 3)

        # Draw ball
        if tracked_ball and 0 in tracked_ball:
            x1, y1, x2, y2 = tracked_ball[0]["bbox"]
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, "Ball", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)
            
        result_queue.put(frame)

# =============================================================================
# --- Main Pipeline ---
# =============================================================================
def main():
    print("[INFO] Loading models...")
    models = {
        'yolo': YOLO(YOLO_MODEL),
        'bbox': load_tflite_model(BBOX_MODEL_PATH),
        'classifier': load_tflite_model(CLASSIFICATION_MODEL_PATH)
    }

    print("[INFO] Opening video file...")
    video = cv2.VideoCapture(INPUT_VIDEO_PATH)
    if not video.isOpened():
        print(f"[ERROR] Could not open video file: {INPUT_VIDEO_PATH}")
        return

    os.makedirs(os.path.dirname(OUTPUT_VIDEO_PATH), exist_ok=True)
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    # MODIFIED: Writer uses the standard processing dimensions
    writer = cv2.VideoWriter(OUTPUT_VIDEO_PATH, fourcc, video.get(cv2.CAP_PROP_FPS), (PROCESSING_WIDTH, PROCESSING_HEIGHT))
    
    # NEW: Threading and queue setup
    frame_queue = queue.Queue(maxsize=8)
    result_queue = queue.Queue()
    
    processing_thread = threading.Thread(target=process_frames, args=(frame_queue, result_queue, models))
    processing_thread.start()

    frame_count = 0
    start_time = time.time()
    
    while True:
        ret, frame = video.read()
        if not ret:
            break
        
        # MODIFIED: Always resize the frame before processing
        resized_frame = cv2.resize(frame, (PROCESSING_WIDTH, PROCESSING_HEIGHT))
        frame_queue.put(resized_frame)
        
        if not result_queue.empty():
            processed_frame = result_queue.get()
            writer.write(processed_frame)
            frame_count += 1

    # --- Cleanup ---
    frame_queue.put(None) # Sentinel to stop the processing thread
    processing_thread.join()

    # Write any remaining frames from the queue
    while not result_queue.empty():
        writer.write(result_queue.get())
        frame_count += 1
    
    end_time = time.time()
    processing_time = end_time - start_time
    fps = frame_count / processing_time if processing_time > 0 else 0

    print(f"\n[INFO] Processing complete.")
    print(f"Total frames processed: {frame_count}")
    print(f"Total time: {processing_time:.2f} seconds")
    print(f"Achieved FPS: {fps:.2f}")

    video.release()
    writer.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()