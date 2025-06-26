# fused_applydetection.py

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
import math

# =============================================================================
# --- Configuration ---
# =============================================================================
# MODIFIED: Paths now point to the optimized .tflite models
MODEL_DIR = 'drive/MyDrive/Colab Notebooks/numbers_detection'
CLASSIFICATION_MODEL_PATH = os.path.join(MODEL_DIR, 'numbers_classifier_multidigit_model.tflite')
BBOX_MODEL_PATH = os.path.join(MODEL_DIR, 'number_detection_model.tflite')

# NEW: Path to the ball detection model
BALL_MODEL_PATH = 'ball-net/model/model.h5'
BALL_MODEL_JSON_PATH = 'ball-net/model/model.json'


INPUT_VIDEO_PATH = os.path.join(MODEL_DIR, 'video_samples/input/IMG_9088.mp4')
OUTPUT_VIDEO_PATH = os.path.join(MODEL_DIR, 'video_samples/output/volleyball_output_fused.mp4')

# MODIFIED: Tuned thresholds and model selection
YOLO_MODEL = 'yolov8s.pt'  # Switched to a more accurate model
YOLO_CONF_THRESHOLD = 0.3   # Lowered for better recall
NUMBER_CONF_THRESHOLD = 0.5

# NEW: Configuration for performance and tracking
PROCESSING_WIDTH = 1280
PROCESSING_HEIGHT = 720
IOU_THRESHOLD = 0.4  # Threshold for matching objects between frames
MAX_FRAMES_TO_SKIP = 10  # How many frames an object can be missed before being delisted
PLAYER_NUM_HISTORY = 15 # Number of recent number detections to store per player

# NEW: Boolean toggles for controlling detection features
DETECT_PLAYERS = True
DETECT_BALL = True
RECOGNIZE_PLAYER_NUMBERS = True  # Note: DETECT_PLAYERS must be True for this to have an effect

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

def calculate_iou(boxA, boxB):
    xA = max(boxA[0], boxB[0])
    yA = max(boxA[1], boxB[1])
    xB = min(boxA[2], boxB[2])
    yB = min(boxA[3], boxB[3])
    interArea = max(0, xB - xA) * max(0, yB - yA)
    boxAArea = (boxA[2] - boxA[0]) * (boxA[3] - boxA[1])
    boxBArea = (boxB[2] - boxB[0]) * (boxB[3] - boxB[1])
    iou = interArea / float(boxAArea + boxBArea - interArea) if (boxAArea + boxBArea - interArea) > 0 else 0
    return iou


# =============================================================================
# --- Ball Detection with Background Subtraction and Blob Analysis ---
# =============================================================================
def pt_dist(x1, y1, x2, y2):
  dx = x1 - x2
  dy = y1 - y2
  return math.sqrt(dx * dx + dy * dy)

class Blob:
  cnt = 1
  def __init__(self, x, y, r, a):
    self.id = Blob.cnt
    Blob.cnt += 1
    self.pts = [[x, y]]
    self.pp = [[r, a]]
    self.status = 0 # 0: init, 1: static, 2: directed
    self.v = None
    self.age = a
    self.nx = None
    self.ny = None

  def fit(self, x, y, r):
    d = pt_dist(self.pts[-1][0], self.pts[-1][1], x, y)
    return d < 60, d # R=60 from blobber.py

  def add(self, x, y, r, a):
    self.pts.append([x, y])
    self.pp.append([r, a])
    self.age = a
    if len(self.pts) > 2:
      dx1 = self.pts[-2][0] - self.pts[-3][0]
      dy1 = self.pts[-2][1] - self.pts[-3][1]
      dx2 = x - self.pts[-2][0]
      dy2 = y - self.pts[-2][1]
      d1 = pt_dist(self.pts[-2][0], self.pts[-2][1], x, y)
      d2 = pt_dist(self.pts[-2][0], self.pts[-2][1], self.pts[-3][0], self.pts[-3][1])
      if dx1 * dx2 > 0 and dy1 * dy2 > 0 and d1 > 5 and d2 > 5:
        self.status = 2
      elif self.status != 2:
        self.status = 1

  def predict(self):
    npts = np.array(self.pts)
    l = len(self.pts) + 1
    idx = np.array(range(1, l))
    kx = np.polyfit(idx, npts[:,0], 1)
    fkx = np.poly1d(kx)
    ky = np.polyfit(idx, npts[:,1], 1)
    fky = np.poly1d(ky)
    self.nx = fkx(l)
    self.ny = fky(l)
    return self.nx, self.ny

# =============================================================================
# --- Main Processing Thread ---
# =============================================================================
def process_frames(frame_queue, result_queue, models):
    """Worker thread function to process frames from the queue."""
    yolo_model = models['yolo']
    bbox_interpreter = models['bbox']
    classifier_interpreter = models['classifier']
    ball_model = models['ball_model']
    backSub = cv2.createBackgroundSubtractorMOG2()

    next_player_id = 0
    tracked_players = {}
    tracked_ball = {}
    frame_num = 0
    B = []
    bb = None
    cnt = 0


    yolo_classes_to_detect = []
    if DETECT_PLAYERS:
        yolo_classes_to_detect.append(0)
    if DETECT_BALL:
        yolo_classes_to_detect.append(32)

    while True:
        frame = frame_queue.get()
        if frame is None:
            break

        frame_num += 1

        # --- YOLO Detection ---
        if yolo_classes_to_detect:
            results = yolo_model.predict(frame, conf=YOLO_CONF_THRESHOLD, classes=yolo_classes_to_detect, verbose=False)
            current_detections = {"players": [], "ball": []}
            for box in results[0].boxes:
                coords = [int(i) for i in box.xyxy[0]]
                cls_id = int(box.cls[0])
                if DETECT_PLAYERS and cls_id == 0:
                    current_detections["players"].append(coords)
                elif DETECT_BALL and cls_id == 32:
                    current_detections["ball"].append(coords)
        else:
             current_detections = {"players": [], "ball": []}


        # --- Player Tracking & Number Recognition ---
        if DETECT_PLAYERS:
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
                            data["history"].append((number_pred, confidence))
                else:
                    data["last_seen"] += 1
                    if data["last_seen"] > MAX_FRAMES_TO_SKIP:
                        del tracked_players[pid]

        # --- Ball Detection with Background Subtraction ---
        if DETECT_BALL:
            fgMask = backSub.apply(frame)
            fgMask = cv2.dilate(fgMask, None, iterations=2)
            fgMask = cv2.GaussianBlur(fgMask, (15, 15), 0)
            _, fgMask = cv2.threshold(fgMask, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)

            contours, _ = cv2.findContours(fgMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # --- Blob Analysis and Ball Classification ---
            prev_bb = bb
            bb = None

            for c in contours:
                (x, y, w, h) = cv2.boundingRect(c)
                if w < 10 or h < 10 or w > 50 or h > 50:
                    continue

                aspect_ratio = float(w) / h
                if aspect_ratio < 0.8 or aspect_ratio > 1.2:
                    continue

                cut_mask = fgMask[y:y+h, x:x+w]
                cut_frame = frame[y:y+h, x:x+w]

                if cut_mask.size == 0 or cut_frame.size == 0:
                    continue

                cut_color = cv2.bitwise_and(cut_frame, cut_frame, mask=cut_mask)

                # Use ball_net to classify
                img_array = cv2.resize(cut_color, (32, 32))
                img_array = np.expand_dims(img_array, axis=0)
                prediction = ball_model.predict(img_array, verbose=0)

                if np.argmax(prediction) == 0:  # Assuming class 0 is ball
                    b = find_fblob(x + w//2, y + h//2, (w+h)//4)
                    if b is None:
                        B.append(Blob(x + w//2, y + h//2, (w+h)//4, cnt))
                    else:
                        b.add(x + w//2, y + h//2, (w+h)//4, cnt)
                        if b.status == 2:
                            if bb is None or len(b.pts) > len(bb.pts):
                                bb = b
            cnt += 1

        # --- Drawing Logic ---
        if DETECT_PLAYERS:
            for pid, data in tracked_players.items():
                x1, y1, x2, y2 = data["bbox"]
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                label = f"PlayerID: {pid}"
                if RECOGNIZE_PLAYER_NUMBERS:
                    high_conf_numbers = [num for num, conf in data["history"] if num != -1 and conf > NUMBER_CONF_THRESHOLD + 0.2]
                    if high_conf_numbers:
                        stable_num = Counter(high_conf_numbers).most_common(1)[0][0]
                        label = f"Player {stable_num}"
                    else:
                        label = "Player ?"
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 0, 0), 3)

        if DETECT_BALL:
            if bb is not None:
                cv2.circle(frame, (bb.pts[-1][0], bb.pts[-1][1]), 10, (0, 255, 0), 3)
                for p in bb.pts:
                    cv2.circle(frame, (p[0], p[1]), 3, (150, 150, 150), -1)
            elif prev_bb is not None:
                x_pred, y_pred = prev_bb.predict()
                cv2.circle(frame, (int(x_pred), int(y_pred)), 10, (0, 200, 200), 3) # Predicted position in yellow
                for p in prev_bb.pts:
                    cv2.circle(frame, (p[0], p[1]), 3, (150, 150, 150), -1)

        result_queue.put(frame)


def find_fblob(x, y, r):
    # This is a simplified version of find_fblob from blobber.py
    # for integration purposes. A more robust implementation might be needed.
    return None

# =============================================================================
# --- Main Pipeline ---
# =============================================================================
def main():
    if RECOGNIZE_PLAYER_NUMBERS and not DETECT_PLAYERS:
        print("[WARNING] RECOGNIZE_PLAYER_NUMBERS is True but DETECT_PLAYERS is False.")
        print("[WARNING] Number recognition will not run. Disabling it.")
        RECOGNIZE_PLAYER_NUMBERS = False

    print("[INFO] Loading models...")

    json_file = open(BALL_MODEL_JSON_PATH, 'r')
    loaded_model_json = json_file.read()
    json_file.close()
    ball_model = tf.keras.models.model_from_json(loaded_model_json)
    ball_model.load_weights(BALL_MODEL_PATH)


    models = {
        'yolo': YOLO(YOLO_MODEL),
        'bbox': load_tflite_model(BBOX_MODEL_PATH),
        'classifier': load_tflite_model(CLASSIFICATION_MODEL_PATH),
        'ball_model': ball_model
    }

    print("[INFO] Opening video file...")
    video = cv2.VideoCapture(INPUT_VIDEO_PATH)
    if not video.isOpened():
        print(f"[ERROR] Could not open video file: {INPUT_VIDEO_PATH}")
        return

    os.makedirs(os.path.dirname(OUTPUT_VIDEO_PATH), exist_ok=True)
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    writer = cv2.VideoWriter(OUTPUT_VIDEO_PATH, fourcc, video.get(cv2.CAP_PROP_FPS), (PROCESSING_WIDTH, PROCESSING_HEIGHT))

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

        resized_frame = cv2.resize(frame, (PROCESSING_WIDTH, PROCESSING_HEIGHT))
        frame_queue.put(resized_frame)

        if not result_queue.empty():
            processed_frame = result_queue.get()
            writer.write(processed_frame)
            frame_count += 1

            if frame_count > 0 and frame_count % 50 == 0:
                print(f"[INFO] Processed {frame_count} frames...")

    # --- Cleanup ---
    frame_queue.put(None)
    processing_thread.join()

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