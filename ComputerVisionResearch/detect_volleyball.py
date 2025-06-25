# detect_volleyball_with_counter.py

# =============================================================================
# --- GPU Configuration & Imports ---
# =============================================================================
import torch 
import tensorflow as tf

gpus = tf.config.experimental.list_physical_devices('GPU')
if gpus:
    try:
        for gpu in gpus:
            tf.config.experimental.set_memory_growth(gpu, True)
        print(f"[INFO] {len(gpus)} GPU(s) found and configured for memory growth.")
    except RuntimeError as e:
        print(f"[ERROR] Could not configure GPU memory growth: {e}")

# --- Standard Imports ---
import os
import cv2
import numpy as np
from ultralytics import YOLO

# =============================================================================
# Configuration
# =============================================================================
MODEL_DIR = 'drive/MyDrive/Colab Notebooks/numbers_detection' 
CLASSIFICATION_MODEL_PATH = os.path.join(MODEL_DIR, 'numbers_classifier_multidigit_model.h5')
BBOX_MODEL_PATH = os.path.join(MODEL_DIR, 'number_detection_model.h5')

INPUT_VIDEO_PATH = os.path.join(MODEL_DIR, 'video_samples/input/IMG_9086.mp4')
OUTPUT_VIDEO_PATH = os.path.join(MODEL_DIR, 'video_samples/output/volleyball_output_final.mp4')

YOLO_CONF_THRESHOLD = 0.5
NUMBER_CONF_THRESHOLD = 0.4

# =============================================================================
# Helper Functions
# =============================================================================

def detect_number_bbox(player_image, bbox_model):
    """Detects the bounding box of the number within a player's image."""
    image = cv2.resize(player_image, (224, 224))
    image = image.astype("float32") / 255.0
    image = np.expand_dims(image, axis=0)
    preds_bbox = bbox_model.predict(image, verbose=0)[0]
    return tuple(preds_bbox)

def identify_number(number_image, classifier_model):
    """Classifies the digit within the number's bounding box."""
    if number_image.shape[0] == 0 or number_image.shape[1] == 0:
        return -1
    
    image = cv2.resize(number_image, (224, 224))
    image = image.astype("float32") / 255.0
    image = np.expand_dims(image, axis=0)
    preds = classifier_model.predict(image, verbose=0)[0]
    
    i = np.argmax(preds)
    return i if preds[i] > NUMBER_CONF_THRESHOLD else -1

# =============================================================================
# Main Inference Pipeline
# =============================================================================

def main():
    """Main function to process the video and perform detections."""
    print("[INFO] Loading models...")
    yolo_model = YOLO('yolov8n.pt')

    try:
        custom_objects = {"mse": tf.keras.losses.MeanSquaredError()}
        classifier_bbox_numbers = tf.keras.models.load_model(BBOX_MODEL_PATH, custom_objects=custom_objects)
        classifier_numbers = tf.keras.models.load_model(CLASSIFICATION_MODEL_PATH)
    except Exception as e:
        print(f"[ERROR] Could not load a required model. Please check paths and file integrity.")
        print(f"Error details: {e}")
        return

    print("[INFO] Opening video file...")
    video = cv2.VideoCapture(INPUT_VIDEO_PATH)
    if not video.isOpened():
        print(f"[ERROR] Could not open video file: {INPUT_VIDEO_PATH}")
        return

    frame_width = int(video.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(video.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = int(video.get(cv2.CAP_PROP_FPS))
    
    os.makedirs(os.path.dirname(OUTPUT_VIDEO_PATH), exist_ok=True)
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    writer = cv2.VideoWriter(OUTPUT_VIDEO_PATH, fourcc, fps, (frame_width, frame_height))
    print(f"[INFO] Output will be saved to {OUTPUT_VIDEO_PATH}")

    # --- NEW: Initialize frame counter ---
    frame_count = 0

    # --- Main Processing Loop ---
    while True:
        ret, frame = video.read()
        if not ret:
            break # Exit the loop if the video has ended

        # --- NEW: Increment and print frame count ---
        frame_count += 1
        if frame_count % 50 == 0:
            print(f"[INFO] Processing frame {frame_count}...")


        # Run YOLOv8 detection
        results = yolo_model.predict(frame, conf=YOLO_CONF_THRESHOLD, classes=[0, 32], verbose=False)
        
        for box in results[0].boxes:
            x1, y1, x2, y2 = [int(i) for i in box.xyxy[0]]
            cls_id = int(box.cls[0])
            
            if cls_id == 0: # Person detected
                player_image = frame[y1:y2, x1:x2]
                if player_image.shape[0] == 0 or player_image.shape[1] == 0: continue
                
                (startX_rel, startY_rel, endX_rel, endY_rel) = detect_number_bbox(player_image, classifier_bbox_numbers)
                
                h, w, _ = player_image.shape
                startX_abs = int(startX_rel * w)
                startY_abs = int(startY_rel * h)
                endX_abs = int(endX_rel * w)
                endY_abs = int(endY_rel * h)
                
                # Use a try-except block for robustness against invalid crop dimensions
                try:
                    number_crop = player_image[startY_abs:endY_abs, startX_abs:endX_abs]
                    number_pred = identify_number(number_crop, classifier_numbers)
                except:
                    number_pred = -1

                # Drawing logic
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                label = f"Player {number_pred}" if number_pred != -1 else "Player"
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)

            elif cls_id == 32: # Sports ball detected
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, "Ball", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

        # Write the processed frame to the output video file
        writer.write(frame)

    # --- Cleanup ---
    print(f"\n[INFO] Processing complete. Total frames processed: {frame_count}")
    video.release()
    writer.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()