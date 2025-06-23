# detect_volleyball_multidigit.py

import os
import cv2
import numpy as np
import tensorflow as tf
from ultralytics import YOLO

# =============================================================================
# Configuration
# =============================================================================
MODEL_DIR = 'drive/MyDrive/Colab Notebooks/numbers_detection'
# --- IMPORTANT: Update the model path to your new multi-digit model ---
CLASSIFICATION_MODEL_PATH = os.path.join(MODEL_DIR, 'numbers_classifier_multidigit.h5')
BBOX_MODEL_PATH = os.path.join(MODEL_DIR, 'number_detection.h5')

INPUT_VIDEO_PATH = os.path.join(MODEL_DIR, 'video_samples/input/example.mp4')
OUTPUT_VIDEO_PATH = os.path.join(MODEL_DIR, 'video_samples/output/volleyball_output.mp4')

YOLO_CONF_THRESHOLD = 0.5
NUMBER_CONF_THRESHOLD = 0.4
# Image dimensions for the models
IMG_WIDTH = 224
IMG_HEIGHT = 224

# Ensure output directory exists
os.makedirs(OUTPUT_VIDEO_PATH, exist_ok=True)

# =============================================================================
# Helper Functions
# =============================================================================

def parse_annotation(label_path, img_width, img_height):
    """
    Parses a single annotation file to get class ID and bounding box.
    
    Args:
        label_path (str): The path to the .txt annotation file.
        img_width (int): The original width of the corresponding image.
        img_height (int): The original height of the corresponding image.
        
    Returns:
        A tuple containing (class_id, normalized_bbox) or (None, None) if parsing fails.
        The normalized_bbox is in the format [startX, startY, endX, endY].
    """
    try:
        with open(label_path, 'r') as f:
            line = f.readline().strip()
            parts = line.split()
            
            # --- IMPORTANT ---
            # This assumes the format: class_id x_min y_min x_max y_max
            # If your format is different (e.g., YOLO format: class_id x_center y_center width height),
            # you will need to modify the parsing logic here.
            class_id = int(parts[0])
            xmin = float(parts[1])
            ymin = float(parts[2])
            xmax = float(parts[3])
            ymax = float(parts[4])

            # Normalize coordinates to be between 0 and 1
            norm_bbox = [
                xmin / img_width,
                ymin / img_height,
                xmax / img_width,
                ymax / img_height
            ]
            
            return class_id, norm_bbox
            
    except (IOError, IndexError, ValueError) as e:
        print(f"[WARNING] Could not parse {label_path}: {e}")
        return None, None

def process_data(image_dir, label_dir):
    """
    Processes all images and labels in a given directory (e.g., 'train' or 'test').
    
    Args:
        image_dir (str): Path to the directory containing images.
        label_dir (str): Path to the directory containing labels.
        
    Returns:
        A tuple of lists: (full_images, number_images, bbox_targets, class_labels)
    """
    full_images = []
    number_images = []
    bbox_targets = []
    class_labels = []

    image_files = [f for f in os.listdir(image_dir) if f.endswith(('.jpg', '.png'))]
    
    for filename in image_files:
        basename = os.path.splitext(filename)[0]
        img_path = os.path.join(image_dir, filename)
        label_path = os.path.join(label_dir, f"{basename}.txt")

        if not os.path.exists(label_path):
            print(f"[WARNING] Label file not found for {filename}, skipping.")
            continue

        # Load the original image to get its dimensions and for cropping
        original_image = cv2.imread(img_path)
        if original_image is None:
            continue
        original_image = cv2.cvtColor(original_image, cv2.COLOR_BGR2RGB)
        h, w, _ = original_image.shape

        # Parse the annotation file to get number class and bbox
        class_id, norm_bbox = parse_annotation(label_path, w, h)
        if class_id is None:
            continue
            
        # --- Prepare data for BBox Detection Model ---
        # Resize the full image and normalize pixel values
        full_img_resized = cv2.resize(original_image, (IMG_HEIGHT, IMG_WIDTH))
        full_images.append(full_img_resized / 255.0)
        bbox_targets.append(norm_bbox)

        # --- Prepare data for Classification Model ---
        # Denormalize bbox to crop from original image
        xmin = int(norm_bbox[0] * w)
        ymin = int(norm_bbox[1] * h)
        xmax = int(norm_bbox[2] * w)
        ymax = int(norm_bbox[3] * h)

        # Crop the number from the original image
        number_crop = original_image[ymin:ymax, xmin:xmax]
        if number_crop.shape[0] == 0 or number_crop.shape[1] == 0:
            continue

        # Resize cropped number image for the classifier and normalize
        number_img_resized = cv2.resize(number_crop, (IMG_HEIGHT, IMG_WIDTH))
        number_images.append(number_img_resized / 255.0)
        class_labels.append(class_id)

    return full_images, number_images, bbox_targets, class_labels

# =============================================================================
# Main Script Execution
# =============================================================================

def main():
    """Main function to process the video with multi-digit detection."""
    print("[INFO] Loading models for multi-digit detection...")
    yolo_model = YOLO('yolov8n.pt')

    try:
        classifier_bbox_numbers = tf.keras.models.load_model(BBOX_MODEL_PATH)
        # --- NEW: Load the multi-digit classifier ---
        classifier_numbers = tf.keras.models.load_model(CLASSIFICATION_MODEL_PATH)
    except IOError as e:
        print(f"[ERROR] Could not load a required model: {e}")
        return

    print("[INFO] Opening video file...")
    video = cv2.VideoCapture(INPUT_VIDEO_PATH)
    if not video.isOpened():
        print(f"[ERROR] Could not open video file: {INPUT_VIDEO_PATH}")
        return

    # --- Video Writer Setup ---
    frame_width = int(video.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(video.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = int(video.get(cv2.CAP_PROP_FPS))
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    writer = cv2.VideoWriter(OUTPUT_VIDEO_PATH, fourcc, fps, (frame_width, frame_height))
    print(f"[INFO] Output will be saved to {OUTPUT_VIDEO_PATH}")

    # --- Main Processing Loop ---
    while True:
        ret, frame = video.read()
        if not ret: break

        results = yolo_model.predict(frame, conf=YOLO_CONF_THRESHOLD, classes=[0, 32], verbose=False)
        boxes = results[0].boxes
        
        for box in boxes:
            x1, y1, x2, y2 = [int(i) for i in box.xyxy[0]]
            cls_id = int(box.cls[0])
            
            if cls_id == 0: # Player detected
                # (The logic for cropping player and number bbox is the same)
                player_image = frame[y1:y2, x1:x2]
                if player_image.shape[0] == 0 or player_image.shape[1] == 0: continue
                
                
                number_pred = identify_number(number_crop, classifier_numbers)
                
                # For demonstration, let's assume identify_number is defined elsewhere
                # and works correctly with the new model. The argmax of the model's
                # prediction will be the number itself.

                # Simplified logic for display
                label = f"Player" # Placeholder if number not found
                # if number_pred != -1:
                #     label = f"Player {number_pred}"
                
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 0), 2)
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
            
            elif cls_id == 32: # Ball detected
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, "Ball", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        writer.write(frame)

    print("[INFO] Processing complete.")
    video.release()
    writer.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()