# prepare_dataset_multidigit.py

import os
import cv2
import pickle
import numpy as np
from tensorflow.keras.utils import to_categorical

# =============================================================================
# Configuration
# =============================================================================
DATASET_BASE_PATH = '/home/hecke/Code/SJN-210k'
OUTPUT_DIR = 'drive/MyDrive/Colab Notebooks/numbers_detection'
BBOX_DATA_PATH = os.path.join(OUTPUT_DIR, 'data_bbox.pickle')
CLASSIFICATION_DATA_PATH = os.path.join(OUTPUT_DIR, 'data.pickle')
IMG_WIDTH = 224
IMG_HEIGHT = 224

os.makedirs(OUTPUT_DIR, exist_ok=True)

# =============================================================================
# Data Processing Function
# =============================================================================

def process_data(label_file_path, image_base_dir):
    """
    Processes all images and labels, now handling two-digit numbers.
    """
    full_images, number_images, bbox_targets, class_labels, filenames = [], [], [], [], []

    print(f"Reading labels from: {label_file_path}")
    with open(label_file_path, 'r') as f:
        lines = f.readlines()

    for line in lines:
        parts = line.strip().split()
        if len(parts) != 11:
            continue

        img_name, num1_str, num2_str = parts[0], parts[1], parts[2]
        num1, num2 = int(num1_str), int(num2_str)

        # --- NEW: Logic for handling two-digit numbers ---
        # If num2 is 10 (blank), the class is just num1.
        # Otherwise, the class is num1*10 + num2 (e.g., 2 and 5 becomes 25).
        # We skip any labels where the first digit is blank (num1=10).
        if num1 == 10:
            continue
        class_id = num1 if num2 == 10 else num1 * 10 + num2

        img_path = os.path.join(image_base_dir, img_name)
        if not os.path.exists(img_path):
            continue

        coords = np.array([float(p) for p in parts[3:]]).reshape(4, 2)
        startX, startY = coords.min(axis=0)
        endX, endY = coords.max(axis=0)
        norm_bbox = [startX, startY, endX, endY]

        original_image = cv2.imread(img_path)
        if original_image is None: continue
        original_image = cv2.cvtColor(original_image, cv2.COLOR_BGR2RGB)
        h, w, _ = original_image.shape

        # --- Prepare data for BBox Detection Model (no change here) ---
        full_img_resized = cv2.resize(original_image, (IMG_HEIGHT, IMG_WIDTH))
        full_images.append(full_img_resized / 255.0)
        bbox_targets.append(norm_bbox)
        filenames.append(img_name)

        # --- Prepare data for Classification Model ---
        xmin_abs, ymin_abs = int(startX * w), int(startY * h)
        xmax_abs, ymax_abs = int(endX * w), int(endY * h)
        
        number_crop = original_image[ymin_abs:ymax_abs, xmin_abs:xmax_abs]
        if number_crop.shape[0] == 0 or number_crop.shape[1] == 0: continue
            
        number_img_resized = cv2.resize(number_crop, (IMG_HEIGHT, IMG_WIDTH))
        number_images.append(number_img_resized / 255.0)
        class_labels.append(class_id)
        
    return full_images, number_images, bbox_targets, class_labels, filenames

# =============================================================================
# Main Script Execution
# =============================================================================

print("[INFO] Starting dataset processing for multi-digit numbers...")

train_label_path = os.path.join(DATASET_BASE_PATH, 'train', 'train_pos_label.txt')
train_image_dir = os.path.join(DATASET_BASE_PATH, 'train', 'train_pos_image')
test_label_path = os.path.join(DATASET_BASE_PATH, 'test', 'test_pos_label.txt')
test_image_dir = os.path.join(DATASET_BASE_PATH, 'test', 'test_pos_image')

print("[INFO] Processing training data...")
(train_full_imgs, train_num_imgs, train_bbox, 
 train_labels, train_filenames) = process_data(train_label_path, train_image_dir)

print("[INFO] Processing testing data...")
(test_full_imgs, test_num_imgs, test_bbox, 
 test_labels, test_filenames) = process_data(test_label_path, test_image_dir)

print(f"[INFO] Processed {len(train_full_imgs)} training samples and {len(test_full_imgs)} testing samples.")

# --- Save BBox Data Pickle (no change in structure) ---
print(f"[INFO] Saving bbox detection data to {BBOX_DATA_PATH}...")
bbox_data_to_pickle = [np.array(train_full_imgs), [], np.array(train_bbox), train_filenames,
                       np.array(test_full_imgs), [], np.array(test_bbox), test_filenames]
with open(BBOX_DATA_PATH, 'wb') as f:
    pickle.dump(bbox_data_to_pickle, f)
print("[INFO] Bbox data saved successfully.")

# --- Save Classification Data Pickle (now with 100 classes) ---
print(f"[INFO] Saving classification data to {CLASSIFICATION_DATA_PATH}...")
# NEW: num_classes is now 100 to handle numbers 0-99.
y_train_cat = to_categorical(train_labels, num_classes=100)
y_test_cat = to_categorical(test_labels, num_classes=100)

classification_data_to_pickle = [np.array(train_num_imgs), y_train_cat,
                                 np.array(test_num_imgs), y_test_cat]
with open(CLASSIFICATION_DATA_PATH, 'wb') as f:
    pickle.dump(classification_data_to_pickle, f)
print("[INFO] Classification data saved successfully.")
print("\n[INFO] All processing complete.")