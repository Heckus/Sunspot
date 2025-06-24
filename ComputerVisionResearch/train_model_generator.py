# train_models_complete.py

import os
import cv2
import glob
import numpy as np
import tensorflow as tf
from tensorflow.keras.utils import Sequence, to_categorical
from tensorflow.keras.applications import MobileNetV2
from tensorflow.keras.layers import Input, Flatten, Dense
from tensorflow.keras.models import Model
from tensorflow.keras.optimizers import Adam
from keras.models import Sequential
from keras.layers import Conv2D, MaxPooling2D, Dropout
# Import the callbacks
from tensorflow.keras.callbacks import ModelCheckpoint, EarlyStopping

# =============================================================================
# Configuration
# =============================================================================
DATASET_BASE_PATH = '/home/hecke/Documents/SJN-210k'
OUTPUT_DIR = 'drive/MyDrive/Colab Notebooks/numbers_detection'

# Updated model paths to reflect they are the "best" versions
BBOX_MODEL_PATH = os.path.join(OUTPUT_DIR, 'number_detection_best.h5')
CLASSIFICATION_MODEL_PATH = os.path.join(OUTPUT_DIR, 'numbers_classifier_multidigit_best.h5')

IMG_WIDTH = 224
IMG_HEIGHT = 224
BATCH_SIZE = 16
NUM_CLASSES = 100
# Set a high number of epochs; EarlyStopping will find the best time to stop.
EPOCHS = 100 

# =============================================================================
# Data Loading and Generator Classes (Unchanged)
# =============================================================================
def load_sample_descriptions(label_file_path, image_base_dir):
    """Loads and parses the label file once to create a list of all valid samples."""
    samples = []
    if not os.path.exists(label_file_path):
        print(f"[ERROR] Label file not found: {label_file_path}")
        return samples
    
    with open(label_file_path, 'r') as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) != 11: continue
            
            img_name_from_label = parts[0]
            base_name = os.path.splitext(img_name_from_label)[0]
            search_pattern = os.path.join(image_base_dir, base_name + '.*')
            found_files = glob.glob(search_pattern)

            if not found_files: continue
            
            img_path = found_files[0]
            num1, num2 = int(parts[1]), int(parts[2])
            if num1 == 10: continue
            class_id = num1 if num2 == 10 else num1 * 10 + num2

            coords = np.array([float(p) for p in parts[3:]]).reshape(4, 2)
            startX, startY = coords.min(axis=0)
            endX, endY = coords.max(axis=0)
            
            samples.append({
                'img_path': img_path, 'class_id': class_id,
                'bbox': [startX, startY, endX, endY]
            })
    return samples

class BboxDataGenerator(Sequence):
    """Generates batches of full images and bbox coordinates."""
    def __init__(self, samples, batch_size):
        self.samples = samples
        self.batch_size = batch_size

    def __len__(self):
        return int(np.floor(len(self.samples) / self.batch_size))

    def __getitem__(self, index):
        batch_samples = self.samples[index * self.batch_size:(index + 1) * self.batch_size]
        X = np.empty((len(batch_samples), IMG_HEIGHT, IMG_WIDTH, 3), dtype="float32")
        y = np.empty((len(batch_samples), 4), dtype="float32")

        for i, sample in enumerate(batch_samples):
            image = cv2.imread(sample['img_path'])
            if image is None: continue
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            resized_image = cv2.resize(image, (IMG_HEIGHT, IMG_WIDTH))
            X[i,] = resized_image / 255.0
            y[i,] = sample['bbox']
        return X, y
    
class JerseyNumberGenerator(Sequence):
    """Generates batches of cropped number images and their class labels."""
    def __init__(self, samples, batch_size, num_classes):
        self.samples = samples
        self.batch_size = batch_size
        self.num_classes = num_classes

    def __len__(self):
        return int(np.floor(len(self.samples) / self.batch_size))

    def __getitem__(self, index):
        batch_samples = self.samples[index * self.batch_size:(index + 1) * self.batch_size]
        batch_X, batch_y = [], []
        for sample in batch_samples:
            original_image = cv2.imread(sample['img_path'])
            if original_image is None: continue
            original_image = cv2.cvtColor(original_image, cv2.COLOR_BGR2RGB)
            h, w, _ = original_image.shape
            coords = np.array(sample['coords']).reshape(4, 2)
            startX, startY = coords.min(axis=0)
            endX, endY = coords.max(axis=0)
            xmin, ymin = int(startX * w), int(startY * h)
            xmax, ymax = int(endX * w), int(endY * h)
            number_crop = original_image[ymin:ymax, xmin:xmax]
            if number_crop.shape[0] == 0 or number_crop.shape[1] == 0: continue
            resized_crop = cv2.resize(number_crop, (IMG_HEIGHT, IMG_WIDTH))
            batch_X.append(resized_crop / 255.0)
            batch_y.append(sample['class_id'])
        return np.array(batch_X), to_categorical(np.array(batch_y), num_classes=self.num_classes)


# =============================================================================
# Training Functions (Updated with Callbacks)
# =============================================================================

def train_number_bbox_detector(train_samples, test_samples):
    """Trains the bounding box detection model with callbacks."""
    print("\n[INFO] Training the number bounding box detector...")
    
    train_generator = BboxDataGenerator(train_samples, BATCH_SIZE)
    test_generator = BboxDataGenerator(test_samples, BATCH_SIZE)

    base_model = MobileNetV2(weights="imagenet", include_top=False, input_tensor=Input(shape=(224, 224, 3)))
    base_model.trainable = False
    
    flatten = Flatten()(base_model.output)
    bboxHead = Dense(128, activation="relu")(flatten)
    bboxHead = Dense(64, activation="relu")(bboxHead)
    bboxHead = Dense(32, activation="relu")(bboxHead)
    bboxHead = Dense(4, activation="sigmoid")(bboxHead)
    model = Model(inputs=base_model.input, outputs=bboxHead)

    opt = Adam(learning_rate=1e-4)
    model.compile(loss="mse", optimizer=opt)
    
    # --- Define Callbacks for BBox Detector ---
    # Save the best model based on validation loss
    checkpoint = ModelCheckpoint(BBOX_MODEL_PATH, monitor='val_loss', save_best_only=True, mode='min', verbose=1)
    # Stop training if val_loss doesn't improve for 5 consecutive epochs
    early_stopping = EarlyStopping(monitor='val_loss', patience=5, verbose=1, mode='min')

    print("[INFO] Starting BBox detector training...")
    model.fit(train_generator, validation_data=test_generator, epochs=EPOCHS, callbacks=[checkpoint, early_stopping])
    print("[INFO] BBox model training complete.")


def train_number_classifier(train_samples, test_samples):
    """Trains the number classification model with callbacks."""
    print("\n[INFO] Training the multi-digit number classifier...")
    
    train_generator = JerseyNumberGenerator(train_samples, BATCH_SIZE, NUM_CLASSES)
    test_generator = JerseyNumberGenerator(test_samples, BATCH_SIZE, NUM_CLASSES)
    
    classifier = Sequential([
        Conv2D(128, (3, 3), input_shape=(224, 224, 3), activation='relu'),
        MaxPooling2D(pool_size=(2, 2)), Dropout(0.2),
        Conv2D(64, (3, 3), activation='relu'),
        MaxPooling2D(pool_size=(2, 2)), Dropout(0.2),
        Conv2D(64, (3, 3), activation='relu'),
        MaxPooling2D(pool_size=(2, 2)), Dropout(0.2),
        Conv2D(32, (3, 3), activation='relu'),
        MaxPooling2D(pool_size=(2, 2)), Dropout(0.2),
        Conv2D(32, (3, 3), activation='relu'),
        MaxPooling2D(pool_size=(2, 2)), Dropout(0.2),
        Flatten(),
        Dense(128, activation='relu'),
        Dense(64, activation='relu'),
        Dense(64, activation='relu'),
        Dense(NUM_CLASSES, activation='softmax')
    ])

    classifier.compile(optimizer='adam', loss='categorical_crossentropy', metrics=['accuracy'])

    # --- Define Callbacks for Classifier ---
    # Save the best model based on validation accuracy
    checkpoint = ModelCheckpoint(CLASSIFICATION_MODEL_PATH, monitor='val_accuracy', save_best_only=True, mode='max', verbose=1)
    # Stop training if val_loss doesn't improve for 10 consecutive epochs (patience is higher here)
    early_stopping = EarlyStopping(monitor='val_loss', patience=10, verbose=1, mode='min')

    print("[INFO] Starting classifier training...")
    classifier.fit(train_generator, validation_data=test_generator, epochs=EPOCHS, callbacks=[checkpoint, early_stopping])
    print("[INFO] Classifier training complete.")

# =============================================================================
# Main Execution
# =============================================================================
if __name__ == '__main__':
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    
    train_label_path = os.path.join(DATASET_BASE_PATH, 'train', 'train_pos_label.txt')
    train_image_dir = os.path.join(DATASET_BASE_PATH, 'train', 'train_pos_image')
    test_label_path = os.path.join(DATASET_BASE_PATH, 'test', 'test_pos_label.txt')
    test_image_dir = os.path.join(DATASET_BASE_PATH, 'test', 'test_pos_image')
    
    print("[INFO] Loading and parsing dataset labels...")
    train_samples_list = load_sample_descriptions(train_label_path, train_image_dir)
    test_samples_list = load_sample_descriptions(test_label_path, test_image_dir)
    
    if not train_samples_list or not test_samples_list:
        print("[ERROR] No valid samples found. Please check dataset paths and label files.")
    else:
        # Run both training processes sequentially
        train_number_bbox_detector(train_samples_list, test_samples_list)
        train_number_classifier(train_samples_list, test_samples_list)
        
        print("\n[SUCCESS] Both models have been trained and saved.")