# train_model_generator.py (Corrected for Pi 5)

import os
import cv2
import numpy as np
import tensorflow as tf
from tensorflow.keras.utils import Sequence, to_categorical
from keras.models import Sequential
from keras.layers import Conv2D, MaxPooling2D, Dropout, Flatten, Dense

# =============================================================================
# Configuration
# =============================================================================
DATASET_BASE_PATH = '/home/hecke/Code/SJN-210k' # Please confirm this path is correct
OUTPUT_DIR = 'drive/MyDrive/Colab Notebooks/numbers_detection'
NUMBER_CLASSIFICATION_MODEL_PATH = os.path.join(OUTPUT_DIR, 'numbers_classifier_multidigit.h5')

IMG_WIDTH = 224
IMG_HEIGHT = 224
BATCH_SIZE = 8  # Reduced batch size for Raspberry Pi's limited RAM
NUM_CLASSES = 100
EPOCHS = 10     # Start with fewer epochs to test, then increase to 80 later

# =============================================================================
# Data Generator (Keras Sequence) - Unchanged
# =============================================================================
class JerseyNumberGenerator(Sequence):
    """A memory-efficient Keras Sequence to generate batches of data."""
    def __init__(self, label_file_path, image_base_dir, batch_size, num_classes):
        self.samples = []
        self.image_base_dir = image_base_dir
        self.batch_size = batch_size
        self.num_classes = num_classes
        self._load_samples(label_file_path)

    def _load_samples(self, label_file_path):
        if not os.path.exists(label_file_path): return
        with open(label_file_path, 'r') as f:
            for line in f:
                parts = line.strip().split()
                if len(parts) != 11: continue
                num1, num2 = int(parts[1]), int(parts[2])
                if num1 == 10: continue
                class_id = num1 if num2 == 10 else num1 * 10 + num2
                self.samples.append({
                    'img_name': parts[0], 'class_id': class_id,
                    'coords': [float(p) for p in parts[3:]]
                })

    def __len__(self):
        return int(np.floor(len(self.samples) / self.batch_size))

    def __getitem__(self, index):
        batch_samples = self.samples[index * self.batch_size:(index + 1) * self.batch_size]
        X = np.empty((self.batch_size, IMG_HEIGHT, IMG_WIDTH, 3), dtype="float32")
        y = np.empty((self.batch_size), dtype="int")

        for i, sample in enumerate(batch_samples):
            img_path = os.path.join(self.image_base_dir, sample['img_name'])
            original_image = cv2.imread(img_path)
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
            X[i,] = resized_crop / 255.0
            y[i] = sample['class_id']
            
        return X, to_categorical(y, num_classes=self.num_classes)

# =============================================================================
# Model Training Function
# =============================================================================
def train_number_classifier_with_generator():
    """Trains the number classifier using the corrected architecture and data generator."""
    print("\n[INFO] Training the multi-digit number classifier with a data generator...")
    
    train_label_path = os.path.join(DATASET_BASE_PATH, 'train', 'train_pos_label.txt')
    train_image_dir = os.path.join(DATASET_BASE_PATH, 'train', 'train_pos_image')
    test_label_path = os.path.join(DATASET_BASE_PATH, 'test', 'test_pos_label.txt')
    test_image_dir = os.path.join(DATASET_BASE_PATH, 'test', 'test_pos_image')
    
    train_generator = JerseyNumberGenerator(train_label_path, train_image_dir, BATCH_SIZE, NUM_CLASSES)
    test_generator = JerseyNumberGenerator(test_label_path, test_image_dir, BATCH_SIZE, NUM_CLASSES)

    # --- CORRECTED MODEL ARCHITECTURE ---
    # This matches the efficient architecture from your original notebook.
    classifier = Sequential()
    classifier.add(Conv2D(128, (3, 3), input_shape=(224, 224, 3), activation='relu'))
    classifier.add(MaxPooling2D(pool_size=(2, 2)))
    classifier.add(Dropout(0.2))

    classifier.add(Conv2D(64, (3, 3), activation='relu'))
    classifier.add(MaxPooling2D(pool_size=(2, 2)))
    classifier.add(Dropout(0.2))

    classifier.add(Conv2D(64, (3, 3), activation='relu'))
    classifier.add(MaxPooling2D(pool_size=(2, 2)))
    classifier.add(Dropout(0.2))

    classifier.add(Conv2D(32, (3, 3), activation='relu'))
    classifier.add(MaxPooling2D(pool_size=(2, 2)))
    classifier.add(Dropout(0.2))

    # This extra Conv2D was in the original notebook and is important for reducing size
    classifier.add(Conv2D(32, (3, 3), activation='relu'))
    classifier.add(MaxPooling2D(pool_size=(2, 2)))
    classifier.add(Dropout(0.2))

    classifier.add(Flatten())
    classifier.add(Dense(units=128, activation='relu'))
    classifier.add(Dense(units=64, activation='relu'))
    classifier.add(Dense(units=64, activation='relu'))
    classifier.add(Dense(units=NUM_CLASSES, activation='softmax'))

    classifier.compile(optimizer='adam', loss='categorical_crossentropy', metrics=['accuracy'])
    print(classifier.summary())
    
    print("[INFO] Starting training...")
    # --- CORRECTED .fit() call ---
    # Removed the 'workers' argument which is not supported on your system.
    H = classifier.fit(
        train_generator,
        validation_data=test_generator,
        epochs=EPOCHS
    )

    os.makedirs(OUTPUT_DIR, exist_ok=True)
    print(f"[INFO] Saving model to {NUMBER_CLASSIFICATION_MODEL_PATH}...")
    classifier.save(NUMBER_CLASSIFICATION_MODEL_PATH)
    print("[INFO] Training complete.")

# =============================================================================
# Main Execution
# =============================================================================
if __name__ == '__main__':
    train_number_classifier_with_generator()