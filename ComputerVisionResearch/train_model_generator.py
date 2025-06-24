# train_models_generator.py

import os
import cv2
import numpy as np
import tensorflow as tf
from tensorflow.keras.utils import Sequence, to_categorical
from tensorflow.keras.applications import VGG16
from tensorflow.keras.layers import Flatten, Dense, Input
from tensorflow.keras.models import Model
from tensorflow.keras.optimizers import Adam
from keras.models import Sequential
from keras.layers import Conv2D, MaxPooling2D, Dropout

# =============================================================================
# Configuration
# =============================================================================
DATASET_BASE_PATH = '/home/hecke/Code/SJN-210k' #<-- SET THIS TO YOUR PATH
OUTPUT_DIR = 'drive/MyDrive/Colab Notebooks/numbers_detection'

NUMBER_CLASSIFICATION_MODEL_PATH = os.path.join(OUTPUT_DIR, 'numbers_classifier_multidigit.h5')
IMG_WIDTH = 224
IMG_HEIGHT = 224
BATCH_SIZE = 4 # You can adjust this based on your GPU memory
NUM_CLASSES = 100

# =============================================================================
# Data Generator (Keras Sequence)
# =============================================================================

class JerseyNumberGenerator(Sequence):
    """
    A Keras Sequence to generate batches of data on-the-fly from the dataset folder.
    This is highly memory-efficient.
    """
    def __init__(self, label_file_path, image_base_dir, batch_size, num_classes):
        self.label_file_path = label_file_path
        self.image_base_dir = image_base_dir
        self.batch_size = batch_size
        self.num_classes = num_classes
        self.samples = self._load_samples()

    def _load_samples(self):
        """Loads and parses the label file to create a list of samples."""
        samples = []
        with open(self.label_file_path, 'r') as f:
            for line in f:
                parts = line.strip().split()
                if len(parts) != 11: continue

                num1, num2 = int(parts[1]), int(parts[2])
                if num1 == 10: continue
                class_id = num1 if num2 == 10 else num1 * 10 + num2

                sample_info = {
                    'img_name': parts[0],
                    'class_id': class_id,
                    'coords': [float(p) for p in parts[3:]]
                }
                samples.append(sample_info)
        return samples

    def __len__(self):
        """Returns the number of batches per epoch."""
        return int(np.floor(len(self.samples) / self.batch_size))

    def __getitem__(self, index):
        """Generates one batch of data."""
        # Get the sample descriptions for the current batch
        batch_samples = self.samples[index * self.batch_size:(index + 1) * self.batch_size]

        # Initialize batch arrays
        X = np.empty((self.batch_size, IMG_HEIGHT, IMG_WIDTH, 3), dtype="float32")
        y = np.empty((self.batch_size), dtype="int")

        for i, sample in enumerate(batch_samples):
            img_path = os.path.join(self.image_base_dir, sample['img_name'])
            original_image = cv2.imread(img_path)
            if original_image is None: continue

            original_image = cv2.cvtColor(original_image, cv2.COLOR_BGR2RGB)
            h, w, _ = original_image.shape

            # Get bbox from the 4 corner points
            coords = np.array(sample['coords']).reshape(4, 2)
            startX, startY = coords.min(axis=0)
            endX, endY = coords.max(axis=0)

            # Crop the number from the original image
            xmin_abs, ymin_abs = int(startX * w), int(startY * h)
            xmax_abs, ymax_abs = int(endX * w), int(endY * h)
            number_crop = original_image[ymin_abs:ymax_abs, xmin_abs:xmax_abs]
            
            if number_crop.shape[0] == 0 or number_crop.shape[1] == 0: continue
            
            # Resize and normalize the cropped number image
            resized_crop = cv2.resize(number_crop, (IMG_HEIGHT, IMG_WIDTH))
            X[i,] = resized_crop / 255.0
            y[i] = sample['class_id']
            
        # Return the batch of images and their one-hot encoded labels
        return X, to_categorical(y, num_classes=self.num_classes)

# =============================================================================
# Model Training Function
# =============================================================================

def train_number_classifier_with_generator():
    """
    Trains the number classifier using the memory-efficient data generator.
    """
    print("\n[INFO] Training the multi-digit number classifier with a data generator...")
    
    # --- 1. Create Data Generators ---
    train_label_path = os.path.join(DATASET_BASE_PATH, 'train', 'train_pos_label.txt')
    train_image_dir = os.path.join(DATASET_BASE_PATH, 'train', 'train_pos_image')
    test_label_path = os.path.join(DATASET_BASE_PATH, 'test', 'test_pos_label.txt')
    test_image_dir = os.path.join(DATASET_BASE_PATH, 'test', 'test_pos_image')
    
    train_generator = JerseyNumberGenerator(train_label_path, train_image_dir, BATCH_SIZE, NUM_CLASSES)
    test_generator = JerseyNumberGenerator(test_label_path, test_image_dir, BATCH_SIZE, NUM_CLASSES)

    # --- 2. Define Model Architecture (same as before) ---
    classifier = Sequential()
    classifier.add(Conv2D(128, (3, 3), input_shape=(224, 224, 3), activation='relu'))
    classifier.add(MaxPooling2D(pool_size=(2, 2)))
    # ... (rest of the layers are the same)
    classifier.add(Flatten())
    classifier.add(Dense(units=128, activation='relu'))
    classifier.add(Dense(units=100, activation='softmax')) # 100 classes

    # --- 3. Compile and Train ---
    classifier.compile(optimizer='adam', loss='categorical_crossentropy', metrics=['accuracy'])
    print(classifier.summary())
    
    print("[INFO] Starting training...")
    H = classifier.fit(
        train_generator,
        validation_data=test_generator,
        epochs=80, # Set number of epochs
        workers=8  # Use multiple CPU cores to load data in parallel
    )

    # --- 4. Save Model ---
    print(f"[INFO] Saving model to {NUMBER_CLASSIFICATION_MODEL_PATH}...")
    classifier.save(NUMBER_CLASSIFICATION_MODEL_PATH)
    print("[INFO] Training complete.")

# =============================================================================
# Main Execution
# =============================================================================
if __name__ == '__main__':
    train_number_classifier_with_generator()