# train_models_multidigit.py

import os
import pickle
import numpy as np
import matplotlib.pyplot as plt
import tensorflow as tf
from tensorflow.keras.applications import VGG16
from tensorflow.keras.layers import Flatten, Dense, Input
from tensorflow.keras.models import Model
from tensorflow.keras.optimizers import Adam
from tensorflow.keras.preprocessing.image import ImageDataGenerator
from keras.models import Sequential
from keras.layers import Conv2D, MaxPooling2D, Dropout

# =============================================================================
# Configuration
# =============================================================================
DRIVE_PATH = 'drive/MyDrive/Colab Notebooks/numbers_detection'
BBOX_DATA_PATH = os.path.join(DRIVE_PATH, 'data_bbox.pickle')
CLASSIFICATION_DATA_PATH = os.path.join(DRIVE_PATH, 'data.pickle')

NUMBER_DETECTION_MODEL_PATH = os.path.join(DRIVE_PATH, 'number_detection.h5')
NUMBER_CLASSIFICATION_MODEL_PATH = os.path.join(DRIVE_PATH, 'numbers_classifier_multidigit.h5') # New name

# =============================================================================
# Model 1: Number Bounding Box Detection (No changes needed here)
# =============================================================================
def train_number_bbox_detector():
    """Trains the model to detect the bounding box of the jersey number."""
    # This function remains the same as in the previous script.
    # It is independent of the number of digits.
    # (Code omitted for brevity, but you would include the full function from the previous step)
    print("[INFO] BBox detector training is unchanged. Skipping if already trained.")
    pass

# =============================================================================
# Model 2: Number Classification (Updated for 100 classes)
# =============================================================================
def train_number_classifier_multidigit():
    """
    Trains the model to classify the jersey number (0-99).
    """
    print("\n[INFO] Training the multi-digit number classifier...")

    print("[INFO] Loading multi-digit classification data...")
    with open(CLASSIFICATION_DATA_PATH, 'rb') as handle:
        (X_train, y_train, X_test, y_test) = pickle.load(handle)
        
    # --- IMPORTANT: Verify the number of classes in your data ---
    # y_train should have a shape of (num_samples, 100)
    if y_train.shape[1] != 100:
        print(f"[ERROR] Loaded classification data has {y_train.shape[1]} classes, but 100 are expected.")
        print("Please ensure prepare_dataset_multidigit.py was run correctly.")
        return

    print("[INFO] Building model architecture for 100 classes...")
    classifier = Sequential()
    # (Convolutional base layers remain the same)
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
    classifier.add(Flatten())
    classifier.add(Dense(units=128, activation='relu'))
    classifier.add(Dense(units=64, activation='relu'))
    classifier.add(Dense(units=64, activation='relu'))
    
    # --- NEW: Final layer now has 100 units for classes 0-99 ---
    classifier.add(Dense(units=100, activation='softmax'))

    print("[INFO] Compiling model...")
    classifier.compile(optimizer='adam', loss='categorical_crossentropy', metrics=['accuracy'])
    print(classifier.summary())

    datagen = ImageDataGenerator(rotation_range=30, shear_range=0.5, width_shift_range=0.2, height_shift_range=0.2)
    datagen.fit(X_train)

    epochs = 80
    batch_size = 32

    print("[INFO] Training multi-digit classifier...")
    H = classifier.fit(
        datagen.flow(X_train, y_train, batch_size=batch_size),
        steps_per_epoch=len(X_train) // batch_size,
        validation_data=(X_test, y_test),
        epochs=epochs
    )

    print(f"[INFO] Saving model to {NUMBER_CLASSIFICATION_MODEL_PATH}...")
    classifier.save(NUMBER_CLASSIFICATION_MODEL_PATH)
    print("[INFO] Multi-digit classifier training complete.")

# =============================================================================
# Main Execution
# =============================================================================
if __name__ == '__main__':
    # train_number_bbox_detector() # You can run this if needed
    train_number_classifier_multidigit()