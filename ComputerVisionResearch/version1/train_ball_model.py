# train_ball_detector.py

import os
import tensorflow as tf
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Conv2D, MaxPooling2D, Flatten, Dense, Dropout
from tensorflow.keras.preprocessing.image import ImageDataGenerator
from tensorflow.keras.callbacks import ModelCheckpoint, EarlyStopping

# =============================================================================
# --- Configuration ---
# =============================================================================
# IMPORTANT: Update this path to the root directory of your dataset.
# It should contain the 'train', 'valid', and 'test' subdirectories.
DATASET_BASE_PATH = '/home/hecke/Documents/YijU_459K' 

# Directory to save the trained models
OUTPUT_DIR = '/home/hecke/Github/Sunspot/ComputerVisionResearch/drive/MyDrive/Colab Notebooks/numbers_detection'

# Model and image settings
IMG_WIDTH = 64
IMG_HEIGHT = 64
BATCH_SIZE = 32
EPOCHS = 50 # Increased epochs for potentially better training

# =============================================================================
# --- Custom Callback for Auto-Stopping ---
# =============================================================================
class AutoStop(tf.keras.callbacks.Callback):
    def __init__(self, monitor='val_accuracy', target_accuracy=0.98):
        super(AutoStop, self).__init__()
        self.monitor = monitor
        self.target_accuracy = target_accuracy

    def on_epoch_end(self, epoch, logs=None):
        current_accuracy = logs.get(self.monitor)
        if current_accuracy is not None and current_accuracy >= self.target_accuracy:
            print(f"\nReached {self.target_accuracy*100:.2f}% accuracy, stopping training.")
            self.model.stop_training = True

# =============================================================================
# --- Model Definition ---
# =============================================================================
def create_ball_classifier_model(input_shape=(IMG_HEIGHT, IMG_WIDTH, 3)):
    """
    Creates, compiles, and returns a CNN model for ball classification.
    """
    model = Sequential([
        # First convolutional block
        Conv2D(32, (3, 3), activation='relu', input_shape=input_shape, padding='same'),
        MaxPooling2D(pool_size=(2, 2)),

        # Second convolutional block
        Conv2D(64, (3, 3), activation='relu', padding='same'),
        MaxPooling2D(pool_size=(2, 2)),

        # Third convolutional block
        Conv2D(128, (3, 3), activation='relu', padding='same'),
        MaxPooling2D(pool_size=(2, 2)),

        # Flatten the results to feed into a DNN
        Flatten(),

        # Dense layer with dropout for regularization
        Dense(128, activation='relu'),
        Dropout(0.5),

        # Output layer: 2 units for 'ball' and 'not_ball'
        # Using softmax for multi-class classification
        Dense(2, activation='softmax')
    ])

    # Compile the model
    model.compile(optimizer='adam',
                  loss='categorical_crossentropy',
                  metrics=['accuracy'])
    
    model.summary()
    return model

# =============================================================================
# --- Training Pipeline ---
# =============================================================================
def main():
    """
    Main function to run the training and conversion pipeline.
    """
    if not os.path.exists(DATASET_BASE_PATH):
        print(f"[ERROR] Dataset path not found: {DATASET_BASE_PATH}")
        print("Please update the 'DATASET_BASE_PATH' variable to point to your data.")
        return

    # Create the output directory if it doesn't exist
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    
    # Define paths for the saved models
    best_model_h5_path = os.path.join(OUTPUT_DIR, 'best_ball_detector.h5')
    final_model_tflite_path = os.path.join(OUTPUT_DIR, 'ball_detector.tflite')

    # --- Data Generators ---
    print("\n[INFO] Setting up data generators...")
    # Add data augmentation to the training generator to improve model robustness
    train_datagen = ImageDataGenerator(
        rescale=1./255,
        rotation_range=20,
        width_shift_range=0.2,
        height_shift_range=0.2,
        shear_range=0.2,
        zoom_range=0.2,
        horizontal_flip=True,
        fill_mode='nearest'
    )

    # Validation and test generators only need rescaling
    validation_test_datagen = ImageDataGenerator(rescale=1./255)

    train_generator = train_datagen.flow_from_directory(
        os.path.join(DATASET_BASE_PATH, 'train'),
        target_size=(IMG_WIDTH, IMG_HEIGHT),
        batch_size=BATCH_SIZE,
        class_mode='categorical'
    )

    validation_generator = validation_test_datagen.flow_from_directory(
        os.path.join(DATASET_BASE_PATH, 'valid'),
        target_size=(IMG_WIDTH, IMG_HEIGHT),
        batch_size=BATCH_SIZE,
        class_mode='categorical'
    )

    test_generator = validation_test_datagen.flow_from_directory(
        os.path.join(DATASET_BASE_PATH, 'test'),
        target_size=(IMG_WIDTH, IMG_HEIGHT),
        batch_size=BATCH_SIZE,
        class_mode='categorical',
        shuffle=False # Important for evaluation
    )

    # --- Model Training ---
    print("\n[INFO] Creating and training the model...")
    model = create_ball_classifier_model()

    # Callbacks to save the best model and stop early if validation loss stops improving
    checkpoint = ModelCheckpoint(best_model_h5_path, monitor='val_accuracy', save_best_only=True, mode='max', verbose=1)
    early_stopping = EarlyStopping(monitor='val_loss', patience=10, verbose=1, mode='min', restore_best_weights=True)
    auto_stop_callback = AutoStop(target_accuracy=0.98) # NEW: Stop at 98% accuracy
    
    history = model.fit(
        train_generator,
        steps_per_epoch=train_generator.samples // BATCH_SIZE,
        validation_data=validation_generator,
        validation_steps=validation_generator.samples // BATCH_SIZE,
        epochs=EPOCHS,
        callbacks=[checkpoint, early_stopping, auto_stop_callback] # NEW: Added auto_stop_callback
    )

    print("\n[INFO] Model training complete.")

    # --- Evaluation ---
    print("\n[INFO] Evaluating the model on the test set...")
    # Load the best model saved by the checkpoint
    best_model = tf.keras.models.load_model(best_model_h5_path)
    test_loss, test_acc = best_model.evaluate(test_generator, steps=test_generator.samples // BATCH_SIZE)
    print(f'\nTest accuracy: {test_acc:.4f}')
    print(f'Test loss: {test_loss:.4f}')

    # --- TFLite Conversion ---
    print(f"\n[INFO] Converting the best model to TFLite format...")
    converter = tf.lite.TFLiteConverter.from_keras_model(best_model)
    converter.optimizations = [tf.lite.Optimize.DEFAULT]
    tflite_model = converter.convert()

    with open(final_model_tflite_path, 'wb') as f:
        f.write(tflite_model)

    print(f"[SUCCESS] TFLite model saved to: {final_model_tflite_path}")

# =============================================================================
# --- Execution ---
# =============================================================================
if __name__ == '__main__':
    main()
