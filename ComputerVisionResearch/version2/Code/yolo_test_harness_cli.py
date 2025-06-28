# yolo_test_harness.py
# A simplified script to test object detection performance of a YOLOv8 .pt model on a video.

# =============================================================================
# --- Imports ---
# =============================================================================
import os
import cv2
import numpy as np
from ultralytics import YOLO
import time
import threading
import queue

# =============================================================================
# --- Configuration ---
# =============================================================================
# --- File Paths ---
# TODO: Set the path to the folder containing your models and videos
MODEL_DIR = 'drive/MyDrive/Colab Notebooks/numbers_detection' 

# TODO: Set the name of the .pt file you want to test
YOLO_MODEL_PATH = os.path.join(MODEL_DIR, 'yolov8m.pt') 

# TODO: Set your input and output video paths
INPUT_VIDEO_PATH = os.path.join(MODEL_DIR, 'video_samples/input/IMG_9086.mp4')
OUTPUT_VIDEO_PATH = os.path.join(MODEL_DIR, 'video_samples/output/yolo_test_output.mp4')

# --- Detection Settings ---
# Confidence threshold for YOLO detections. Detections below this will be ignored.
YOLO_CONF_THRESHOLD = 0.4 

# A list of class IDs that you want to detect. 
# For example, if your model detects 'player' (class 0) and 'ball' (class 1), 
# and you only want to see ball detections, you would set this to [1].
# Set to None to detect all classes in the model.
CLASSES_TO_DETECT = None # Example: [0, 32] for person and sports ball in COCO

# --- Processing Settings ---
PROCESSING_WIDTH = 1280
PROCESSING_HEIGHT = 720

# =============================================================================
# --- Main Processing Thread ---
# =============================================================================
def process_frames(frame_queue, result_queue, yolo_model):
    """
    Worker thread function to perform YOLO detection on frames.
    """
    while True:
        frame = frame_queue.get()
        if frame is None:  # Sentinel value to signal the end
            break

        # Perform YOLO inference
        results = yolo_model.predict(
            frame,
            conf=YOLO_CONF_THRESHOLD,
            classes=CLASSES_TO_DETECT,
            verbose=False # Set to True for detailed console output per frame
        )

        # Get the first result object
        result = results[0]
        
        # Get the class names from the model
        class_names = result.names

        # Iterate through each detected object in the frame
        for box in result.boxes:
            # Get bounding box coordinates, confidence, and class ID
            coords = [int(i) for i in box.xyxy[0]]
            conf = float(box.conf[0])
            cls_id = int(box.cls[0])
            class_name = class_names.get(cls_id, f"Class {cls_id}")

            x1, y1, x2, y2 = coords

            # Draw the bounding box on the frame
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # Create the label text (Class Name + Confidence)
            label = f"{class_name}: {conf:.2f}"

            # Draw a filled rectangle for the label background
            (text_width, text_height), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
            cv2.rectangle(frame, (x1, y1 - text_height - baseline), (x1 + text_width, y1), (0, 255, 0), -1)

            # Put the label text on the background
            cv2.putText(frame, label, (x1, y1 - baseline), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)

        # Put the processed frame in the result queue
        result_queue.put(frame)

# =============================================================================
# --- Main Pipeline ---
# =============================================================================
def main():
    """
    Main function to set up the video processing pipeline.
    """
    print(f"[INFO] Loading YOLO model: {YOLO_MODEL_PATH}")
    if not os.path.exists(YOLO_MODEL_PATH):
        print(f"[ERROR] YOLO model file not found at: {YOLO_MODEL_PATH}")
        return
    yolo_model = YOLO(YOLO_MODEL_PATH)

    print(f"[INFO] Opening video file: {INPUT_VIDEO_PATH}")
    video = cv2.VideoCapture(INPUT_VIDEO_PATH)
    if not video.isOpened():
        print(f"[ERROR] Could not open video file: {INPUT_VIDEO_PATH}")
        return

    # Ensure the output directory exists
    os.makedirs(os.path.dirname(OUTPUT_VIDEO_PATH), exist_ok=True)
    
    # Set up the video writer
    writer = cv2.VideoWriter(
        OUTPUT_VIDEO_PATH,
        cv2.VideoWriter_fourcc(*'mp4v'),
        video.get(cv2.CAP_PROP_FPS),
        (PROCESSING_WIDTH, PROCESSING_HEIGHT)
    )
    
    # Create queues for communication between threads
    frame_queue = queue.Queue(maxsize=8)
    result_queue = queue.Queue()
    
    print("[INFO] Starting processing thread...")
    processing_thread = threading.Thread(
        target=process_frames,
        args=(frame_queue, result_queue, yolo_model)
    )
    processing_thread.start()

    frame_count = 0
    start_time = time.time()
    
    print("[INFO] Reading frames from video...")
    while True:
        ret, frame = video.read()
        if not ret:
            break
        
        # Resize frame and put it into the queue for the worker thread
        resized_frame = cv2.resize(frame, (PROCESSING_WIDTH, PROCESSING_HEIGHT))
        frame_queue.put(resized_frame)
        
        # If there's a processed frame available, write it to the output video
        if not result_queue.empty():
            processed_frame = result_queue.get()
            writer.write(processed_frame)
            frame_count += 1
            if frame_count % 50 == 0:
                print(f"[INFO] Processed {frame_count} frames...")

    print("[INFO] End of video reached. Cleaning up...")
    frame_queue.put(None)  # Signal the processing thread to terminate
    processing_thread.join()  # Wait for the thread to finish

    # Write any remaining frames from the result queue
    while not result_queue.empty():
        writer.write(result_queue.get())
        frame_count += 1
    
    end_time = time.time()
    processing_time = end_time - start_time
    fps = frame_count / processing_time if processing_time > 0 else 0

    print(f"\n[INFO] Processing complete.")
    print(f"Output video saved to: {OUTPUT_VIDEO_PATH}")
    print(f"Total frames processed: {frame_count}, Total time: {processing_time:.2f}s, Estimated FPS: {fps:.2f}")

    # Release resources
    video.release()
    writer.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()