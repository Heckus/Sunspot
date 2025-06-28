# yolo_test_harness_cli.py
# A flexible script to test object detection of a YOLOv8 .pt model on a video.
# Model and video paths are provided as command-line arguments.

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
import argparse # Added for command-line arguments

# =============================================================================
# --- Configuration ---
# =============================================================================
# --- Detection Settings ---
# Confidence threshold for YOLO detections. Detections below this will be ignored.
YOLO_CONF_THRESHOLD = 0.4 

# A list of class IDs that you want to detect. 
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
            verbose=False
        )
        
        result = results[0]
        class_names = result.names

        # Iterate through each detected object in the frame
        for box in result.boxes:
            coords = [int(i) for i in box.xyxy[0]]
            conf = float(box.conf[0])
            cls_id = int(box.cls[0])
            class_name = class_names.get(cls_id, f"Class {cls_id}")

            x1, y1, x2, y2 = coords
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            label = f"{class_name}: {conf:.2f}"
            (text_width, text_height), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
            cv2.rectangle(frame, (x1, y1 - text_height - baseline), (x1 + text_width, y1), (0, 255, 0), -1)
            cv2.putText(frame, label, (x1, y1 - baseline), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)

        result_queue.put(frame)

# =============================================================================
# --- Main Pipeline ---
# =============================================================================
def main(args):
    """
    Main function to set up the video processing pipeline.
    """
    input_video_path = args.input
    yolo_model_path = args.model
    
    # Determine output video path
    if args.output:
        output_video_path = args.output
    else:
        # Auto-generate output path based on input video name
        base_name, ext = os.path.splitext(os.path.basename(input_video_path))
        output_dir = os.path.dirname(input_video_path)
        output_video_path = os.path.join(output_dir, f"{base_name}_output.mp4")

    print(f"[INFO] Loading YOLO model: {yolo_model_path}")
    if not os.path.exists(yolo_model_path):
        print(f"[ERROR] YOLO model file not found at: {yolo_model_path}")
        return
    yolo_model = YOLO(yolo_model_path)

    print(f"[INFO] Opening video file: {input_video_path}")
    video = cv2.VideoCapture(input_video_path)
    if not video.isOpened():
        print(f"[ERROR] Could not open video file: {input_video_path}")
        return

    os.makedirs(os.path.dirname(output_video_path), exist_ok=True)
    
    writer = cv2.VideoWriter(
        output_video_path,
        cv2.VideoWriter_fourcc(*'mp4v'),
        video.get(cv2.CAP_PROP_FPS),
        (PROCESSING_WIDTH, PROCESSING_HEIGHT)
    )
    
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
        if not ret: break
        
        resized_frame = cv2.resize(frame, (PROCESSING_WIDTH, PROCESSING_HEIGHT))
        frame_queue.put(resized_frame)
        
        if not result_queue.empty():
            processed_frame = result_queue.get()
            writer.write(processed_frame)
            frame_count += 1
            if frame_count % 50 == 0:
                print(f"[INFO] Processed {frame_count} frames...")

    print("[INFO] End of video reached. Cleaning up...")
    frame_queue.put(None)
    processing_thread.join()

    while not result_queue.empty():
        writer.write(result_queue.get())
        frame_count += 1
    
    end_time = time.time()
    processing_time = end_time - start_time
    fps = frame_count / processing_time if processing_time > 0 else 0

    print(f"\n[INFO] Processing complete.")
    print(f"Output video saved to: {output_video_path}")
    print(f"Total frames processed: {frame_count}, Total time: {processing_time:.2f}s, Estimated FPS: {fps:.2f}")

    video.release()
    writer.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    # Set up the argument parser
    parser = argparse.ArgumentParser(description="Process a video with a YOLOv8 model to detect objects.")
    
    # Required arguments
    parser.add_argument("-m", "--model", required=True, help="Path to the YOLOv8 .pt model file.")
    parser.add_argument("-i", "--input", required=True, help="Path to the input video file.")
    
    # Optional argument
    parser.add_argument("-o", "--output", help="Optional. Path to save the output video file. If not provided, it will be saved next to the input file with an '_output' suffix.")
    
    # Parse the command-line arguments
    args = parser.parse_args()
    
    # Run the main function
    main(args)