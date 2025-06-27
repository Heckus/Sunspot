# interactive_labeler.py (V3 - 'm' to save manual box)

import os
import cv2
import numpy as np
from ultralytics import YOLO
import argparse
import yaml

# =============================================================================
# --- Global Variables for Manual Drawing ---
# =============================================================================
drawing = False
ix, iy = -1, -1
manual_bbox = ()
WINDOW_NAME = 'Volleyball Labeler'

# =============================================================================
# --- Mouse Callback Function for Drawing Bounding Boxes ---
# =============================================================================
def draw_bounding_box(event, x, y, flags, param):
    """Callback function for drawing a rectangle with the mouse."""
    global ix, iy, drawing, manual_bbox
    frame_to_draw_on = param['frame']

    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        ix, iy = x, y
        manual_bbox = ()

    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing:
            temp_frame = frame_to_draw_on.copy()
            cv2.rectangle(temp_frame, (ix, iy), (x, y), (0, 255, 0), 2)
            param['temp_frame'] = temp_frame

    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False
        x1, y1 = min(ix, x), min(iy, y)
        x2, y2 = max(ix, x), max(iy, y)
        if x2 > x1 and y2 > y1:
            manual_bbox = (x1, y1, x2, y2)
            cv2.rectangle(frame_to_draw_on, manual_bbox[:2], manual_bbox[2:], (0, 255, 0), 2)
            param['temp_frame'] = frame_to_draw_on.copy()

# (Helper & Core Logic Functions remain the same)
def convert_to_yolo_format(bbox, img_width, img_height):
    x1, y1, x2, y2 = bbox
    dw = 1.0 / img_width; dh = 1.0 / img_height
    x_center = (x1 + x2) / 2.0; y_center = (y1 + y2) / 2.0
    width = x2 - x1; height = y2 - y1
    x_center_norm, y_center_norm = x_center * dw, y_center * dh
    width_norm, height_norm = width * dw, height * dh
    return f"0 {x_center_norm:.6f} {y_center_norm:.6f} {width_norm:.6f} {height_norm:.6f}"

def save_data(frame, bbox, base_filename, dirs, target_dims):
    target_width, target_height = target_dims
    orig_height, orig_width, _ = frame.shape
    resized_frame = cv2.resize(frame, (target_width, target_height), interpolation=cv2.INTER_AREA)
    x_ratio, y_ratio = target_width / orig_width, target_height / orig_height
    x1, y1, x2, y2 = bbox
    scaled_bbox = (int(x1 * x_ratio), int(y1 * y_ratio), int(x2 * x_ratio), int(y2 * y_ratio))
    image_path = os.path.join(dirs['images'], f"{base_filename}.jpg")
    label_path = os.path.join(dirs['labels'], f"{base_filename}.txt")
    cv2.imwrite(image_path, resized_frame)
    yolo_label = convert_to_yolo_format(scaled_bbox, target_width, target_height)
    with open(label_path, 'w') as f:
        f.write(yolo_label)
    return True

## --- MODIFIED: Updated instructions
def print_instructions():
    """Prints usage instructions to the console."""
    print("\n--- Volleyball Interactive Labeler ---")
    print("  's' - Save auto-detection")
    print("  'd' - Discard detection / drawing / frame")
    print("  'm' - Enter MANUAL drawing mode. Press 'm' AGAIN to save.")
    print("  'q' - Quit the application")
    print("-------------------------------------\n")

# =============================================================================
# --- Main Function ---
# =============================================================================
def main(args):
    # (Setup is the same)
    TARGET_DIMS = (640, 640); YOLO_MODEL = 'yolov8n.pt'
    print_instructions()
    print("[INFO] Loading YOLOv8 model...")
    yolo_model = YOLO(YOLO_MODEL)
    video_path = args.video; output_dir = args.output_dir
    video_filename = os.path.splitext(os.path.basename(video_path))[0]
    dirs = {'images': os.path.join(output_dir, "images"), 'labels': os.path.join(output_dir, "labels")}
    os.makedirs(dirs['images'], exist_ok=True); os.makedirs(dirs['labels'], exist_ok=True)
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened(): print(f"[ERROR] Could not open video file: {video_path}"); return
    cv2.namedWindow(WINDOW_NAME)
    frame_idx = 0; saved_frame_count = 0
    
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret: break

        yolo_results = yolo_model.predict(frame, conf=args.conf, classes=[32], verbose=False)
        detections = [ [int(i) for i in box.xyxy[0]] for box in yolo_results[0].boxes ]
        display_frame = frame.copy()
        
        if detections:
            final_ball_box = detections[0]
            x1, y1, x2, y2 = final_ball_box
            cv2.rectangle(display_frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
            cv2.putText(display_frame, "Auto: Save (s) or Discard (d)?", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
            cv2.imshow(WINDOW_NAME, display_frame)
            key = cv2.waitKey(0) & 0xFF
            if key == ord('s'):
                base_filename = f"{video_filename}_frame_{frame_idx:06d}"
                if save_data(frame, final_ball_box, base_filename, dirs, TARGET_DIMS):
                    saved_frame_count += 1
                    print(f"Saved auto-detection for frame {frame_idx}")
            elif key == ord('d'): print(f"Discarded auto-detection for frame {frame_idx}")
            elif key == ord('q'): break
        else:
            cv2.putText(display_frame, "No Detection: Skip (d) or Manual (m)?", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.imshow(WINDOW_NAME, display_frame)
            key = cv2.waitKey(0) & 0xFF

            if key == ord('m'):
                global manual_bbox
                manual_bbox = ()
                
                callback_param = {'frame': frame.copy(), 'temp_frame': frame.copy()}
                ## --- MODIFIED: Updated on-screen instructions
                cv2.putText(callback_param['frame'], "Draw box. Then: Save (m) or Discard (d)", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                callback_param['temp_frame'] = callback_param['frame'].copy()

                cv2.setMouseCallback(WINDOW_NAME, draw_bounding_box, callback_param)
                
                while True: 
                    cv2.imshow(WINDOW_NAME, callback_param['temp_frame'])
                    ## --- MODIFIED: The key to break the loop is now 'm' instead of 's'
                    key_manual = cv2.waitKey(1) & 0xFF
                    if key_manual in [ord('m'), ord('d'), ord('q')]:
                        break
                
                ## --- MODIFIED: The key to save is now 'm' instead of 's'
                if key_manual == ord('m') and manual_bbox:
                    base_filename = f"{video_filename}_frame_{frame_idx:06d}"
                    if save_data(frame, manual_bbox, base_filename, dirs, TARGET_DIMS):
                        saved_frame_count += 1
                        print(f"Saved MANUAL label for frame {frame_idx}")
                elif key_manual == ord('q'):
                    break 
                else: # This will catch the 'd' key or 'm' without a box drawn
                    print(f"Discarded manual annotation for frame {frame_idx}")

                cv2.setMouseCallback(WINDOW_NAME, lambda *args: None)
            
            elif key == ord('d'): pass
            elif key == ord('q'): break

        frame_idx += 1

    # (Cleanup is the same)
    print(f"\n[INFO] Exiting. Total images saved: {saved_frame_count}")
    cap.release()
    cv2.destroyAllWindows()
    yaml_path = os.path.join(output_dir, "data.yaml")
    abs_output_dir = os.path.abspath(output_dir)
    yaml_data = {'path': abs_output_dir, 'train': 'images', 'val': 'images', 'names': {0: 'volleyball'}}
    with open(yaml_path, 'w') as f:
        yaml.dump(yaml_data, f, sort_keys=False)
    print(f"[SUCCESS] Dataset and YAML file are in: {output_dir}")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Interactively label volleyballs in a video for a YOLOv8 dataset.")
    parser.add_argument("--video", type=str, required=True, help="Path to the input video file.")
    parser.add_argument("--output-dir", type=str, required=True, help="Directory to save the dataset.")
    parser.add_argument("--conf", type=float, default=0.3, help="Confidence threshold for initial YOLO detection.")
    args = parser.parse_args()
    main(args)