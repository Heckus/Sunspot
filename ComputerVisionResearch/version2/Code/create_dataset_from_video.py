import os
import cv2
import numpy as np
from ultralytics import YOLO
import argparse
import yaml
import math

# =============================================================================
# --- Assistant Detector for MULTIPLE BALLS ---
# =============================================================================
class ColorBlobDetector:
    def __init__(self):
        self.backSub = cv2.createBackgroundSubtractorMOG2(history=500, varThreshold=50, detectShadows=False)
        self.lower_yellow = np.array([18, 100, 100])
        self.upper_yellow = np.array([40, 255, 255])
        self.frame_count = 0
        self.warmup_frames = 50

    def detect(self, frame):
        self.frame_count += 1
        fgMask = self.backSub.apply(frame)
        if self.frame_count < self.warmup_frames:
            return []

        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        colorMask = cv2.inRange(hsv_frame, self.lower_yellow, self.upper_yellow)
        combinedMask = cv2.bitwise_and(fgMask, colorMask)
        kernel = np.ones((11, 11), np.uint8)
        closed_mask = cv2.morphologyEx(combinedMask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(closed_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return []

        detected_boxes = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < 100 or area > 20000:
                continue
            x, y, w, h = cv2.boundingRect(c)
            aspect_ratio = w / float(h)
            if aspect_ratio < 0.7 or aspect_ratio > 1.3:
                continue
            hull = cv2.convexHull(c)
            solidity = area / float(cv2.contourArea(hull))
            if solidity < 0.85:
                continue
            detected_boxes.append((x, y, x + w, y + h))
        return detected_boxes

# =============================================================================
# --- Global Variables & Multi-Box Mouse Callback ---
# =============================================================================
drawing = False
ix, iy = -1, -1
WINDOW_NAME = 'Volleyball Labeler'

def multi_box_editor_callback(event, x, y, flags, param):
    global ix, iy, drawing
    boxes = param['boxes']
    frame_copy = param['frame'].copy()
    # --- FIX 1: Get source_map from the callback parameters ---
    source_map = param['source_map']

    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        ix, iy = x, y

    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing:
            # --- FIX 2: Pass source_map to the drawing function ---
            draw_all_boxes(frame_copy, boxes, source_map)
            cv2.rectangle(frame_copy, (ix, iy), (x, y), (0, 255, 0), 2)
            cv2.imshow(WINDOW_NAME, frame_copy)

    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False
        x1, y1, x2, y2 = min(ix, x), min(iy, y), max(ix, x), max(iy, y)
        if x2 > x1 and y2 > y1:
            boxes.append((x1, y1, x2, y2))

    elif event == cv2.EVENT_RBUTTONDOWN:
        if boxes:
            min_dist, box_to_delete_idx = float('inf'), -1
            for i, box in enumerate(boxes):
                center_x, center_y = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
                dist = math.sqrt((center_x - x)**2 + (center_y - y)**2)
                if dist < min_dist:
                    min_dist, box_to_delete_idx = dist, i
            
            if box_to_delete_idx != -1:
                box_width = boxes[box_to_delete_idx][2] - boxes[box_to_delete_idx][0]
                if min_dist < box_width:
                    boxes.pop(box_to_delete_idx)
                    # Since a manual box has no source, we don't need to pop from source_map
                    # The main loop will redraw with the correct colors

# =============================================================================
# --- Core Functions (Drawing, Saving, etc.) ---
# =============================================================================
def draw_all_boxes(frame, boxes, source_map):
    # This function now correctly receives source_map
    for i, box in enumerate(boxes):
        # Manually drawn boxes won't be in the source_map, so they get a default color
        color = source_map.get(i, (0, 255, 0)) # Default to green for manual
        cv2.rectangle(frame, (box[0], box[1]), (box[2], box[3]), color, 2)

def convert_to_yolo_format(bbox, img_width, img_height):
    x1, y1, x2, y2 = bbox
    dw, dh = 1.0 / img_width, 1.0 / img_height
    x_center, y_center = (x1 + x2) / 2.0, (y1 + y2) / 2.0
    width, height = x2 - x1, y2 - y1
    return f"0 {x_center*dw:.6f} {y_center*dh:.6f} {width*dw:.6f} {height*dh:.6f}"

def save_labels_and_image(frame, bboxes, base_filename, dirs, target_dims):
    target_width, target_height = target_dims
    resized_frame = cv2.resize(frame, (target_width, target_height), interpolation=cv2.INTER_AREA)
    image_path = os.path.join(dirs['images'], f"{base_filename}.jpg")
    cv2.imwrite(image_path, resized_frame)
    
    label_path = os.path.join(dirs['labels'], f"{base_filename}.txt")
    if not bboxes:
        open(label_path, 'w').close()
        return True

    orig_height, orig_width, _ = frame.shape
    yolo_labels = []
    # Note: We now scale inside this function, assuming bboxes are for the original frame.
    for bbox in bboxes:
        x_ratio, y_ratio = target_width / orig_width, target_height / orig_height
        x1, y1, x2, y2 = bbox
        # It's better practice to convert to YOLO format using target dimensions directly
        yolo_labels.append(convert_to_yolo_format(bbox, orig_width, orig_height))

    with open(label_path, 'w') as f:
        f.write("\n".join(yolo_labels))
    return True

def print_instructions():
    print("\n--- Volleyball Interactive Labeler (Multi-Ball Edition) ---")
    # ... (instructions remain the same)
    print("  'q' - Quit the application")
    print("\n--- Review & Edit Mode ---")
    print("  's' - Save all current boxes and advance")
    print("  'd' - Discard frame and advance")
    print("  'c' - Clear all boxes to draw manually")
    print("  Left-Click & Drag - Add a new box")
    print("  Right-Click       - Delete the nearest box")
    print("\n--- Box Colors ---")
    print("  BLUE   - YOLO Detection")
    print("  YELLOW - Color Detection Assistant")
    print("  GREEN  - Manually Drawn Box")
    print("----------------------------------------------------------\n")

# =============================================================================
# --- Main Function ---
# =============================================================================
def main(args):
    TARGET_DIMS = (640, 640); YOLO_MODEL = "Code/modeln_ballV1.pt"
    print_instructions()
    yolo_model = YOLO(YOLO_MODEL)
    video_path, output_dir = args.video, args.output_dir
    video_filename = os.path.splitext(os.path.basename(video_path))[0]
    dirs = {'images': os.path.join(output_dir, "images"), 'labels': os.path.join(output_dir, "labels")}
    os.makedirs(dirs['images'], exist_ok=True); os.makedirs(dirs['labels'], exist_ok=True)
    
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened(): print(f"[ERROR] Could not open video file: {video_path}"); return
    
    frame_idx, saved_frame_count = 0, 0
    assistant_detector = ColorBlobDetector()

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret: break
        frame = cv2.rotate(frame,cv2.ROTATE_180)
        yolo_results = yolo_model.predict(frame, conf=args.conf, classes=[0], verbose=False)
        current_boxes = [ [int(i) for i in box.xyxy[0]] for box in yolo_results[0].boxes ]
        source_map = {i: (255, 100, 0) for i in range(len(current_boxes))} # Blue for YOLO

        if not current_boxes:
            assistant_boxes = assistant_detector.detect(frame)
            if assistant_boxes:
                current_boxes = assistant_boxes
                source_map = {i: (0, 255, 255) for i in range(len(current_boxes))} # Yellow

        cv2.namedWindow(WINDOW_NAME)
        # --- FIX 3: Add source_map to the dictionary passed to the callback ---
        callback_param = {'boxes': current_boxes, 'frame': frame, 'source_map': source_map}
        cv2.setMouseCallback(WINDOW_NAME, multi_box_editor_callback, callback_param)

        while True:
            frame_copy = frame.copy()
            # The main loop's draw call always has access to the correct source_map
            draw_all_boxes(frame_copy, current_boxes, source_map)
            
            info_text = "'s' Save | 'd' Discard | 'c' Clear | 'q' Quit"
            cv2.putText(frame_copy, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2, cv2.LINE_AA)
            cv2.imshow(WINDOW_NAME, frame_copy)

            key = cv2.waitKey(20) & 0xFF
            
            if key == ord('q'):
                cap.release()
                break
            elif key == ord('s'):
                base_filename = f"{video_filename}_frame_{frame_idx:06d}"
                # The save function now correctly takes the original frame and its boxes
                if save_labels_and_image(frame, current_boxes, base_filename, dirs, TARGET_DIMS):
                    saved_frame_count += 1
                    print(f"Saved {len(current_boxes)} boxes for frame {frame_idx}")
                break
            elif key == ord('d'):
                print(f"Discarded frame {frame_idx}")
                break
            elif key == ord('c'):
                current_boxes.clear()
                source_map.clear()
        
        if not cap.isOpened(): break
        frame_idx += 1
    
    cv2.destroyAllWindows()
    print(f"\n[INFO] Exiting. Total images saved: {saved_frame_count}")
    yaml_path = os.path.join(output_dir, "data.yaml")
    abs_output_dir = os.path.abspath(output_dir)
    yaml_data = {'path': abs_output_dir, 'train': 'images', 'val': 'images', 'names': {0: 'volleyball'}}
    with open(yaml_path, 'w') as f:
        yaml.dump(yaml_data, f, sort_keys=False)
    print(f"[SUCCESS] Dataset and YAML file are in: {output_dir}")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Interactively label multiple volleyballs in a video.")
    parser.add_argument("--video", type=str, required=True, help="Path to the input video file.")
    parser.add_argument("--output-dir", type=str, required=True, help="Directory to save the dataset.")
    parser.add_argument("--conf", type=float, default=0.4, help="Confidence threshold for initial YOLO detection.")
    args = parser.parse_args()
    main(args)