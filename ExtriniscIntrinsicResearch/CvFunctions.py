# cv_functions.py

import cv2
import numpy as np
import logging
import Config
from ultralytics import YOLO

# --- Global YOLO model ---
# We initialize the model once and reuse it to avoid loading it on every frame.
yolo_model = None

def initialize_yolo_model():
    """Loads the YOLO model from the path specified in Config."""
    global yolo_model
    try:
        if Config.YOLO_MODEL_PATH and yolo_model is None:
            logging.info(f"Loading YOLO model from: {Config.YOLO_MODEL_PATH}")
            yolo_model = YOLO(Config.YOLO_MODEL_PATH)
            logging.info("YOLO model loaded successfully.")
    except Exception as e:
        logging.error(f"Failed to load YOLO model: {e}", exc_info=True)
        yolo_model = None

def detect_volleyball_yolo(frame):
    """
    Detects a volleyball in a frame using a YOLO model.
    This function replaces the color-based detection.
    """
    global yolo_model
    if yolo_model is None:
        logging.warning("CV: YOLO model is not initialized. Cannot perform detection.")
        return None, None, None, frame # Return the original frame as the debug frame

    try:
        # Perform inference
        results = yolo_model(frame, verbose=False) # verbose=False to reduce console spam
        
        # We assume the first result object contains the detections for our single image
        result = results[0]

        # Filter detections by confidence threshold
        confident_detections = [
            box for box in result.boxes 
            if box.conf[0] > Config.YOLO_CONFIDENCE_THRESHOLD
        ]

        if not confident_detections:
            return None, None, None, frame

        # If multiple balls are detected, choose the one with the highest confidence
        best_detection = max(confident_detections, key=lambda box: box.conf[0])
        
        # --- Convert Bounding Box to Center and Radius ---
        box_coords = best_detection.xyxy[0].cpu().numpy()
        x1, y1, x2, y2 = box_coords
        
        # Calculate center
        u = int((x1 + x2) / 2)
        v = int((y1 + y2) / 2)
        
        # Approximate radius from the bounding box
        # We average the width and height to get a more stable radius value
        box_width = x2 - x1
        box_height = y2 - y1
        radius_px = int((box_width + box_height) / 4)

        logging.debug(f"CV (YOLO): Ball detected at ({u}, {v}), radius ~{radius_px}px, conf: {best_detection.conf[0]:.2f}")

        # --- Create a debug frame for visualization ---
        debug_frame = frame.copy()
        # Draw bounding box
        cv2.rectangle(debug_frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
        # Draw center circle
        cv2.circle(debug_frame, (u, v), 5, (0, 0, 255), -1)
        # Add label
        label = f"Ball: {best_detection.conf[0]:.2f}"
        cv2.putText(debug_frame, label, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        return u, v, radius_px, debug_frame

    except Exception as e:
        logging.error(f"CV: Error during YOLO detection: {e}", exc_info=True)
        return None, None, None, frame


# --- Helper functions (undistort, 3D estimation, drawing) remain the same ---

def undistort_frame(frame, mtx, dist, new_camera_mtx):
    if mtx is None or dist is None:
        return frame
    try:
        return cv2.undistort(frame, mtx, dist, None, new_camera_mtx if new_camera_mtx is not None else mtx)
    except Exception as e:
        logging.error(f"CV: Error during undistortion: {e}", exc_info=True)
        return frame

def estimate_3d_position_from_radius(u, v, radius_px, mtx, dist, rvec, tvec):
    if u is None or v is None or radius_px is None or radius_px <= 1:
        return None
    if mtx is None or dist is None or rvec is None or tvec is None:
        return None
    try:
        fx = mtx[0, 0]
        fy = mtx[1, 1]
        focal_length_px = (fx + fy) / 2.0
        distance_cam_z = (Config.VOLLEYBALL_RADIUS_M * focal_length_px) / radius_px
        undistorted_point = cv2.undistortPoints(np.array([[[float(u), float(v)]]], dtype=np.float32), mtx, dist, P=mtx)
        u_undistorted, v_undistorted = undistorted_point[0,0,0], undistorted_point[0,0,1]
        cx = mtx[0, 2]
        cy = mtx[1, 2]
        cam_x = (u_undistorted - cx) / fx * distance_cam_z
        cam_y = (v_undistorted - cy) / fy * distance_cam_z
        point_in_cam_coords = np.array([cam_x, cam_y, distance_cam_z])
        R, _ = cv2.Rodrigues(rvec)
        R_transpose = R.T
        point_in_cam_coords_col = point_in_cam_coords.reshape(3, 1)
        tvec_col = tvec.reshape(3, 1)
        world_coords = R_transpose @ (point_in_cam_coords_col - tvec_col)
        return world_coords.flatten()
    except Exception as e:
        logging.error(f"CV: Error during 3D position estimation: {e}", exc_info=True)
        return None

def draw_3d_info_on_frame(frame, world_coords_3d):
    if frame is None or world_coords_3d is None: return frame
    text_x = f"X: {world_coords_3d[0]:.2f}m"
    text_y = f"Y: {world_coords_3d[1]:.2f}m"
    text_z = f"Z: {world_coords_3d[2]:.2f}m"
    cv2.putText(frame, text_x, (10, frame.shape[0] - 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
    cv2.putText(frame, text_y, (10, frame.shape[0] - 45), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
    cv2.putText(frame, text_z, (10, frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
    return frame

def draw_axes_on_frame(frame, rvec, tvec, mtx, dist, axis_length_m=0.5):
    if frame is None or rvec is None or tvec is None or mtx is None or dist is None: return frame
    axis_points_3d = np.float32([[0,0,0], [axis_length_m,0,0], [0,axis_length_m,0], [0,0,axis_length_m]]).reshape(-1,3)
    try:
        image_points, _ = cv2.projectPoints(axis_points_3d, rvec, tvec, mtx, dist)
        image_points = np.int32(image_points.reshape(-1, 2))
        origin, x_end, y_end, z_end = image_points
        cv2.line(frame, tuple(origin), tuple(x_end), (255,0,0), 2) # X=Blue
        cv2.line(frame, tuple(origin), tuple(y_end), (0,255,0), 2) # Y=Green
        cv2.line(frame, tuple(origin), tuple(z_end), (0,0,255), 2) # Z=Red
    except Exception as e:
        logging.error(f"CV: Error drawing axes: {e}", exc_info=True)
    return frame
