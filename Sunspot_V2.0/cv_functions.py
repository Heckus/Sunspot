# -*- coding: utf-8 -*-
"""
cv_functions.py

Contains Computer Vision functions for processing camera frames.
Includes placeholders for steps outlined in the 3D reconstruction document.
Applies preprocessing like undistortion based on config.

**Modification 6:** Implemented initial version of detect_court_lines.
**Modification 7:** Modified functions to accept config parameters as arguments
                  instead of importing config directly.
"""

import cv2
import numpy as np
import logging
import time
import math



# --- CV Function Implementations ---

def detect_court_lines(frame_undistorted, camera_matrix, dist_coeffs):
    """
    Detects potential court line segments in the undistorted frame.
    Uses Canny edge detection and Probabilistic Hough Transform, followed by
    basic angle filtering.

    Args:
        frame_undistorted (np.ndarray): The undistorted BGR frame (or potentially scaled/gray).
        camera_matrix (np.ndarray): Camera intrinsic matrix (unused in this version).
        dist_coeffs (np.ndarray): Camera distortion coefficients (unused in this version).

    Returns:
        list: A list of detected line segments [[x1, y1, x2, y2], ...].
              Coordinates correspond to the input frame dimensions. Returns empty list if none found.
    """
    # ... (Internal logic remains the same as previous version) ...
    logging.debug("CV: Detecting court lines...")
    court_lines = []
    try:
        # Convert to gray only if it's not already gray
        if len(frame_undistorted.shape) == 3 and frame_undistorted.shape[2] == 3:
            gray = cv2.cvtColor(frame_undistorted, cv2.COLOR_BGR2GRAY)
        else:
            gray = frame_undistorted # Assume already grayscale if not 3 channels

        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150, apertureSize=3) # Thresholds may need tuning
    except Exception as e:
        logging.error(f"CV: Error during preprocessing/Canny in detect_court_lines: {e}")
        return court_lines

    lines_p = cv2.HoughLinesP(edges, rho=1, theta=np.pi / 180, threshold=50,
                              minLineLength=50, maxLineGap=20) # Params may need tuning

    if lines_p is not None:
        angle_tolerance_horz = np.deg2rad(15)
        min_angle_vert = np.deg2rad(20)
        max_angle_vert = np.deg2rad(80)

        for line in lines_p:
            x1, y1, x2, y2 = line[0]
            angle = math.atan2(y2 - y1, x2 - x1); angle_abs = abs(angle)
            is_horizontal = (angle_abs < angle_tolerance_horz) or (abs(angle_abs - math.pi) < angle_tolerance_horz)
            is_sideline_angle = (min_angle_vert < angle_abs < max_angle_vert) or (min_angle_vert < abs(angle_abs - math.pi) < max_angle_vert)
            if is_horizontal or is_sideline_angle: court_lines.append([x1, y1, x2, y2])

    logging.debug(f"CV: Found {len(court_lines)} potential court line segments after filtering.")
    return court_lines


def detect_players_2d(frame):
    """Placeholder for detecting players and estimating their 2D poses."""
    logging.debug("CV: detect_players_2d (placeholder)")
    return None

def detect_ball_2d(frame):
    """Placeholder for detecting the ball."""
    logging.debug("CV: detect_ball_2d (placeholder)")
    return None

def project_to_3d(detections_2d, entity_type, camera_matrix, dist_coeffs, R_matrix=None, t_vector=None, scale_factor=1.0, ground_plane=None):
    """Placeholder for projecting 2D detections to 3D coordinates."""
    logging.debug(f"CV: project_to_3d ({entity_type}) (placeholder)")
    if detections_2d: return "Dummy_3D_Data"
    return None

def apply_constraints(data_3d, entity_type, ground_plane=None, physics_params=None):
    """Placeholder for applying constraints to refine 3D estimates."""
    logging.debug(f"CV: apply_constraints ({entity_type}) (placeholder)")
    return data_3d


# --- Main Processing Function ---

def process_frame(frame_raw, cv_enabled, camera_matrix, dist_coeffs, scale_factor, apply_grayscale):
    """
    Processes a raw camera frame using provided config parameters.

    Args:
        frame_raw (np.ndarray): The raw BGR frame.
        cv_enabled (bool): Flag to enable/disable processing.
        camera_matrix (np.ndarray): Camera intrinsic matrix.
        dist_coeffs (np.ndarray): Camera distortion coefficients.
        scale_factor (float): Scaling factor for analysis frame (1.0 for no scaling).
        apply_grayscale (bool): Whether to convert analysis frame to grayscale.

    Returns:
        np.ndarray: The frame with CV overlays drawn on it, or None if processing fails/disabled.
    """
    if not cv_enabled: return None # Return None if CV is disabled
    if frame_raw is None: logging.warning("CV: process_frame received None input."); return None

    start_time = time.monotonic()
    output_frame = None

    # --- 1. Preprocessing ---
    try:
        # Use passed camera parameters
        frame_undistorted = cv2.undistort(frame_raw, camera_matrix, dist_coeffs)
        output_frame = frame_undistorted.copy() # Use undistorted frame for drawing
        analysis_frame = frame_undistorted # Use undistorted for analysis
        logging.debug("CV: Frame undistorted.")
    except Exception as e:
        logging.error(f"CV: Failed to undistort frame: {e}")
        output_frame = frame_raw.copy(); analysis_frame = frame_raw.copy() # Fallback

    # Optional Scaling (use passed scale_factor)
    current_scale_factor = scale_factor # Store locally for scaling back calculations
    if current_scale_factor != 1.0 and current_scale_factor > 0:
        try:
            h, w = analysis_frame.shape[:2]
            new_w = int(w * current_scale_factor); new_h = int(h * current_scale_factor)
            analysis_frame = cv2.resize(analysis_frame, (new_w, new_h), interpolation=cv2.INTER_AREA)
            logging.debug(f"CV: Resized analysis frame to {new_w}x{new_h}")
        except Exception as e:
            logging.error(f"CV: Failed to resize frame: {e}"); current_scale_factor = 1.0 # Reset scale factor

    # Optional Grayscale (use passed apply_grayscale)
    if apply_grayscale:
        try:
             analysis_frame = cv2.cvtColor(analysis_frame, cv2.COLOR_BGR2GRAY)
             logging.debug("CV: Converted analysis frame to grayscale.")
        except Exception as e:
             logging.error(f"CV: Failed grayscale conversion: {e}")

    # --- 2. Run CV Detection/Estimation ---
    # Pass camera parameters to functions that need them
    detected_lines = detect_court_lines(analysis_frame, camera_matrix, dist_coeffs)
    detected_players = detect_players_2d(analysis_frame)
    detected_ball = detect_ball_2d(analysis_frame)

    # --- 3. Project to 3D & Apply Constraints (Placeholders) ---
    # Pass camera parameters
    players_3d = project_to_3d(detected_players, 'player', camera_matrix, dist_coeffs)
    ball_3d = project_to_3d(detected_ball, 'ball', camera_matrix, dist_coeffs)
    players_3d_refined = apply_constraints(players_3d, 'player')
    ball_3d_refined = apply_constraints(ball_3d, 'ball')

    # --- 4. Draw Overlays ---
    line_color = (255, 0, 0); player_color = (0, 255, 0); ball_color = (0, 165, 255); text_color = (255, 255, 0)
    if detected_lines:
        logging.debug(f"CV: Drawing {len(detected_lines)} detected lines")
        scale_x = 1.0 / current_scale_factor if current_scale_factor != 1.0 else 1.0 # Calculate inverse scale
        scale_y = 1.0 / current_scale_factor if current_scale_factor != 1.0 else 1.0
        for line in detected_lines:
            x1, y1, x2, y2 = line
            pt1 = (int(x1 * scale_x), int(y1 * scale_y)); pt2 = (int(x2 * scale_x), int(y2 * scale_y))
            try: cv2.line(output_frame, pt1, pt2, line_color, 2)
            except Exception as draw_err: logging.warning(f"CV: Error drawing line {pt1}-{pt2}: {draw_err}")
    # ... (drawing logic for players/ball placeholders unchanged) ...
    if detected_players: cv2.putText(output_frame, "Players (placeholder)", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, text_color, 2)
    if detected_ball: cv2.putText(output_frame, "Ball (placeholder)", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, text_color, 2)
    if players_3d_refined or ball_3d_refined: cv2.putText(output_frame, "3D (placeholder)", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, text_color, 2)

    # --- 5. Performance Info ---
    processing_time = time.monotonic() - start_time
    cv2.putText(output_frame, f"CV Time: {processing_time:.3f}s", (output_frame.shape[1] - 200, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)

    return output_frame