# -*- coding: utf-8 -*-
"""
cv_test.py

Loads a single image, processes it using the functions defined in
cv_functions.py (using locally defined config values), and displays/saves the output.

Usage: python cv_test.py <image_filename>
"""

import cv2
import numpy as np
import os
import argparse
import logging

# --- Import the modified project module ---
try:
    import cv_functions
except ImportError as e:
    print(f"Error importing project module: {e}")
    print("Please ensure cv_functions.py is in the same directory as cv_test.py")
    exit(1)

# --- Define Configuration Variables Locally (Copied from config.py) ---
# General
LOG_LEVEL = "INFO" # Or "DEBUG" for more verbose output from cv_functions

# CV Settings
CV_PROCESSING_ENABLED = True # Set to True to enable processing
CV_INPUT_SCALING_FACTOR = 1.0 # Process at full resolution for testing initially
CV_APPLY_GRAYSCALE = False # Set to True to test grayscale processing

# Camera Intrinsics (Replace with your actual calibration data)
CAMERA_INTRINSIC_MATRIX = np.array([
    [1000.0,    0.0, 960.0], # Example fx, cx for 1920 width
    [   0.0, 1000.0, 540.0], # Example fy, cy for 1080 height
    [   0.0,    0.0,   1.0]
])
CAMERA_DISTORTION_COEFFS = np.array([0.0, 0.0, 0.0, 0.0, 0.0]) # Example: [k1, k2, p1, p2, k3]

# --- Argument Parsing ---
parser = argparse.ArgumentParser(description="Test CV functions on a single image.")
parser.add_argument(
    "image_file",
    type=str,
    help="Path to the input image file."
)
args = parser.parse_args()

# --- Basic Logging Setup ---
log_level_setting = getattr(logging, LOG_LEVEL.upper(), logging.INFO)
logging.basicConfig(level=log_level_setting, format='%(asctime)s - %(levelname)s - [%(filename)s] - %(message)s')

# --- Main Execution ---
if __name__ == "__main__":
    input_image_path = args.image_file
    output_image_path = "output_cv_test.jpg"

    if not os.path.exists(input_image_path):
        logging.error(f"Input image file not found: {input_image_path}")
        exit(1)

    logging.info(f"Loading image: {input_image_path}")
    frame_raw = cv2.imread(input_image_path)

    if frame_raw is None:
        logging.error(f"Failed to load image: {input_image_path}")
        exit(1)

    logging.info(f"Image loaded successfully (shape: {frame_raw.shape})")

    # --- Call process_frame with locally defined config values ---
    logging.info("Processing image using cv_functions.process_frame...")
    output_frame = cv_functions.process_frame(
        frame_raw=frame_raw,
        cv_enabled=CV_PROCESSING_ENABLED,
        camera_matrix=CAMERA_INTRINSIC_MATRIX,
        dist_coeffs=CAMERA_DISTORTION_COEFFS,
        scale_factor=CV_INPUT_SCALING_FACTOR,
        apply_grayscale=CV_APPLY_GRAYSCALE
    )

    # --- Display and Save ---
    if output_frame is not None:
        logging.info(f"Processing complete. Saving output to: {output_image_path}")
        try:
            cv2.imwrite(output_image_path, output_frame)
            logging.info("Output image saved successfully.")
        except Exception as e:
            logging.error(f"Failed to save output image: {e}")

        try:
            display_h, display_w = output_frame.shape[:2]
            max_display_h = 800
            if display_h > max_display_h:
                scale = max_display_h / display_h
                display_w = int(display_w * scale); display_h = max_display_h
                display_frame = cv2.resize(output_frame, (display_w, display_h))
            else:
                display_frame = output_frame
            cv2.imshow("CV Test Output", display_frame)
            logging.info("Displaying output image. Press any key to close.")
            cv2.waitKey(0); cv2.destroyAllWindows()
            logging.info("Output window closed.")
        except Exception as e:
            logging.error(f"Failed to display output image: {e}")
            print("\nCould not display image window. Check GUI environment.")
            print("Output image should still be saved if processing was successful.")
    else:
        logging.error("CV processing function did not return an output frame (or CV is disabled).")

    logging.info("cv_test.py finished.")