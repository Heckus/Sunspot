# -*- coding: utf-8 -*-
"""
cv_test.py

Loads a single image, processes it using the functions defined in
cv_functions.py (which uses config.py), and displays/saves the output.

Usage: python cv_test.py <image_filename>
"""

import cv2
import numpy as np
import os
import argparse
import logging

# --- Import project modules ---
# Ensure config.py and cv_functions.py are in the same directory
# or accessible via PYTHONPATH
try:
    import config
    import cv_functions
except ImportError as e:
    print(f"Error importing project modules: {e}")
    print("Please ensure config.py and cv_functions.py are in the same directory as cv_test.py")
    exit(1)

# --- Argument Parsing ---
parser = argparse.ArgumentParser(description="Test CV functions on a single image.")
parser.add_argument(
    "image_file",
    type=str,
    help="Path to the input image file.",
    # Example default if you want one: default="image_663670.jpg", nargs='?'
)
args = parser.parse_args()

# --- Basic Logging Setup ---
# Configure logging similar to main app if desired, or use basic config
log_level = getattr(logging, config.LOG_LEVEL.upper(), logging.INFO)
logging.basicConfig(level=log_level, format='%(asctime)s - %(levelname)s - %(message)s')

# --- Main Execution ---
if __name__ == "__main__":
    input_image_path = args.image_file
    output_image_path = "output_cv_test.jpg"

    # 1. Check if input file exists
    if not os.path.exists(input_image_path):
        logging.error(f"Input image file not found: {input_image_path}")
        exit(1)

    # 2. Load the image
    logging.info(f"Loading image: {input_image_path}")
    frame_raw = cv2.imread(input_image_path)

    if frame_raw is None:
        logging.error(f"Failed to load image: {input_image_path}")
        exit(1)

    logging.info(f"Image loaded successfully (shape: {frame_raw.shape})")

    # 3. Process the image using the main CV function
    logging.info("Processing image using cv_functions.process_frame...")
    # process_frame uses camera matrix and distortion from config internally
    output_frame = cv_functions.process_frame(frame_raw)

    # 4. Display and Save the output
    if output_frame is not None:
        logging.info(f"Processing complete. Saving output to: {output_image_path}")
        try:
            cv2.imwrite(output_image_path, output_frame)
            logging.info("Output image saved successfully.")
        except Exception as e:
            logging.error(f"Failed to save output image: {e}")

        # Display the image in a window
        try:
            # Resize for display if the image is very large
            display_h, display_w = output_frame.shape[:2]
            max_display_h = 800
            if display_h > max_display_h:
                scale = max_display_h / display_h
                display_w = int(display_w * scale)
                display_h = max_display_h
                display_frame = cv2.resize(output_frame, (display_w, display_h))
            else:
                display_frame = output_frame

            cv2.imshow("CV Test Output", display_frame)
            logging.info("Displaying output image. Press any key to close.")
            cv2.waitKey(0) # Wait indefinitely until a key is pressed
            cv2.destroyAllWindows()
            logging.info("Output window closed.")

        except Exception as e:
            logging.error(f"Failed to display output image: {e}")
            print("\nCould not display image window. Check if you have a GUI environment available.")
            print("Output image should still be saved if processing was successful.")

    else:
        logging.error("CV processing function did not return an output frame.")

    logging.info("cv_test.py finished.")