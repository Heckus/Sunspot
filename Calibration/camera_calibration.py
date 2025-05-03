#!/usr/bin/env python3

import cv2
import numpy as np
import time
from picamera2 import Picamera2
from pathlib import Path
import os

# --- Configuration ---
# Chessboard settings
# Number of inner corners per row and column (e.g., 9x6 board has 8x5 inner corners)
CHESSBOARD_DIMS = (10, 7)
# Size of a chessboard square in your chosen units (e.g., millimeters)
# *** MEASURE THIS ACCURATELY ***
SQUARE_SIZE_MM = 20.0

# Image capture settings
NUM_IMAGES_TO_CAPTURE = 20  # Minimum number of images for good calibration
IMAGE_SAVE_DIR = Path("calibration_images") # Directory to save images
IMAGE_FILENAME_PREFIX = "calib_img_"

# Calibration output file
CALIBRATION_FILE = Path("camera_calibration_data.npz")

# --- Helper Functions ---

def create_object_points(chessboard_dims, square_size):
    """
    Creates the 3D coordinates for the chessboard corners in the pattern's local coordinate system.
    """
    objp = np.zeros((chessboard_dims[0] * chessboard_dims[1], 3), np.float32)
    # Create a grid of (x, y, z) points. Z is always 0 as the pattern is planar.
    # Multiply by square_size to get real-world dimensions.
    objp[:, :2] = np.mgrid[0:chessboard_dims[0], 0:chessboard_dims[1]].T.reshape(-1, 2) * square_size
    return objp

def capture_calibration_images(output_dir, num_images, filename_prefix, chessboard_dims):
    """
    Captures images from the Pi camera for calibration. Shows a preview and saves
    images when the spacebar is pressed if corners are detected.
    """
    output_dir.mkdir(parents=True, exist_ok=True) # Create directory if it doesn't exist
    print(f"Created directory: {output_dir.resolve()}")

    picam2 = Picamera2()
    # Configure preview and still capture resolutions
    # Using a lower resolution for preview can be faster
    preview_config = picam2.create_preview_configuration(main={"size": (1024, 768)})
    capture_config = picam2.create_still_configuration() # Use higher resolution for capture
    picam2.configure(preview_config)

    picam2.start()
    print("Camera started. Press SPACE to capture, 'q' to quit.")
    print(f"Need to capture {num_images} valid images.")

    captured_count = 0
    cv2.namedWindow("Camera Feed - Press SPACE to capture, 'q' to quit", cv2.WINDOW_NORMAL)

    try:
        while captured_count < num_images:
            # Capture frame for preview
            buffer = picam2.capture_array()
            # Convert RGBA to BGR for OpenCV display
            frame_bgr = cv2.cvtColor(buffer, cv2.COLOR_RGBA2BGR)

            # --- Check for chessboard corners in the preview frame (optional but helpful) ---
            gray_preview = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
            ret_preview, corners_preview = cv2.findChessboardCorners(gray_preview, chessboard_dims, None)

            # Draw corners on the preview if found
            if ret_preview:
                cv2.drawChessboardCorners(frame_bgr, chessboard_dims, corners_preview, ret_preview)
                print("Chessboard detected in preview.", end='\r')
            else:
                print("Chessboard not detected in preview.", end='\r')

            cv2.imshow("Camera Feed - Press SPACE to capture, 'q' to quit", frame_bgr)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                print("\nQuitting capture phase.")
                break
            elif key == ord(' '):
                print(f"\nAttempting capture {captured_count + 1}/{num_images}...")
                # Switch to higher resolution capture config
                picam2.switch_mode(capture_config)
                # Capture high-resolution image data
                capture_data = picam2.capture_array()
                # Switch back to preview config
                picam2.switch_mode(preview_config)

                # Convert captured image (likely RGBA or RGB) to BGR for OpenCV processing
                if capture_data.shape[2] == 4: # RGBA
                     img_bgr = cv2.cvtColor(capture_data, cv2.COLOR_RGBA2BGR)
                else: # RGB
                     img_bgr = cv2.cvtColor(capture_data, cv2.COLOR_RGB2BGR)

                # Find corners in the high-res captured image
                gray_capture = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
                ret_capture, corners_capture = cv2.findChessboardCorners(gray_capture, chessboard_dims, None)

                if ret_capture:
                    # Refine corner locations
                    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                    corners_refined = cv2.cornerSubPix(gray_capture, corners_capture, (11, 11), (-1, -1), criteria)

                    # Save the image
                    img_filename = output_dir / f"{filename_prefix}{captured_count:02d}.png"
                    cv2.imwrite(str(img_filename), img_bgr)
                    print(f"Image saved: {img_filename}")
                    captured_count += 1

                     # Draw corners on the saved image (optional, for verification)
                    cv2.drawChessboardCorners(img_bgr, chessboard_dims, corners_refined, ret_capture)
                    cv2.imwrite(str(output_dir / f"{filename_prefix}{captured_count-1:02d}_corners.png"), img_bgr)

                else:
                    print("Chessboard not found in captured image. Try a different angle/distance.")

    finally:
        print("\nStopping camera.")
        picam2.stop()
        cv2.destroyAllWindows()
        print(f"Captured {captured_count} images.")
        return captured_count >= num_images


def calibrate_camera(image_dir, chessboard_dims, square_size, calibration_file):
    """
    Performs camera calibration using the captured images.
    """
    print("\n--- Starting Calibration ---")
    image_files = sorted(list(image_dir.glob("*.png"))) # Get image files

    if not image_files:
        print(f"Error: No images found in {image_dir}. Run capture first.")
        return False

    print(f"Found {len(image_files)} images for calibration.")

    # Prepare object points (3D points in real world space)
    objp = create_object_points(chessboard_dims, square_size)

    # Arrays to store object points and image points from all images
    objpoints = []  # 3D points in real world space
    imgpoints = []  # 2D points in image plane

    # Termination criteria for corner refinement
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    processed_count = 0
    successful_detections = 0

    print("Processing images...")
    for fname in image_files:
        img = cv2.imread(str(fname))
        if img is None:
            print(f"Warning: Could not read image {fname}")
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        processed_count += 1

        # Find the chessboard corners
        ret, corners = cv2.findChessboardCorners(gray, chessboard_dims, None)

        # If found, add object points, image points (after refining them)
        if ret:
            successful_detections += 1
            objpoints.append(objp)

            # Refine corner locations
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)

            # --- Optional: Draw and display the corners ---
            cv2.drawChessboardCorners(img, chessboard_dims, corners2, ret)
            cv2.imshow(f'Corners Found - {fname.name}', img)
            cv2.waitKey(500) # Display for 0.5 seconds
            cv2.destroyWindow(f'Corners Found - {fname.name}')
            print(f"  Processed {fname.name} - Corners found.")
        else:
            print(f"  Processed {fname.name} - Corners not found.")


    if successful_detections < 10: # Need at least ~10 good views for reliable calibration
         print(f"Error: Only found corners in {successful_detections} images.")
         print("Need more images with clearly visible chessboard patterns from diverse angles.")
         return False

    print(f"\nFound corners in {successful_detections}/{processed_count} images.")
    print("Calibrating camera...")

    if not imgpoints or not objpoints:
        print("Error: No valid image points or object points collected.")
        return False

    # Get image size from the first successfully processed grayscale image
    if 'gray' not in locals():
        print("Error: Could not determine image size. No images processed successfully?")
        return False
    img_size = (gray.shape[1], gray.shape[0]) # width, height

    # Perform calibration
    # ret: Overall success flag (usually True if calibration runs)
    # mtx: Camera intrinsic matrix (K)
    # dist: Distortion coefficients (k1, k2, p1, p2, k3)
    # rvecs: Rotation vectors for each image
    # tvecs: Translation vectors for each image
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, img_size, None, None)

    if not ret:
        print("Error: Camera calibration failed.")
        return False

    print("\n--- Calibration Results ---")
    print("Camera Matrix (Intrinsics):\n", mtx)
    print("\nDistortion Coefficients:\n", dist)
    # print("\nRotation Vectors (rvecs):\n", rvecs) # Optional: Per-image rotation
    # print("\nTranslation Vectors (tvecs):\n", tvecs) # Optional: Per-image translation

    # Calculate reprojection error (a measure of calibration accuracy)
    mean_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        mean_error += error

    reprojection_error = mean_error / len(objpoints)
    print(f"\nMean Reprojection Error: {reprojection_error:.4f} pixels")
    # Lower values (ideally < 1.0 pixels) indicate better calibration.

    # Save calibration data
    print(f"\nSaving calibration data to: {calibration_file}")
    np.savez(calibration_file, camera_matrix=mtx, dist_coeffs=dist, rvecs=rvecs, tvecs=tvecs, reprojection_error=reprojection_error)
    print("Calibration data saved successfully.")

    return True

# --- Main Execution ---
if __name__ == "__main__":
    print("Starting Raspberry Pi Camera Calibration Script")
    print("-" * 30)
    print(f"Chessboard Dimensions (inner corners): {CHESSBOARD_DIMS}")
    print(f"Square Size: {SQUARE_SIZE_MM} mm")
    print(f"Number of images to capture: {NUM_IMAGES_TO_CAPTURE}")
    print(f"Images will be saved in: ./{IMAGE_SAVE_DIR}/")
    print(f"Calibration data will be saved to: ./{CALIBRATION_FILE}")
    print("-" * 30)

    # --- Step 1: Capture Images ---
    # Check if enough images already exist
    existing_images = list(IMAGE_SAVE_DIR.glob(f"{IMAGE_FILENAME_PREFIX}*.png"))
    if len(existing_images) >= NUM_IMAGES_TO_CAPTURE:
        print(f"Found {len(existing_images)} existing images in {IMAGE_SAVE_DIR}.")
        capture_choice = input("Do you want to re-capture images? (y/N): ").lower()
        if capture_choice == 'y':
             # Optional: Clear existing images if re-capturing
             # for f in existing_images:
             #     os.remove(f)
             # print("Removed existing images.")
             capture_success = capture_calibration_images(IMAGE_SAVE_DIR, NUM_IMAGES_TO_CAPTURE, IMAGE_FILENAME_PREFIX, CHESSBOARD_DIMS)
        else:
             print("Skipping image capture.")
             capture_success = True # Assume existing images are sufficient
    else:
        print("Starting image capture phase...")
        capture_success = capture_calibration_images(IMAGE_SAVE_DIR, NUM_IMAGES_TO_CAPTURE, IMAGE_FILENAME_PREFIX, CHESSBOARD_DIMS)

    if not capture_success:
        print("\nImage capture did not complete successfully or was aborted.")
        print("Cannot proceed to calibration without sufficient images.")
    else:
        # --- Step 2: Calibrate ---
        if calibrate_camera(IMAGE_SAVE_DIR, CHESSBOARD_DIMS, SQUARE_SIZE_MM, CALIBRATION_FILE):
             print("\nCalibration finished successfully!")
        else:
             print("\nCalibration failed.")

    print("\nScript finished.")




'''
You can load these values in other Python scripts using numpy:
Python

import numpy as np

calibration_data = np.load("camera_calibration_data.npz")
camera_matrix = calibration_data['camera_matrix']
dist_coeffs = calibration_data['dist_coeffs']

print("Loaded Camera Matrix:\n", camera_matrix)
print("\nLoaded Distortion Coefficients:\n", dist_coeffs
'''