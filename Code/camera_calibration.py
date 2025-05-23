#!/usr/bin/env python3

import cv2
import numpy as np
import time
from picamera2 import Picamera2
from pathlib import Path
import os
import traceback

# --- Configuration ---
# Chessboard settings
# Number of inner corners per row and column (e.g., 10x7 board has 9x6 inner corners)
CHESSBOARD_DIMS = (10, 7)  # IMPORTANT: Adjust to your checkerboard (squares_across-1, squares_down-1)
# Size of a chessboard square in your chosen units (e.g., millimeters)
# *** MEASURE THIS ACCURATELY ***
SQUARE_SIZE_MM = 20.0

# Image capture settings
NUM_IMAGES_TO_CAPTURE = 40  # Minimum number of images for good calibration
IMAGE_SAVE_DIR = Path("calibration_images") # Directory to save images
IMAGE_FILENAME_PREFIX = "calib_img_"
FAILED_IMAGE_FILENAME_PREFIX = "failed_detect_"

# Calibration output file
CALIBRATION_FILE = Path("camera_calibration_data.npz")

# --- Helper Functions ---

def get_timestamp():
    return time.strftime('%Y-%m-%d %H:%M:%S')

def create_object_points(chessboard_dims, square_size):
    objp = np.zeros((chessboard_dims[0] * chessboard_dims[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:chessboard_dims[0], 0:chessboard_dims[1]].T.reshape(-1, 2) * square_size
    return objp

def capture_calibration_images(output_dir, num_images, filename_prefix, chessboard_dims):
    output_dir.mkdir(parents=True, exist_ok=True)
    print(f"[{get_timestamp()}] Created image directory: {output_dir.resolve()}")

    picam2 = None
    try:
        picam2 = Picamera2()
        # Configure preview (lower resolution for speed)
        preview_config = picam2.create_preview_configuration(main={"size": (1024, 768)})
        
        # Configure capture (higher resolution for calibration)
        # Explicitly set a high resolution. 2028x1520 can be demanding.
        # Try (1920,1080) for FullHD if 2028x1520 is unstable, or comment out main={"size":...}
        # to let Picamera2 pick its default still resolution (which might be 2028x1520 or higher).
        # capture_config = picam2.create_still_configuration(main={"size": (2028, 1520)})
        capture_config = picam2.create_still_configuration(main={"size": (1920, 1080)}) # More conservative high-res
        # capture_config = picam2.create_still_configuration() # Let Picamera2 choose

        print(f"[{get_timestamp()}] Preview config: {preview_config.get('main', {})}")
        print(f"[{get_timestamp()}] Capture config: {capture_config.get('main', {})}")

        picam2.configure(preview_config)
        picam2.start()
        print(f"[{get_timestamp()}] Camera started. Press SPACE to capture, 'q' to quit.")
        print(f"Need to capture {num_images} valid images.")

        captured_count = 0
        cv2.namedWindow("Camera Feed - Press SPACE to capture, 'q' to quit", cv2.WINDOW_NORMAL)

        while captured_count < num_images:
            # Capture frame for preview
            buffer = picam2.capture_array()
            frame_bgr = cv2.cvtColor(buffer, cv2.COLOR_RGBA2BGR)

            # --- Check for chessboard corners in the preview frame (optional but helpful) ---
            gray_preview = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
            # Use FAST_CHECK for preview to keep it responsive
            ret_preview, corners_preview = cv2.findChessboardCorners(gray_preview, chessboard_dims, cv2.CALIB_CB_FAST_CHECK)

            if ret_preview:
                cv2.drawChessboardCorners(frame_bgr, chessboard_dims, corners_preview, ret_preview)
            
            cv2.imshow("Camera Feed - Press SPACE to capture, 'q' to quit", frame_bgr)
            key = cv2.waitKey(1) & 0xFF # Process GUI events

            if key == ord('q'):
                print(f"\n[{get_timestamp()}] Quitting capture phase by user request.")
                break
            elif key == ord(' '):
                print(f"\n[{get_timestamp()}] Spacebar pressed. Attempting capture {captured_count + 1}/{num_images}...")
                
                print(f"[{get_timestamp()}] Switching to capture_config...")
                picam2.switch_mode(capture_config)
                print(f"[{get_timestamp()}] Switched. Actual main config: {picam2.camera_configuration().get('main', {})}")

                print(f"[{get_timestamp()}] Capturing high-resolution array...")
                capture_data = picam2.capture_array()
                print(f"[{get_timestamp()}] Captured array. Shape: {capture_data.shape if capture_data is not None else 'None'}.")

                print(f"[{get_timestamp()}] Switching back to preview_config...")
                picam2.switch_mode(preview_config)
                print(f"[{get_timestamp()}] Switched back to preview_config.")

                if capture_data is None:
                    print(f"[{get_timestamp()}] ERROR: Captured data is None. Skipping this attempt.")
                    continue

                # Convert captured image to BGR for OpenCV processing
                if capture_data.shape[2] == 4: # RGBA
                     img_bgr_capture = cv2.cvtColor(capture_data, cv2.COLOR_RGBA2BGR)
                else: # RGB
                     img_bgr_capture = cv2.cvtColor(capture_data, cv2.COLOR_RGB2BGR)
                print(f"[{get_timestamp()}] Converted to BGR. Converting to grayscale...")

                gray_capture = cv2.cvtColor(img_bgr_capture, cv2.COLOR_BGR2GRAY)
                print(f"[{get_timestamp()}] Converted to grayscale. Finding chessboard corners (dims: {chessboard_dims})...")
                
                # Use more robust flags for actual capture detection
                find_flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
                ret_capture, corners_capture = cv2.findChessboardCorners(gray_capture, chessboard_dims, find_flags)
                print(f"[{get_timestamp()}] findChessboardCorners returned: {ret_capture}")

                if ret_capture:
                    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                    print(f"[{get_timestamp()}] Refining corners...")
                    corners_refined = cv2.cornerSubPix(gray_capture, corners_capture, (11, 11), (-1, -1), criteria)
                    print(f"[{get_timestamp()}] Corners refined.")

                    img_filename = output_dir / f"{filename_prefix}{captured_count:02d}.png"
                    cv2.imwrite(str(img_filename), img_bgr_capture)
                    print(f"[{get_timestamp()}] Image saved: {img_filename}")
                    
                    # Draw corners on a copy and save for verification
                    img_with_corners = img_bgr_capture.copy()
                    cv2.drawChessboardCorners(img_with_corners, chessboard_dims, corners_refined, ret_capture)
                    cv2.imwrite(str(output_dir / f"{filename_prefix}{captured_count:02d}_corners.png"), img_with_corners)
                    
                    captured_count += 1
                else:
                    print(f"[{get_timestamp()}] Chessboard not found in captured image. Try a different angle/distance.")
                    failed_img_path = output_dir / f"{FAILED_IMAGE_FILENAME_PREFIX}{time.strftime('%Y%m%d_%H%M%S')}.png"
                    cv2.imwrite(str(failed_img_path), img_bgr_capture)
                    print(f"[{get_timestamp()}] Saved image with failed detection to: {failed_img_path}")
        
        # End of while loop
        if captured_count < num_images and key != ord('q'):
            print(f"\n[{get_timestamp()}] Loop exited before capturing enough images. Captured: {captured_count}")


    except Exception as e:
        print(f"[{get_timestamp()}] CRITICAL ERROR in capture_calibration_images: {e}")
        print(traceback.format_exc())
    finally:
        if picam2 is not None:
            print(f"\n[{get_timestamp()}] Stopping camera...")
            picam2.stop()
            print(f"[{get_timestamp()}] Camera stopped.")
        cv2.destroyAllWindows()
        print(f"[{get_timestamp()}] OpenCV windows destroyed.")
        print(f"[{get_timestamp()}] Captured {captured_count} valid images.")
        return captured_count >= num_images


def calibrate_camera(image_dir, chessboard_dims, square_size, calibration_file):
    print(f"\n[{get_timestamp()}] --- Starting Calibration ---")
    
    # Filter for main calibration images, excluding _corners and failed_detect images
    image_files = sorted([p for p in image_dir.glob(f"{IMAGE_FILENAME_PREFIX}*.png")
                          if "_corners" not in p.name and FAILED_IMAGE_FILENAME_PREFIX not in p.name])

    if not image_files:
        print(f"[{get_timestamp()}] Error: No suitable calibration images ({IMAGE_FILENAME_PREFIX}*.png without _corners or {FAILED_IMAGE_FILENAME_PREFIX}) found in {image_dir}.")
        return False

    print(f"[{get_timestamp()}] Found {len(image_files)} images for calibration: {[f.name for f in image_files]}")

    objp = create_object_points(chessboard_dims, square_size)
    objpoints = []
    imgpoints = []
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    processed_count = 0
    successful_detections = 0
    first_gray_image_for_size = None # To get image dimensions

    print(f"[{get_timestamp()}] Processing images for calibration...")
    for fname in image_files:
        img = cv2.imread(str(fname))
        if img is None:
            print(f"[{get_timestamp()}] Warning: Could not read image {fname.name}")
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if first_gray_image_for_size is None:
            first_gray_image_for_size = gray
        
        processed_count += 1
        print(f"[{get_timestamp()}] Calibrating: Processing {fname.name}. Finding corners (dims: {chessboard_dims})...")
        
        # Use robust flags for calibration detection as well
        find_flags_calib = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        ret, corners = cv2.findChessboardCorners(gray, chessboard_dims, find_flags_calib)
        print(f"[{get_timestamp()}] Calibrating: findChessboardCorners for {fname.name} returned: {ret}")

        if ret:
            successful_detections += 1
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)
            print(f"  [{get_timestamp()}] Processed {fname.name} - Corners found.")
            # Optional: Display corners during calibration (can be slow)
            # img_display_calib = img.copy()
            # cv2.drawChessboardCorners(img_display_calib, chessboard_dims, corners2, ret)
            # cv2.imshow(f'Calib Corners - {fname.name}', img_display_calib)
            # cv2.waitKey(200) 
            # cv2.destroyWindow(f'Calib Corners - {fname.name}')
        else:
            print(f"  [{get_timestamp()}] Processed {fname.name} - Corners NOT found.")
    
    # cv2.destroyAllWindows() # Clean up any display windows from calibration

    # Require a reasonable number of successful detections
    min_successful_detections = max(1, min(NUM_IMAGES_TO_CAPTURE // 2, 10)) # e.g., at least 1, half of target, or 10
    if successful_detections < min_successful_detections:
         print(f"[{get_timestamp()}] Error: Only found corners in {successful_detections}/{processed_count} images.")
         print(f"Need at least {min_successful_detections} images with clearly visible chessboard patterns for reliable calibration.")
         return False

    print(f"\n[{get_timestamp()}] Found corners in {successful_detections}/{processed_count} images during calibration step.")
    
    if not objpoints or not imgpoints:
        print(f"[{get_timestamp()}] Error: No valid object points or image points collected. Cannot calibrate.")
        return False
        
    if first_gray_image_for_size is None:
        print(f"[{get_timestamp()}] Error: Could not determine image size (no images successfully read for calibration).")
        return False
    img_size = (first_gray_image_for_size.shape[1], first_gray_image_for_size.shape[0]) # width, height
    print(f"[{get_timestamp()}] Image size for calibration: {img_size}")

    print(f"[{get_timestamp()}] Calibrating camera...")
    try:
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, img_size, None, None)
    except cv2.error as e:
        print(f"[{get_timestamp()}] OpenCV Error during calibrateCamera: {e}")
        print(traceback.format_exc())
        if "Assertion failed" in str(e) and ("nimages > 0" in str(e) or "objpoints.size()" in str(e)):
             print("This often means no images had detectable patterns, or objpoints/imgpoints arrays are empty/mismatched.")
        return False
    except Exception as e:
        print(f"[{get_timestamp()}] Unexpected Error during calibrateCamera: {e}")
        print(traceback.format_exc())
        return False


    if not ret:
        print(f"[{get_timestamp()}] Error: Camera calibration failed (calibrateCamera returned False).")
        return False

    print(f"\n[{get_timestamp()}] --- Calibration Results ---")
    print("Camera Matrix (Intrinsics):\n", mtx)
    print("\nDistortion Coefficients:\n", dist)

    mean_error = 0
    if objpoints: # Ensure objpoints is not empty
        for i in range(len(objpoints)):
            imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
            error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
            mean_error += error
        reprojection_error = mean_error / len(objpoints)
        print(f"\nMean Reprojection Error: {reprojection_error:.4f} pixels")
    else:
        print("\nMean Reprojection Error: Could not be calculated (no object points).")
        reprojection_error = -1.0 # Indicate error

    print(f"\n[{get_timestamp()}] Saving calibration data to: {calibration_file}")
    np.savez(calibration_file, camera_matrix=mtx, dist_coeffs=dist, rvecs=rvecs, tvecs=tvecs, reprojection_error=reprojection_error, image_size=img_size)
    print(f"[{get_timestamp()}] Calibration data saved successfully.")
    return True

# --- Main Execution ---
if __name__ == "__main__":
    print(f"[{get_timestamp()}] Starting Raspberry Pi Camera Calibration Script")
    print("-" * 40)
    print(f"Chessboard Dimensions (inner corners): {CHESSBOARD_DIMS}")
    print(f"Square Size: {SQUARE_SIZE_MM} mm")
    print(f"Target number of images to capture: {NUM_IMAGES_TO_CAPTURE}")
    print(f"Images will be saved in: ./{IMAGE_SAVE_DIR}/")
    print(f"Calibration data will be saved to: ./{CALIBRATION_FILE}")
    print(f"NOTE: Ensure your Raspberry Pi has adequate power, cooling, and memory, especially for high-resolution captures.")
    print("-" * 40)

    # Check for existing suitable images first
    existing_images = sorted([p for p in IMAGE_SAVE_DIR.glob(f"{IMAGE_FILENAME_PREFIX}*.png")
                              if "_corners" not in p.name and FAILED_IMAGE_FILENAME_PREFIX not in p.name])
    
    capture_phase_needed = True
    if len(existing_images) >= NUM_IMAGES_TO_CAPTURE:
        print(f"[{get_timestamp()}] Found {len(existing_images)} existing suitable images in {IMAGE_SAVE_DIR}.")
        while True:
            capture_choice = input("Do you want to re-capture images? (y/N): ").lower().strip()
            if capture_choice == 'y':
                # Consider cleaning up old images or archiving them
                print(f"[{get_timestamp()}] Will re-capture images.")
                # for f_existing in IMAGE_SAVE_DIR.glob(f"{IMAGE_FILENAME_PREFIX}*"):
                #     try: os.remove(f_existing)
                #     except OSError as e: print(f"Error removing {f_existing}: {e}")
                # print(f"[{get_timestamp()}] Cleaned existing images in {IMAGE_SAVE_DIR}.")
                break
            elif capture_choice == 'n' or capture_choice == '':
                print(f"[{get_timestamp()}] Skipping image capture, using existing images.")
                capture_phase_needed = False
                break
            else:
                print("Invalid input. Please enter 'y' or 'n'.")
    else:
        print(f"[{get_timestamp()}] Found only {len(existing_images)} suitable images. Image capture phase is required.")

    capture_successful = False
    if capture_phase_needed:
        print(f"[{get_timestamp()}] Starting image capture phase...")
        capture_successful = capture_calibration_images(IMAGE_SAVE_DIR, NUM_IMAGES_TO_CAPTURE, IMAGE_FILENAME_PREFIX, CHESSBOARD_DIMS)
    else:
        capture_successful = True # Using existing images

    if not capture_successful and capture_phase_needed:
        print(f"\n[{get_timestamp()}] Image capture did not complete successfully or was aborted.")
        print(f"[{get_timestamp()}] Cannot proceed to calibration without sufficient captured images.")
    elif not capture_phase_needed and len(existing_images) < NUM_IMAGES_TO_CAPTURE:
        print(f"\n[{get_timestamp()}] Not enough existing suitable images ({len(existing_images)} found, {NUM_IMAGES_TO_CAPTURE} required).")
        print(f"[{get_timestamp()}] Cannot proceed to calibration.")
    else:
        # Re-check suitable images if capture phase was skipped or even if it ran
        final_image_check = sorted([p for p in IMAGE_SAVE_DIR.glob(f"{IMAGE_FILENAME_PREFIX}*.png")
                                   if "_corners" not in p.name and FAILED_IMAGE_FILENAME_PREFIX not in p.name])
        if len(final_image_check) < max(1, min(NUM_IMAGES_TO_CAPTURE // 2, 10)) : # Check against min needed for calibration
             print(f"\n[{get_timestamp()}] After capture/check, only {len(final_image_check)} suitable images available. This may not be enough for good calibration.")
             print(f"[{get_timestamp()}] Attempting calibration anyway, but results might be poor.")
        
        if calibrate_camera(IMAGE_SAVE_DIR, CHESSBOARD_DIMS, SQUARE_SIZE_MM, CALIBRATION_FILE):
             print(f"\n[{get_timestamp()}] Calibration finished successfully!")
        else:
             print(f"\n[{get_timestamp()}] Calibration failed.")

    print(f"\n[{get_timestamp()}] Script finished.")