#!/usr/bin/env python3

import cv2
import numpy as np
import time # Make sure time is imported
from picamera2 import Picamera2
from pathlib import Path
import os

# --- Configuration ---
# (Your existing configuration)
CHESSBOARD_DIMS = (10, 7) # Example: Or (10,7) based on your actual board
SQUARE_SIZE_MM = 20.0
NUM_IMAGES_TO_CAPTURE = 20
IMAGE_SAVE_DIR = Path("calibration_images")
IMAGE_FILENAME_PREFIX = "calib_img_"
CALIBRATION_FILE = Path("camera_calibration_data.npz")

# --- Helper Functions ---
def create_object_points(chessboard_dims, square_size):
    objp = np.zeros((chessboard_dims[0] * chessboard_dims[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:chessboard_dims[0], 0:chessboard_dims[1]].T.reshape(-1, 2) * square_size
    return objp

def capture_calibration_images(output_dir, num_images, filename_prefix, chessboard_dims):
    output_dir.mkdir(parents=True, exist_ok=True)
    print(f"Created directory: {output_dir.resolve()}")

    picam2 = Picamera2()
    preview_config = picam2.create_preview_configuration(main={"size": (1024, 768)})
    # Consider making this configurable or testing a slightly smaller resolution if 2028x1520 is problematic
    # For example: capture_config = picam2.create_still_configuration(main={"size": (1920, 1080)})
    capture_config = picam2.create_still_configuration() # This will pick a high resolution, e.g., 2028x1520
    
    print(f"DEBUG: Preview config selected: {preview_config.get('main', {})}")
    print(f"DEBUG: Capture config selected: {capture_config.get('main', {})}") # Check what res it defaults to
    
    picam2.configure(preview_config)
    picam2.start()
    print("Camera started. Press SPACE to capture, 'q' to quit.")
    print(f"Need to capture {num_images} valid images.")

    captured_count = 0
    cv2.namedWindow("Camera Feed - Press SPACE to capture, 'q' to quit", cv2.WINDOW_NORMAL)

    try:
        while captured_count < num_images:
            buffer = picam2.capture_array()
            frame_bgr = cv2.cvtColor(buffer, cv2.COLOR_RGBA2BGR)
            gray_preview = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
            # ret_preview, corners_preview = cv2.findChessboardCorners(gray_preview, chessboard_dims, None) # Keep preview detection fast
            # For preview, consider CALIB_CB_FAST_CHECK if it's slow, or skip drawing corners in preview if that helps responsiveness
            ret_preview, corners_preview = cv2.findChessboardCorners(gray_preview, chessboard_dims, cv2.CALIB_CB_FAST_CHECK)


            if ret_preview:
                cv2.drawChessboardCorners(frame_bgr, chessboard_dims, corners_preview, ret_preview)
            cv2.imshow("Camera Feed - Press SPACE to capture, 'q' to quit", frame_bgr)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                print("\nQuitting capture phase.")
                break
            elif key == ord(' '):
                current_time = time.strftime('%Y-%m-%d %H:%M:%S')
                print(f"\nDEBUG: [{current_time}] Spacebar pressed. Attempting capture {captured_count + 1}/{num_images}...")

                print(f"DEBUG: [{time.strftime('%Y-%m-%d %H:%M:%S')}] Switching to capture_config...")
                picam2.switch_mode(capture_config)
                print(f"DEBUG: [{time.strftime('%Y-%m-%d %H:%M:%S')}] Switched. Actual main config: {picam2.camera_configuration().get('main', {})}")

                print(f"DEBUG: [{time.strftime('%Y-%m-%d %H:%M:%S')}] Capturing array (this may take a moment)...")
                capture_data = picam2.capture_array()
                print(f"DEBUG: [{time.strftime('%Y-%m-%d %H:%M:%S')}] Captured array. Shape: {capture_data.shape if capture_data is not None else 'None'}.")

                print(f"DEBUG: [{time.strftime('%Y-%m-%d %H:%M:%S')}] Switching back to preview_config...")
                picam2.switch_mode(preview_config)
                print(f"DEBUG: [{time.strftime('%Y-%m-%d %H:%M:%S')}] Switched back to preview_config.")

                if capture_data is None:
                    print(f"DEBUG: [{time.strftime('%Y-%m-%d %H:%M:%S')}] Capture data is None. Skipping.")
                    continue

                if capture_data.shape[2] == 4:
                     img_bgr_capture = cv2.cvtColor(capture_data, cv2.COLOR_RGBA2BGR)
                else:
                     img_bgr_capture = cv2.cvtColor(capture_data, cv2.COLOR_RGB2BGR)
                print(f"DEBUG: [{time.strftime('%Y-%m-%d %H:%M:%S')}] Converted to BGR. Converting to grayscale...")
                gray_capture = cv2.cvtColor(img_bgr_capture, cv2.COLOR_BGR2GRAY)
                print(f"DEBUG: [{time.strftime('%Y-%m-%d %H:%M:%S')}] Converted to grayscale. Finding chessboard corners (dims: {chessboard_dims})...")
                
                # --- This is a primary suspect for long processing/hangs ---
                # Consider adding flags or testing alternatives if this hangs
                ret_capture, corners_capture = cv2.findChessboardCorners(gray_capture, chessboard_dims, None)
                # Example with adaptive thresh:
                # ret_capture, corners_capture = cv2.findChessboardCorners(gray_capture, chessboard_dims, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
                print(f"DEBUG: [{time.strftime('%Y-%m-%d %H:%M:%S')}] findChessboardCorners returned: {ret_capture}")

                if ret_capture:
                    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                    print(f"DEBUG: [{time.strftime('%Y-%m-%d %H:%M:%S')}] Refining corners...")
                    corners_refined = cv2.cornerSubPix(gray_capture, corners_capture, (11, 11), (-1, -1), criteria)
                    print(f"DEBUG: [{time.strftime('%Y-%m-%d %H:%M:%S')}] Corners refined.")

                    img_filename = output_dir / f"{filename_prefix}{captured_count:02d}.png"
                    cv2.imwrite(str(img_filename), img_bgr_capture) # Save the BGR image from capture_data
                    print(f"Image saved: {img_filename}")
                    captured_count += 1
                    cv2.drawChessboardCorners(img_bgr_capture, chessboard_dims, corners_refined, ret_capture)
                    cv2.imwrite(str(output_dir / f"{filename_prefix}{captured_count-1:02d}_corners.png"), img_bgr_capture)
                else:
                    print(f"DEBUG: [{time.strftime('%Y-%m-%d %H:%M:%S')}] Chessboard not found in captured image.")
                    # Optionally save images where detection failed for later analysis:
                    # cv2.imwrite(str(output_dir / f"failed_detect_{time.strftime('%H%M%S')}.png"), img_bgr_capture)
    except Exception as e:
        print(f"ERROR in capture_calibration_images: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\nStopping camera.")
        picam2.stop()
        cv2.destroyAllWindows()
        print(f"Captured {captured_count} images.")
        return captured_count >= num_images

# (Rest of your script: calibrate_camera and __main__)
# ... ensure these also have robust error handling or logging if needed ...
def calibrate_camera(image_dir, chessboard_dims, square_size, calibration_file):
    """
    Performs camera calibration using the captured images.
    """
    print("\n--- Starting Calibration ---")
    # Use glob to find PNG images that do not contain "_corners" or "failed_detect"
    image_files = [p for p in image_dir.glob(f"{IMAGE_FILENAME_PREFIX}*.png")
                   if "_corners" not in p.name and "failed_detect" not in p.name]
    image_files.sort()


    if not image_files:
        print(f"Error: No suitable images found in {image_dir} (excluding _corners.png and failed_detect.png). Run capture first.")
        return False

    print(f"Found {len(image_files)} images for calibration: {image_files}")


    objp = create_object_points(chessboard_dims, square_size)
    objpoints = []
    imgpoints = []
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    processed_count = 0
    successful_detections = 0
    first_gray_img_for_size = None

    print("Processing images for calibration...")
    for fname in image_files:
        img = cv2.imread(str(fname))
        if img is None:
            print(f"Warning: Could not read image {fname}")
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if first_gray_img_for_size is None:
            first_gray_img_for_size = gray
        processed_count += 1

        current_time = time.strftime('%Y-%m-%d %H:%M:%S')
        print(f"DEBUG: [{current_time}] Calibrating: Processing {fname.name}. Finding corners (dims: {chessboard_dims})...")
        ret, corners = cv2.findChessboardCorners(gray, chessboard_dims, None)
        print(f"DEBUG: [{current_time}] Calibrating: findChessboardCorners for {fname.name} returned: {ret}")


        if ret:
            successful_detections += 1
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)
            # cv2.drawChessboardCorners(img, chessboard_dims, corners2, ret)
            # cv2.imshow(f'Corners Found - {fname.name}', img)
            # cv2.waitKey(500)
            # cv2.destroyWindow(f'Corners Found - {fname.name}')
            print(f"  Processed {fname.name} - Corners found.")
        else:
            print(f"  Processed {fname.name} - Corners not found.")


    if successful_detections < min(10, len(image_files) * 0.5): # Stricter condition
         print(f"Error: Only found corners in {successful_detections}/{processed_count} images.")
         print("Need more images with clearly visible chessboard patterns from diverse angles, or check detection parameters.")
         return False

    print(f"\nFound corners in {successful_detections}/{processed_count} images during calibration step.")
    print("Calibrating camera...")

    if not imgpoints or not objpoints:
        print("Error: No valid image points or object points collected for calibration.")
        return False

    if first_gray_img_for_size is None:
        print("Error: Could not determine image size. No images processed successfully for calibration?")
        return False
    img_size = (first_gray_img_for_size.shape[1], first_gray_img_for_size.shape[0])

    try:
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, img_size, None, None)
    except cv2.error as e:
        print(f"OpenCV Error during calibrateCamera: {e}")
        if "Assertion failed" in str(e) and "nimages > 0" in str(e):
             print("This often means no images had detectable patterns, or objpoints/imgpoints arrays are empty/mismatched.")
        return False


    if not ret:
        print("Error: Camera calibration failed (calibrateCamera returned False).")
        return False

    print("\n--- Calibration Results ---")
    print("Camera Matrix (Intrinsics):\n", mtx)
    print("\nDistortion Coefficients:\n", dist)

    mean_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        mean_error += error

    reprojection_error = mean_error / len(objpoints) if objpoints else float('inf')
    print(f"\nMean Reprojection Error: {reprojection_error:.4f} pixels")

    print(f"\nSaving calibration data to: {calibration_file}")
    np.savez(calibration_file, camera_matrix=mtx, dist_coeffs=dist, rvecs=rvecs, tvecs=tvecs, reprojection_error=reprojection_error)
    print("Calibration data saved successfully.")
    return True


if __name__ == "__main__":
    print("Starting Raspberry Pi Camera Calibration Script")
    # ... (rest of your main execution logic) ...
    print("Starting Raspberry Pi Camera Calibration Script")
    print("-" * 30)
    print(f"Chessboard Dimensions (inner corners): {CHESSBOARD_DIMS}")
    print(f"Square Size: {SQUARE_SIZE_MM} mm")
    print(f"Number of images to capture: {NUM_IMAGES_TO_CAPTURE}")
    print(f"Images will be saved in: ./{IMAGE_SAVE_DIR}/")
    print(f"Calibration data will be saved to: ./{CALIBRATION_FILE}")
    print("-" * 30)

    existing_images = [p for p in IMAGE_SAVE_DIR.glob(f"{IMAGE_FILENAME_PREFIX}*.png")
                       if "_corners" not in p.name and "failed_detect" not in p.name]
    existing_images.sort()

    capture_success = False
    if len(existing_images) >= NUM_IMAGES_TO_CAPTURE:
        print(f"Found {len(existing_images)} existing suitable images in {IMAGE_SAVE_DIR}.")
        capture_choice = input("Do you want to re-capture images? (y/N): ").lower()
        if capture_choice == 'y':
            # Optional: Clear existing images if re-capturing
            # Consider a more robust way to clean, e.g., moving to a backup folder or being more selective
            # for f_existing in IMAGE_SAVE_DIR.glob(f"{IMAGE_FILENAME_PREFIX}*"):
            #    os.remove(f_existing)
            # print("Removed existing images.")
            capture_success = capture_calibration_images(IMAGE_SAVE_DIR, NUM_IMAGES_TO_CAPTURE, IMAGE_FILENAME_PREFIX, CHESSBOARD_DIMS)
        else:
            print("Skipping image capture, using existing images.")
            capture_success = True # Assume existing images are sufficient
    else:
        print(f"Found only {len(existing_images)} existing images. Starting image capture phase...")
        capture_success = capture_calibration_images(IMAGE_SAVE_DIR, NUM_IMAGES_TO_CAPTURE, IMAGE_FILENAME_PREFIX, CHESSBOARD_DIMS)

    if not capture_success:
        print("\nImage capture did not complete successfully, was aborted, or not enough images were captured/found.")
        print("Cannot proceed to calibration without sufficient images.")
    else:
        if calibrate_camera(IMAGE_SAVE_DIR, CHESSBOARD_DIMS, SQUARE_SIZE_MM, CALIBRATION_FILE):
             print("\nCalibration finished successfully!")
        else:
             print("\nCalibration failed.")

    print("\nScript finished.")