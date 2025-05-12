# cv_functions.py

import cv2
import numpy as np
import logging
import Config # Import the new config file

def undistort_frame(frame, mtx, dist, new_camera_mtx):
    """
    Undistorts an image frame using camera calibration parameters.
    Args:
        frame (np.ndarray): The raw BGR frame.
        mtx (np.ndarray): The camera intrinsic matrix.
        dist (np.ndarray): The camera distortion coefficients.
        new_camera_mtx (np.ndarray): The new camera matrix, possibly optimized, from cv2.getOptimalNewCameraMatrix.
                                     If None, mtx is used.
    Returns:
        np.ndarray: The undistorted frame. Returns original frame if mtx or dist is None.
    """
    if mtx is None or dist is None:
        logging.warning("CV: Camera matrix or distortion coefficients not available. Skipping undistortion.")
        return frame
    
    # If new_camera_mtx is not provided or is not valid, use original mtx for undistortion
    # This ensures that an undistorted image is always produced if mtx and dist are available
    current_new_mtx = new_camera_mtx if new_camera_mtx is not None else mtx

    try:
        undistorted_img = cv2.undistort(frame, mtx, dist, None, current_new_mtx)
        return undistorted_img
    except Exception as e:
        logging.error(f"CV: Error during undistortion: {e}")
        return frame # Return original frame on error


def detect_volleyball_2d(undistorted_frame, roi=None):
    """
    Detects the Mikasa volleyball in an undistorted frame using color segmentation.
    Args:
        undistorted_frame (np.ndarray): The undistorted BGR frame.
        roi (tuple, optional): A tuple (x, y, w, h) defining a Region of Interest.
                               If None, the whole frame is processed.
    Returns:
        tuple: (u, v, radius_px, masked_image)
               u, v: Pixel coordinates of the ball's center (or None if not found).
               radius_px: Approximate radius of the ball in pixels (or None).
               masked_image: The combined mask for visualization (or None).
    """
    if undistorted_frame is None:
        logging.warning("CV: detect_volleyball_2d received None frame.")
        return None, None, None, None

    frame_to_process = undistorted_frame
    offset_x, offset_y = 0, 0

    if roi:
        try:
            x, y, w, h = roi
            if x >= 0 and y >= 0 and w > 0 and h > 0 and \
               x + w <= undistorted_frame.shape[1] and \
               y + h <= undistorted_frame.shape[0]:
                frame_to_process = undistorted_frame[y:y+h, x:x+w]
                offset_x, offset_y = x, y
                logging.debug(f"CV: Processing ROI: {roi}")
            else:
                logging.warning(f"CV: Invalid ROI {roi} for frame shape {undistorted_frame.shape[:2]}. Processing full frame.")
        except Exception as e:
            logging.warning(f"CV: Error applying ROI {roi}: {e}. Processing full frame.")


    try:
        hsv_frame = cv2.cvtColor(frame_to_process, cv2.COLOR_BGR2HSV)
    except Exception as e:
        logging.error(f"CV: Failed to convert frame to HSV: {e}")
        return None, None, None, None

    # Create masks for yellow and blue
    mask_yellow = cv2.inRange(hsv_frame, config.LOWER_YELLOW_HSV, config.UPPER_YELLOW_HSV)
    mask_blue = cv2.inRange(hsv_frame, config.LOWER_BLUE_HSV, config.UPPER_BLUE_HSV)

    # Combine masks
    combined_mask = cv2.bitwise_or(mask_yellow, mask_blue)

    # Morphological operations (Opening followed by Closing)
    kernel = np.ones(config.MORPH_KERNEL_SIZE, np.uint8)
    mask_morphed = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)
    mask_morphed = cv2.morphologyEx(mask_morphed, cv2.MORPH_CLOSE, kernel, iterations=2) # More closing

    # Find contours
    contours, _ = cv2.findContours(mask_morphed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    ball_center_u, ball_center_v, ball_radius_px = None, None, None

    if contours:
        # Filter contours - e.g., find the largest one that fits size criteria
        valid_contours = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if config.MIN_BALL_CONTOUR_AREA < area < config.MAX_BALL_CONTOUR_AREA:
                # Optional: Add circularity check if needed
                # perimeter = cv2.arcLength(cnt, True)
                # if perimeter == 0: continue
                # circularity = 4 * np.pi * (area / (perimeter * perimeter))
                # if 0.7 < circularity < 1.3: # Example circularity range
                valid_contours.append(cnt)

        if valid_contours:
            # Assume the largest valid contour is the ball
            ball_contour = max(valid_contours, key=cv2.contourArea)

            # Get bounding circle
            ((u_roi, v_roi), radius_px_roi) = cv2.minEnclosingCircle(ball_contour) #

            # Check if radius is within expected pixel range based on config
            if config.MIN_BALL_RADIUS_PX <= radius_px_roi <= config.MAX_BALL_RADIUS_PX:
                # Calculate centroid using moments (can be more stable than minEnclosingCircle center for non-perfect circles)
                M = cv2.moments(ball_contour)
                if M["m00"] != 0:
                    centroid_u_roi = int(M["m10"] / M["m00"])
                    centroid_v_roi = int(M["m01"] / M["m00"])

                    # Adjust coordinates back to full frame if ROI was used
                    ball_center_u = centroid_u_roi + offset_x
                    ball_center_v = centroid_v_roi + offset_y
                    ball_radius_px = int(radius_px_roi) # Use radius from minEnclosingCircle

                    logging.debug(f"CV: Ball detected at ({ball_center_u}, {ball_center_v}), radius ~{ball_radius_px}px.")
                else:
                    logging.warning("CV: Valid ball contour found but m00 is zero.")
            else:
                logging.debug(f"CV: Largest contour's radius {radius_px_roi:.1f}px out of range ({config.MIN_BALL_RADIUS_PX}-{config.MAX_BALL_RADIUS_PX}px).")
        else:
            logging.debug("CV: No contours passed filtering criteria.")
    else:
        logging.debug("CV: No contours found in mask.")

    # Prepare the mask_morphed for output (convert to BGR for display if needed by UI)
    # If ROI was used, create a full-size mask for visualization
    if offset_x > 0 or offset_y > 0:
        full_mask_morphed = np.zeros_like(undistorted_frame[:,:,0], dtype=np.uint8)
        full_mask_morphed[offset_y:offset_y+frame_to_process.shape[0], offset_x:offset_x+frame_to_process.shape[1]] = mask_morphed
        mask_for_display = cv2.cvtColor(full_mask_morphed, cv2.COLOR_GRAY2BGR)
    else:
        mask_for_display = cv2.cvtColor(mask_morphed, cv2.COLOR_GRAY2BGR)


    return ball_center_u, ball_center_v, ball_radius_px, mask_for_display


def map_2d_to_3d_on_ground(u, v, mtx, dist, rvec, tvec):
    """
    Maps 2D image coordinates (u,v) to 3D world coordinates (X, Y, Z=0) on the ground plane.
    This function implements the ray-plane intersection method described in the PDF (Section 4).

    Args:
        u (int): The x-coordinate (pixel) of the ball in the image.
        v (int): The y-coordinate (pixel) of the ball in the image.
        mtx (np.ndarray): The camera intrinsic matrix.
        dist (np.ndarray): The camera distortion coefficients.
        rvec (np.ndarray): The rotation vector (from cv2.solvePnP) describing world to camera.
        tvec (np.ndarray): The translation vector (from cv2.solvePnP) describing world to camera.

    Returns:
        np.ndarray: A 3-element NumPy array [X, Y, Z] representing the 3D world coordinates
                    of the ball on the ground plane (Z will be 0). Returns None if mapping fails.
    """
    if u is None or v is None:
        logging.debug("CV: map_2d_to_3d received None for u or v.")
        return None
    if mtx is None or dist is None or rvec is None or tvec is None:
        logging.error("CV: Missing camera parameters (mtx, dist, rvec, tvec) for 2D to 3D mapping.")
        return None

    try:
        # 1. Normalize 2D Image Point (and undistort it)
        # cv2.undistortPoints requires points in shape (N, 1, 2)
        image_point_2d = np.array([[[float(u), float(v)]]], dtype=np.float32)

        # UndistortPoints output is ALREADY NORMALIZED if P is not given or is identity
        # It gives coordinates on a plane at Z=1 in camera coordinates.
        # If P=mtx, it gives undistorted pixel coordinates. We need normalized for ray.
        normalized_points = cv2.undistortPoints(image_point_2d, mtx, dist, P=None) # Using P=None for normalized
        # normalized_points has shape (1,1,2), containing (xn, yn)
        xn = normalized_points[0,0,0]
        yn = normalized_points[0,0,1]
        logging.debug(f"CV: Pixel ({u},{v}) -> Normalized ({xn:.4f},{yn:.4f})")


        # 2. Define Ray in Camera Coordinates
        # Ray origin in camera coordinates is (0,0,0)
        # Ray direction vector in camera coordinates (points from camera center through normalized point on Z=1 plane)
        ray_direction_camera = np.array([xn, yn, 1.0], dtype=np.float32)
        ray_direction_camera = ray_direction_camera / np.linalg.norm(ray_direction_camera) # Normalize the direction vector

        # 3. Transform Ray to World Coordinates
        # We need the inverse transformation: from camera to world.
        # R_wc is rotation from world to camera (from rvec)
        # t_wc is translation of world origin in camera coords (from tvec)
        R_wc, _ = cv2.Rodrigues(rvec) # Convert rotation vector to matrix

        # Inverse rotation: R_cw = R_wc.T
        R_cw = R_wc.T

        # Camera optical center (origin of the ray) in world coordinates:
        # O_world = -R_wc.T * t_wc  (or -R_cw * t_wc)
        O_world = -np.dot(R_cw, tvec).reshape(3) # Camera position in world coordinates

        # Ray direction in world coordinates:
        # d_world = R_wc.T * d_camera (or R_cw * d_camera)
        ray_direction_world = np.dot(R_cw, ray_direction_camera).reshape(3)
        logging.debug(f"CV: Ray Origin (Camera Pos in World): {O_world}")
        logging.debug(f"CV: Ray Direction in World: {ray_direction_world}")


        # 4. Ray-Plane Intersection (Ground Plane Z=0)
        # Parametric equation of the ray in world coordinates: P(k) = O_world + k * ray_direction_world
        # We want the Z-component of P(k) to be 0:
        # O_world[2] + k * ray_direction_world[2] = 0

        # Check if ray is parallel to the ground plane (Z=0)
        # This happens if the Z-component of the ray's world direction is close to zero.
        if abs(ray_direction_world[2]) < 1e-6: # Threshold for parallelism
            logging.warning("CV: Ray is parallel or near parallel to the ground plane (Z_d is close to 0). Cannot intersect.")
            return None

        # Solve for k:
        k = -O_world[2] / ray_direction_world[2]
        logging.debug(f"CV: Solved k for ray-plane intersection: {k:.4f}")

        # If k is negative, it means the intersection point is behind the camera's origin (along the ray's direction).
        # This can happen if the camera is looking away from the ground or the detected point is above horizon.
        if k < 0:
            logging.warning(f"CV: Intersection parameter k ({k:.4f}) is negative. "
                            "This means the 2D point projects to a 3D point behind the camera's view on the Z=0 plane. "
                            "This might indicate the ball is detected above the horizon or camera is aimed upwards.")
            # Depending on use case, might still return this or None.
            # For ball on ground, this is likely an error condition or out of bounds.
            # return None # Or proceed if "behind" is acceptable in some contexts

        # Calculate the 3D intersection point in world coordinates:
        X_ball = O_world[0] + k * ray_direction_world[0]
        Y_ball = O_world[1] + k * ray_direction_world[1]
        Z_ball = 0.0  # By definition, the intersection is on the Z=0 plane.
                      # O_world[2] + k * ray_direction_world[2] should be close to 0

        world_coords = np.array([X_ball, Y_ball, Z_ball], dtype=np.float32)
        logging.info(f"CV: Mapped 2D ({u},{v}) to 3D World ({X_ball:.3f}, {Y_ball:.3f}, {Z_ball:.3f})m")
        return world_coords

    except Exception as e:
        logging.error(f"CV: Error during 2D to 3D mapping: {e}", exc_info=True)
        return None


def draw_ball_on_frame(frame, u, v, radius_px):
    """Draws a circle and center for the detected ball on the frame."""
    if frame is None: return
    if u is not None and v is not None and radius_px is not None:
        cv2.circle(frame, (int(u), int(v)), int(radius_px), (0, 255, 0), 2) # Green circle
        cv2.circle(frame, (int(u), int(v)), 3, (0, 0, 255), -1)      # Red center dot
    return frame

def draw_3d_info_on_frame(frame, world_coords_3d, ball_radius_m):
    """Displays the 3D world coordinates on the frame."""
    if frame is None: return
    if world_coords_3d is not None:
        text_x = f"X: {world_coords_3d[0]:.2f}m"
        text_y = f"Y: {world_coords_3d[1]:.2f}m"
        # Z is assumed 0 for ground, but we can show it.
        # For visualization, the ball sphere center will be at (X, Y, ball_radius_m)
        text_z_vis = f"Z_vis: {ball_radius_m:.2f}m (center)"

        cv2.putText(frame, text_x, (10, frame.shape[0] - 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        cv2.putText(frame, text_y, (10, frame.shape[0] - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        cv2.putText(frame, text_z_vis, (10, frame.shape[0] - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
    return frame

def draw_axes_on_frame(frame, rvec, tvec, mtx, dist, axis_length_m=0.5):
    """
    Projects and draws the world coordinate axes onto the image.
    Args:
        frame (np.ndarray): The image to draw on.
        rvec (np.ndarray): Rotation vector of the world w.r.t camera.
        tvec (np.ndarray): Translation vector of the world w.r.t camera.
        mtx (np.ndarray): Camera intrinsic matrix.
        dist (np.ndarray): Camera distortion coefficients.
        axis_length_m (float): Length of the axes to draw in meters.
    Returns:
        np.ndarray: Frame with axes drawn.
    """
    if frame is None or rvec is None or tvec is None or mtx is None or dist is None:
        return frame

    # Define 3D points for the axes in world coordinates
    # Origin, and points along X, Y, Z axes
    axis_points_3d = np.float32([
        [0, 0, 0],  # Origin
        [axis_length_m, 0, 0],  # X-axis end
        [0, axis_length_m, 0],  # Y-axis end
        [0, 0, axis_length_m]   # Z-axis end (pointing up)
    ]).reshape(-1, 3)

    try:
        # Project 3D points to 2D image plane
        image_points, _ = cv2.projectPoints(axis_points_3d, rvec, tvec, mtx, dist)
        image_points = np.int32(image_points.reshape(-1, 2))

        # Draw lines for the axes
        origin_2d = tuple(image_points[0])
        x_axis_end_2d = tuple(image_points[1])
        y_axis_end_2d = tuple(image_points[2])
        z_axis_end_2d = tuple(image_points[3])

        cv2.line(frame, origin_2d, x_axis_end_2d, (255, 0, 0), 3)  # X-axis: Blue
        cv2.line(frame, origin_2d, y_axis_end_2d, (0, 255, 0), 3)  # Y-axis: Green
        cv2.line(frame, origin_2d, z_axis_end_2d, (0, 0, 255), 3)  # Z-axis: Red

        # Optional: Add labels
        cv2.putText(frame, "X", (x_axis_end_2d[0] + 5, x_axis_end_2d[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,0,0), 2)
        cv2.putText(frame, "Y", (y_axis_end_2d[0] + 5, y_axis_end_2d[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
        cv2.putText(frame, "Z", (z_axis_end_2d[0] + 5, z_axis_end_2d[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)

    except Exception as e:
        logging.error(f"CV: Error drawing axes: {e}")

    return frame


if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')
    # This block is for testing functions individually.
    # You would need a sample image and camera parameters.

    # Example: Test undistortion
    # mock_frame = np.zeros((config.CAM_REQUESTED_HEIGHT, config.CAM_REQUESTED_WIDTH, 3), dtype=np.uint8)
    # cv2.putText(mock_frame, "Original Test Frame (Distorted)", (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255),2)
    # mock_mtx = config.CAMERA_INTRINSIC_MTX # Use from config
    # mock_dist = config.CAMERA_DIST_COEFFS # Use from config
    # mock_new_mtx, _ = cv2.getOptimalNewCameraMatrix(mock_mtx, mock_dist, (mock_frame.shape[1], mock_frame.shape[0]), 1, (mock_frame.shape[1], mock_frame.shape[0]))
    #
    # undistorted_test_frame = undistort_frame(mock_frame, mock_mtx, mock_dist, mock_new_mtx)
    # if undistorted_test_frame is not None:
    #     cv2.imshow("Original", mock_frame)
    #     cv2.imshow("Undistorted Test", undistorted_test_frame)
    #     logging.info("Displaying original and undistorted test frames. Press any key to continue.")
    #     cv2.waitKey(0)
    #     cv2.destroyAllWindows()

    # Example: Test ball detection (requires a sample image with a ball)
    # sample_image_path = "path_to_your_sample_image_with_ball.jpg"
    # try:
    #     test_ball_frame = cv2.imread(sample_image_path)
    #     if test_ball_frame is None: raise FileNotFoundError
    #
    #     # Assume undistortion is done or not critical for this standalone test
    #     u_ball, v_ball, r_ball, dbg_mask = detect_volleyball_2d(test_ball_frame, roi=config.BALL_DETECTION_ROI)
    #
    #     if u_ball is not None:
    #         logging.info(f"Test: Ball detected at ({u_ball}, {v_ball}) with radius {r_ball}px.")
    #         draw_ball_on_frame(test_ball_frame, u_ball, v_ball, r_ball)
    #         if dbg_mask is not None: cv2.imshow("Detection Mask", dbg_mask)
    #     else:
    #         logging.info("Test: Ball not detected.")
    #
    #     cv2.imshow("Ball Detection Test", test_ball_frame)
    #     logging.info("Displaying ball detection test frame. Press any key to continue.")
    #     cv2.waitKey(0)
    #     cv2.destroyAllWindows()
    # except FileNotFoundError:
    #     logging.warning(f"Sample image for ball detection test not found at {sample_image_path}")
    # except Exception as e:
    #     logging.error(f"Error in ball detection test: {e}")


    # Example: Test 2D to 3D mapping
    # This requires actual calibrated camera parameters (mtx, dist) and pose (rvec, tvec)
    # MOCK values for rvec, tvec (these would come from solvePnP)
    # Typically, rvec might be like np.array([0.1, -0.2, 0.05])
    # tvec might be like np.array([-0.1, 0.1, 1.5]) # (camera is 1.5m away from world origin, slightly offset)
    # mock_rvec = np.array([0.0, 0.0, 0.0], dtype=np.float32) # World aligned with camera (unrealistic)
    # mock_tvec = np.array([0.0, 0.0, 2.0], dtype=np.float32) # Camera 2m away along Z axis of world
    #
    # test_u, test_v = config.CAM_REQUESTED_WIDTH // 2, config.CAM_REQUESTED_HEIGHT // 2 # Center of image
    #
    # # Use actual config values for mtx, dist if available, or placeholders
    # test_mtx = config.CAMERA_INTRINSIC_MTX
    # test_dist = config.CAMERA_DIST_COEFFS
    #
    # # Load real calibration if available for a more meaningful test
    # try:
    #     calib_data = np.load(config.CALIBRATION_DATA_FILE)
    #     test_mtx = calib_data['camera_matrix']
    #     test_dist = calib_data['dist_coeffs']
    #     logging.info(f"Loaded real calibration data for 2D-3D test from {config.CALIBRATION_DATA_FILE}")
    #
    #     # For a full test, rvecs and tvecs for one of the calibration images could be used
    #     # For now, we'll use placeholder rvec/tvec. In a real app, these come from solvePnP on the box corners.
    #     # This mock rvec/tvec means the world origin is 2m in front of the camera, aligned.
    #     mock_rvec = np.array([0.0, 0.0, 0.0], dtype=np.float32) # World Y is Camera Y, World X is Camera X
    #     mock_tvec = np.array([0.0, 0.0, 2.0], dtype=np.float32) # World origin is 2m in front (camera Z)
    #
    # except FileNotFoundError:
    #     logging.warning(f"Calibration file {config.CALIBRATION_DATA_FILE} not found. Using placeholder mtx/dist for 2D-3D test.")
    # except KeyError:
    #     logging.warning(f"Calibration file {config.CALIBRATION_DATA_FILE} has incorrect format. Using placeholder mtx/dist.")
    #
    #
    # coords_3d = map_2d_to_3d_on_ground(test_u, test_v, test_mtx, test_dist, mock_rvec, mock_tvec)
    # if coords_3d is not None:
    #     logging.info(f"Test: Mapped 2D ({test_u},{test_v}) to 3D World ({coords_3d[0]:.3f}, {coords_3d[1]:.3f}, {coords_3d[2]:.3f})m")
    # else:
    #     logging.warning("Test: 2D to 3D mapping failed.")

    logging.info("cv_functions.py standalone tests finished (commented out).")