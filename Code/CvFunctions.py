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

    # Create masks for yellow and blue - Corrected Config access
    mask_yellow = cv2.inRange(hsv_frame, Config.LOWER_YELLOW_HSV, Config.UPPER_YELLOW_HSV)
    mask_blue = cv2.inRange(hsv_frame, Config.LOWER_BLUE_HSV, Config.UPPER_BLUE_HSV)

    # Combine masks
    combined_mask = cv2.bitwise_or(mask_yellow, mask_blue)

    # Morphological operations (Opening followed by Closing) - Corrected Config access
    kernel = np.ones(Config.MORPH_KERNEL_SIZE, np.uint8)
    mask_morphed = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)
    mask_morphed = cv2.morphologyEx(mask_morphed, cv2.MORPH_CLOSE, kernel, iterations=2)

    # Find contours
    contours, _ = cv2.findContours(mask_morphed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    ball_center_u, ball_center_v, ball_radius_px = None, None, None

    if contours:
        # Filter contours - e.g., find the largest one that fits size criteria
        valid_contours = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            # Corrected Config access
            if Config.MIN_BALL_CONTOUR_AREA < area < Config.MAX_BALL_CONTOUR_AREA:
                valid_contours.append(cnt)

        if valid_contours:
            ball_contour = max(valid_contours, key=cv2.contourArea)
            ((u_roi, v_roi), radius_px_roi) = cv2.minEnclosingCircle(ball_contour)

            # Corrected Config access
            if Config.MIN_BALL_RADIUS_PX <= radius_px_roi <= Config.MAX_BALL_RADIUS_PX:
                M = cv2.moments(ball_contour)
                if M["m00"] != 0:
                    centroid_u_roi = int(M["m10"] / M["m00"])
                    centroid_v_roi = int(M["m01"] / M["m00"])

                    ball_center_u = centroid_u_roi + offset_x
                    ball_center_v = centroid_v_roi + offset_y
                    ball_radius_px = int(radius_px_roi)

                    logging.debug(f"CV: Ball detected at ({ball_center_u}, {ball_center_v}), radius ~{ball_radius_px}px.")
                else:
                    logging.warning("CV: Valid ball contour found but m00 is zero.")
            else:
                # Corrected Config access
                logging.debug(f"CV: Largest contour's radius {radius_px_roi:.1f}px out of range ({Config.MIN_BALL_RADIUS_PX}-{Config.MAX_BALL_RADIUS_PX}px).")
        else:
            logging.debug("CV: No contours passed filtering criteria.")
    else:
        logging.debug("CV: No contours found in mask.")

    # Prepare the mask for display, ensuring it's BGR if it's to be overlaid or displayed directly
    # If an ROI was used, create a full-size mask for display.
    if offset_x > 0 or offset_y > 0:
        # Create a full-size black image
        full_mask_morphed_gray = np.zeros(undistorted_frame.shape[:2], dtype=np.uint8)
        # Place the ROI-processed mask onto it
        full_mask_morphed_gray[offset_y:offset_y+frame_to_process.shape[0], offset_x:offset_x+frame_to_process.shape[1]] = mask_morphed
        # Convert the full-size grayscale mask to BGR
        mask_for_display = cv2.cvtColor(full_mask_morphed_gray, cv2.COLOR_GRAY2BGR)
    else:
        # Convert the already full-size grayscale mask to BGR
        mask_for_display = cv2.cvtColor(mask_morphed, cv2.COLOR_GRAY2BGR)


    return ball_center_u, ball_center_v, ball_radius_px, mask_for_display


def map_2d_to_3d_on_ground(u, v, mtx, dist, rvec, tvec):
    """
    Maps 2D image coordinates (u,v) to 3D world coordinates (X, Y, Z=0) on the ground plane.
    This function implements the ray-plane intersection method.

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
        image_point_2d = np.array([[[float(u), float(v)]]], dtype=np.float32)
        
        # Undistort the 2D image point.
        # The P argument to undistortPoints can be the new camera matrix if refinement was done,
        # or the original camera matrix (mtx) if no refinement/cropping is strictly applied here.
        # For ray casting, we generally want points in the normalized camera coordinate system,
        # so P=None or P=cv2.getOptimalNewCameraMatrix with alpha=0 might be relevant,
        # but usually P=mtx (or new_camera_mtx if used for undistortion map) is for remapping to pixels.
        # The most straightforward way is to undistort to normalized coordinates, then form the ray.
        # Using P=None for cv2.undistortPoints gives normalized coordinates if no P is given (or P=identity).
        # Let's assume mtx is the correct intrinsic matrix for the (u,v) coordinates.
        
        # Use cv2.undistortPoints to get normalized image coordinates (xn, yn)
        # The points are passed as a 1xNx2 array.
        normalized_points = cv2.undistortPoints(image_point_2d, mtx, dist, P=None) # P=None gives normalized coords
        xn = normalized_points[0,0,0]
        yn = normalized_points[0,0,1]
        logging.debug(f"CV: Pixel ({u},{v}) -> Normalized ({xn:.4f},{yn:.4f})")

        # The ray direction in camera coordinates is (xn, yn, 1.0)
        ray_direction_camera = np.array([xn, yn, 1.0], dtype=np.float32)
        # No need to normalize here as k will scale it.

        # Convert rotation vector to rotation matrix (camera to world)
        R_cw, _ = cv2.Rodrigues(rvec) # This is World to Camera rotation
        R_wc = R_cw.T # Camera to World rotation

        # Transform ray direction from camera to world coordinates
        ray_direction_world = R_wc @ ray_direction_camera # Matrix multiplication

        # Camera origin in world coordinates (Optical center O_w)
        # O_c = R_cw * O_w + t_cw  => O_w = R_cw_T * (O_c - t_cw) where O_c is (0,0,0)
        # O_w = -R_cw_T * t_cw
        camera_origin_world = -R_wc @ tvec.reshape(3) # tvec is from world to camera

        logging.debug(f"CV: Ray Origin (Camera Pos in World): {camera_origin_world}")
        logging.debug(f"CV: Ray Direction in World: {ray_direction_world}")

        # Intersection with ground plane Z=0
        # Parametric ray: P(k) = camera_origin_world + k * ray_direction_world
        # We want Z-component of P(k) to be 0.
        # camera_origin_world[2] + k * ray_direction_world[2] = 0
        
        if abs(ray_direction_world[2]) < 1e-6: # Ray is parallel to the ground plane
            logging.warning("CV: Ray is parallel or near parallel to the ground plane (Z_d is close to 0). Cannot intersect.")
            return None

        k = -camera_origin_world[2] / ray_direction_world[2]
        logging.debug(f"CV: Solved k for ray-plane intersection: {k:.4f}")

        if k < 0:
            logging.warning(f"CV: Intersection parameter k ({k:.4f}) is negative. "
                            "This means the 2D point projects to a 3D point behind the camera's view on the Z=0 plane. ")
            # Depending on use case, might still return this or None.
            # For ball on ground, this is likely an error condition or out of bounds.
            # return None 

        # Calculate intersection point
        X_ball = camera_origin_world[0] + k * ray_direction_world[0]
        Y_ball = camera_origin_world[1] + k * ray_direction_world[1]
        Z_ball = 0.0 # By definition of intersection with Z=0 plane

        world_coords = np.array([X_ball, Y_ball, Z_ball], dtype=np.float32)
        logging.info(f"CV: Mapped 2D ({u},{v}) to 3D World ({X_ball:.3f}, {Y_ball:.3f}, {Z_ball:.3f})m")
        return world_coords

    except Exception as e:
        logging.error(f"CV: Error during 2D to 3D mapping: {e}", exc_info=True)
        return None


def draw_ball_on_frame(frame, u, v, radius_px):
    """Draws a circle and center for the detected ball on the frame."""
    if frame is None: return frame # Return frame if it's None
    if u is not None and v is not None and radius_px is not None:
        try:
            cv2.circle(frame, (int(u), int(v)), int(radius_px), (0, 255, 0), 2) # Green circle
            cv2.circle(frame, (int(u), int(v)), 3, (0, 0, 255), -1) # Red center
        except Exception as e:
            logging.error(f"CV: Error drawing ball on frame: {e}")
    return frame

def draw_3d_info_on_frame(frame, world_coords_3d, ball_radius_m):
    """Displays the 3D world coordinates on the frame."""
    if frame is None: return frame # Return frame if it's None
    if world_coords_3d is not None:
        text_x = f"X: {world_coords_3d[0]:.2f}m"
        text_y = f"Y: {world_coords_3d[1]:.2f}m"
        # The Z coordinate from map_2d_to_3d_on_ground is Z=0 (contact point)
        # The visualization Z for the ball center is its radius.
        text_z_contact = f"Z_contact: {world_coords_3d[2]:.2f}m" # This should be 0.00m
        text_z_vis = f"Z_ball_center: {ball_radius_m:.2f}m (est.)"

        try:
            cv2.putText(frame, text_x, (10, frame.shape[0] - 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2) # Cyan
            cv2.putText(frame, text_y, (10, frame.shape[0] - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            cv2.putText(frame, text_z_contact, (10, frame.shape[0] - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            # cv2.putText(frame, text_z_vis, (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2) # If space
        except Exception as e:
             logging.error(f"CV: Error drawing 3D info on frame: {e}")
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
    if frame is None: return frame # Return frame if it's None
    if rvec is None or tvec is None or mtx is None or dist is None:
        # Optionally log this state if it's unexpected for axes not to be drawn
        # logging.debug("CV: Cannot draw axes due to missing rvec, tvec, mtx, or dist.")
        return frame

    # Define the 3D points for the axes (origin, X, Y, Z ends)
    # Origin is at (0,0,0) in world coordinates.
    axis_points_3d = np.float32([
        [0, 0, 0],                  # Origin
        [axis_length_m, 0, 0],      # X-axis end
        [0, axis_length_m, 0],      # Y-axis end
        [0, 0, axis_length_m]       # Z-axis end (pointing up from the ground)
    ]).reshape(-1, 3)

    try:
        # Project the 3D points to 2D image plane
        image_points, _ = cv2.projectPoints(axis_points_3d, rvec, tvec, mtx, dist)
        image_points = np.int32(image_points.reshape(-1, 2))

        origin_2d = tuple(image_points[0])
        x_axis_end_2d = tuple(image_points[1])
        y_axis_end_2d = tuple(image_points[2])
        z_axis_end_2d = tuple(image_points[3])

        # Draw lines for the axes
        # BGR colors: Blue for X, Green for Y, Red for Z (common convention)
        cv2.line(frame, origin_2d, x_axis_end_2d, (255, 0, 0), 3)  # Blue X
        cv2.line(frame, origin_2d, y_axis_end_2d, (0, 255, 0), 3)  # Green Y
        cv2.line(frame, origin_2d, z_axis_end_2d, (0, 0, 255), 3)  # Red Z

        # Optionally, put labels
        cv2.putText(frame, "X", (x_axis_end_2d[0] + 5, x_axis_end_2d[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,0,0), 2)
        cv2.putText(frame, "Y", (y_axis_end_2d[0] + 5, y_axis_end_2d[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
        cv2.putText(frame, "Z", (z_axis_end_2d[0] + 5, z_axis_end_2d[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)

    except Exception as e:
        logging.error(f"CV: Error drawing axes: {e}")

    return frame


if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')
    
    # Test block requires Config.py to be in the same directory or Python path
    # and specific config values to be set.
    
    # Example: Test undistortion
    # mock_frame = np.zeros((Config.CAM_REQUESTED_HEIGHT, Config.CAM_REQUESTED_WIDTH, 3), dtype=np.uint8) # Needs Config
    # cv2.putText(mock_frame, "Original Test Frame (Distorted)", (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255),2)
    # mock_mtx = Config.CAMERA_INTRINSIC_MTX 
    # mock_dist = Config.CAMERA_DIST_COEFFS 
    # if mock_mtx is not None and mock_dist is not None and mock_frame is not None:
    #     mock_new_mtx, _ = cv2.getOptimalNewCameraMatrix(mock_mtx, mock_dist, 
    #                                                   (mock_frame.shape[1], mock_frame.shape[0]), 1, 
    #                                                   (mock_frame.shape[1], mock_frame.shape[0]))
    #
    #     undistorted_test_frame = undistort_frame(mock_frame, mock_mtx, mock_dist, mock_new_mtx)
    #     if undistorted_test_frame is not None:
    #         cv2.imshow("Original", mock_frame)
    #         cv2.imshow("Undistorted Test", undistorted_test_frame)
    #         logging.info("Displaying original and undistorted test frames. Press any key to continue.")
    #         cv2.waitKey(0)
    #         cv2.destroyAllWindows()
    # else:
    #     logging.warning("CV Test: Could not run undistortion test due to missing mock data or Config values.")


    # Example: Test ball detection (requires a sample image with a ball and Config values)
    # sample_image_path = "path_to_your_sample_image_with_ball.jpg" 
    # try:
    #     test_ball_frame = cv2.imread(sample_image_path)
    #     if test_ball_frame is None: raise FileNotFoundError(f"File not found: {sample_image_path}")
    #
    #     # Ensure Config values used in detect_volleyball_2d are valid
    #     u_ball, v_ball, r_ball, dbg_mask = detect_volleyball_2d(test_ball_frame, roi=Config.BALL_DETECTION_ROI) # Needs Config
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
    # except FileNotFoundError as e:
    #     logging.warning(f"CV Test: Sample image for ball detection test not found. {e}")
    # except AttributeError as e:
    #     logging.error(f"CV Test: Error in ball detection test - likely missing Config attribute: {e}")
    # except Exception as e:
    #     logging.error(f"CV Test: Error in ball detection test: {e}")


    # Example: Test 2D to 3D mapping (requires Config values and calibrated parameters)
    # test_u, test_v = Config.CAM_REQUESTED_WIDTH // 2, Config.CAM_REQUESTED_HEIGHT // 2 # Needs Config
    #
    # test_mtx = Config.CAMERA_INTRINSIC_MTX # Needs Config
    # test_dist = Config.CAMERA_DIST_COEFFS # Needs Config
    # mock_rvec = np.array([0.0, 0.0, 0.0], dtype=np.float32) 
    # mock_tvec = np.array([0.0, 0.0, 2.0], dtype=np.float32) 
    #
    # if test_mtx is not None and test_dist is not None:
    #     try:
    #         # Load real calibration if available for a more meaningful test
    #         calib_data = np.load(Config.CALIBRATION_DATA_FILE) # Needs Config
    #         test_mtx = calib_data['camera_matrix']
    #         test_dist = calib_data['dist_coeffs']
    #         logging.info(f"Loaded real calibration data for 2D-3D test from {Config.CALIBRATION_DATA_FILE}")
    #     except FileNotFoundError:
    #         logging.warning(f"Calibration file {Config.CALIBRATION_DATA_FILE} not found. Using placeholder mtx/dist for 2D-3D test.")
    #     except KeyError:
    #         logging.warning(f"Calibration file {Config.CALIBRATION_DATA_FILE} has incorrect format. Using placeholder mtx/dist.")
    #     except Exception as e:
    #         logging.warning(f"Could not load calibration data for test: {e}")
    #
    #     coords_3d = map_2d_to_3d_on_ground(test_u, test_v, test_mtx, test_dist, mock_rvec, mock_tvec)
    #     if coords_3d is not None:
    #         logging.info(f"Test: Mapped 2D ({test_u},{test_v}) to 3D World ({coords_3d[0]:.3f}, {coords_3d[1]:.3f}, {coords_3d[2]:.3f})m")
    #     else:
    #         logging.warning("Test: 2D to 3D mapping failed.")
    # else:
    #     logging.warning("CV Test: Cannot run 2D to 3D mapping test due to missing Config mtx/dist.")

    logging.info("cv_functions.py standalone tests finished (commented out or depend on valid Config).")