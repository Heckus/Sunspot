# cv_functions.py

import cv2
import numpy as np
import logging
import Config # Import the new config file

# --- Add a flag for debugging masks ---
DEBUG_MASKS = False # Set to True to display masks during detect_volleyball_2d

def undistort_frame(frame, mtx, dist, new_camera_mtx):
    if mtx is None or dist is None:
        logging.warning("CV: Camera matrix or distortion coefficients not available. Skipping undistortion.")
        return frame
    
    current_new_mtx = new_camera_mtx if new_camera_mtx is not None else mtx

    try:
        undistorted_img = cv2.undistort(frame, mtx, dist, None, current_new_mtx)
        return undistorted_img
    except Exception as e:
        logging.error(f"CV: Error during undistortion: {e}", exc_info=True)
        return frame


def detect_volleyball_2d(undistorted_frame, roi=None):
    global DEBUG_MASKS
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
            else:
                logging.warning(f"CV: Invalid ROI {roi} for frame shape {undistorted_frame.shape[:2]}. Processing full frame.")
        except Exception as e:
            logging.warning(f"CV: Error applying ROI {roi}: {e}. Processing full frame.")

    try:
        hsv_frame = cv2.cvtColor(frame_to_process, cv2.COLOR_BGR2HSV)
    except Exception as e:
        logging.error(f"CV: Failed to convert frame to HSV: {e}", exc_info=True)
        return None, None, None, None

    mask_yellow = cv2.inRange(hsv_frame, Config.LOWER_YELLOW_HSV, Config.UPPER_YELLOW_HSV)
    mask_blue = cv2.inRange(hsv_frame, Config.LOWER_BLUE_HSV, Config.UPPER_BLUE_HSV)
    combined_mask = cv2.bitwise_or(mask_yellow, mask_blue)

    kernel = np.ones(Config.MORPH_KERNEL_SIZE, np.uint8)
    # Apply MORPH_OPEN (erode then dilate) to remove small noise specks
    mask_opened = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel, iterations=1)
    # Apply MORPH_CLOSE (dilate then erode) to close small holes in the main object
    mask_morphed = cv2.morphologyEx(mask_opened, cv2.MORPH_CLOSE, kernel, iterations=2) # Iterations can be tuned

    # --- Debugging Section: Display Masks ---
    if DEBUG_MASKS:
        cv2.imshow("HSV Frame (ROI)", hsv_frame)
        cv2.imshow("Yellow Mask", mask_yellow)
        cv2.imshow("Blue Mask", mask_blue)
        cv2.imshow("Combined Mask", combined_mask)
        cv2.imshow("Opened Mask", mask_opened)
        cv2.imshow("Morphed (Final) Mask", mask_morphed)
    # --- End Debugging Section ---

    contours, _ = cv2.findContours(mask_morphed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    ball_center_u, ball_center_v, ball_radius_px = None, None, None
    valid_contours_found = False # Flag

    if contours:
        valid_contours = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if Config.MIN_BALL_CONTOUR_AREA < area < Config.MAX_BALL_CONTOUR_AREA:
                # Additional check: circularity (optional, but can help)
                # perimeter = cv2.arcLength(cnt, True)
                # if perimeter > 0:
                #     circularity = 4 * np.pi * (area / (perimeter * perimeter))
                #     if 0.6 < circularity < 1.4: # Adjust threshold for "circle-likeness"
                #         valid_contours.append(cnt)
                # else:
                #     valid_contours.append(cnt) # If perimeter is 0, it's a point, skip circularity check
                valid_contours.append(cnt)


        if valid_contours:
            ball_contour = max(valid_contours, key=cv2.contourArea)
            ((u_roi, v_roi), radius_px_roi) = cv2.minEnclosingCircle(ball_contour)

            if Config.MIN_BALL_RADIUS_PX <= radius_px_roi <= Config.MAX_BALL_RADIUS_PX:
                M = cv2.moments(ball_contour)
                if M["m00"] != 0:
                    centroid_u_roi = int(M["m10"] / M["m00"])
                    centroid_v_roi = int(M["m01"] / M["m00"])

                    ball_center_u = centroid_u_roi + offset_x
                    ball_center_v = centroid_v_roi + offset_y
                    ball_radius_px = int(radius_px_roi)
                    valid_contours_found = True # Mark that we found a candidate
                    logging.debug(f"CV: Ball candidate detected at ({ball_center_u}, {ball_center_v}), radius ~{ball_radius_px}px.")
                else:
                    logging.debug("CV: Valid ball contour found but m00 is zero (cannot compute centroid).")
            else:
                logging.debug(f"CV: Largest valid contour's radius {radius_px_roi:.1f}px out of range ({Config.MIN_BALL_RADIUS_PX}-{Config.MAX_BALL_RADIUS_PX}px).")
        else:
            logging.debug("CV: No contours passed area filtering criteria.")
    else:
        logging.debug("CV: No contours found in morphed mask.")

    # Prepare the mask for display output (dbg_mask in main.py)
    # This should be the final mask_morphed, converted to BGR, full size if ROI was used.
    final_mask_for_display_gray = np.zeros(undistorted_frame.shape[:2], dtype=np.uint8)
    if offset_x > 0 or offset_y > 0 : # If ROI was used
        final_mask_for_display_gray[offset_y:offset_y+frame_to_process.shape[0], offset_x:offset_x+frame_to_process.shape[1]] = mask_morphed
    else: # No ROI
        final_mask_for_display_gray = mask_morphed
    
    # Convert the grayscale mask to BGR for consistent display overlay
    dbg_mask_output = cv2.cvtColor(final_mask_for_display_gray, cv2.COLOR_GRAY2BGR)

    if not valid_contours_found:
        logging.debug("CV: No valid volleyball detected in the current frame.")
        return None, None, None, dbg_mask_output # Return current mask for debugging even if no ball

    return ball_center_u, ball_center_v, ball_radius_px, dbg_mask_output


def map_2d_to_3d_on_ground(u, v, mtx, dist, rvec, tvec):
    if u is None or v is None:
        logging.debug("CV: map_2d_to_3d received None for u or v.")
        return None
    if mtx is None or dist is None or rvec is None or tvec is None:
        logging.error("CV: Missing camera parameters (mtx, dist, rvec, tvec) for 2D to 3D mapping.")
        return None

    try:
        image_point_2d = np.array([[[float(u), float(v)]]], dtype=np.float32)
        normalized_points = cv2.undistortPoints(image_point_2d, mtx, dist, P=None)
        xn = normalized_points[0,0,0]
        yn = normalized_points[0,0,1]
        logging.debug(f"CV: Pixel ({u},{v}) -> Normalized ({xn:.4f},{yn:.4f})")

        ray_direction_camera = np.array([xn, yn, 1.0], dtype=np.float32)
        
        R_cw, _ = cv2.Rodrigues(rvec) 
        R_wc = R_cw.T 
        camera_origin_world = -R_wc @ tvec.reshape(3)
        ray_direction_world = R_wc @ ray_direction_camera

        logging.debug(f"CV: Ray Origin (Camera Pos in World): {camera_origin_world}")
        logging.debug(f"CV: Ray Direction in World: {ray_direction_world}")
        
        if abs(ray_direction_world[2]) < 1e-6:
            logging.warning("CV: Ray is parallel or near parallel to the ground plane. Cannot intersect.")
            return None

        k = -camera_origin_world[2] / ray_direction_world[2]
        logging.debug(f"CV: Solved k for ray-plane intersection: {k:.4f}")

        # Note: k < 0 means intersection is "behind" camera plane along ray.
        # For ball on ground in front, k should generally be positive.
        # However, the definition of "behind" depends on camera orientation (rvec, tvec).
        # If a point is visible, k should usually be positive.
        # If you get negative k for visible points, review extrinsic calibration (rvec/tvec)
        # or how the camera is oriented relative to the world origin.

        X_ball = camera_origin_world[0] + k * ray_direction_world[0]
        Y_ball = camera_origin_world[1] + k * ray_direction_world[1]
        Z_ball = 0.0 

        world_coords = np.array([X_ball, Y_ball, Z_ball], dtype=np.float32)
        logging.info(f"CV: Mapped 2D ({u},{v}) to 3D World ({X_ball:.3f}, {Y_ball:.3f}, {Z_ball:.3f})m")
        return world_coords

    except Exception as e:
        logging.error(f"CV: Error during 2D to 3D mapping: {e}", exc_info=True)
        return None


def draw_ball_on_frame(frame, u, v, radius_px):
    if frame is None: return frame
    if u is not None and v is not None and radius_px is not None:
        try:
            cv2.circle(frame, (int(u), int(v)), int(radius_px), (0, 255, 0), 2) 
            cv2.circle(frame, (int(u), int(v)), 3, (0, 0, 255), -1) 
        except Exception as e:
            logging.error(f"CV: Error drawing ball on frame: {e}", exc_info=True)
    return frame

def draw_3d_info_on_frame(frame, world_coords_3d, ball_radius_m):
    if frame is None: return frame
    if world_coords_3d is not None:
        text_x = f"X: {world_coords_3d[0]:.2f}m"
        text_y = f"Y: {world_coords_3d[1]:.2f}m"
        text_z_contact = f"Z_contact: {world_coords_3d[2]:.2f}m"

        try:
            cv2.putText(frame, text_x, (10, frame.shape[0] - 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.putText(frame, text_y, (10, frame.shape[0] - 45), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.putText(frame, text_z_contact, (10, frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        except Exception as e:
             logging.error(f"CV: Error drawing 3D info on frame: {e}", exc_info=True)
    return frame

def draw_axes_on_frame(frame, rvec, tvec, mtx, dist, axis_length_m=0.5):
    if frame is None: return frame
    if rvec is None or tvec is None or mtx is None or dist is None:
        return frame

    axis_points_3d = np.float32([
        [0, 0, 0], [axis_length_m, 0, 0], [0, axis_length_m, 0], [0, 0, axis_length_m]
    ]).reshape(-1, 3)

    try:
        image_points, _ = cv2.projectPoints(axis_points_3d, rvec, tvec, mtx, dist)
        image_points = np.int32(image_points.reshape(-1, 2))

        origin_2d, x_axis_end_2d, y_axis_end_2d, z_axis_end_2d = \
            tuple(image_points[0]), tuple(image_points[1]), tuple(image_points[2]), tuple(image_points[3])

        cv2.line(frame, origin_2d, x_axis_end_2d, (255, 0, 0), 2)  # Blue X
        cv2.line(frame, origin_2d, y_axis_end_2d, (0, 255, 0), 2)  # Green Y
        cv2.line(frame, origin_2d, z_axis_end_2d, (0, 0, 255), 2)  # Red Z

        cv2.putText(frame, "X", (x_axis_end_2d[0] + 5, x_axis_end_2d[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 2)
        cv2.putText(frame, "Y", (y_axis_end_2d[0] + 5, y_axis_end_2d[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
        cv2.putText(frame, "Z", (z_axis_end_2d[0] + 5, z_axis_end_2d[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
    except Exception as e:
        logging.error(f"CV: Error drawing axes: {e}", exc_info=True)
    return frame


if __name__ == '__main__':
    # To test CvFunctions with your image:
    # 1. Make sure Config.py is in the same directory or Python path.
    # 2. Set DEBUG_MASKS = True at the top of this file.
    # 3. Put your 'volleyball.jpg' in the same directory.
    # 4. Uncomment and run the block below.

    logging.basicConfig(level=logging.DEBUG, format=Config.LOG_FORMAT, datefmt=Config.LOG_DATE_FORMAT)
    DEBUG_MASKS = True # Override for standalone testing
    
    # Load the test image
    test_image_path = 'volleyball.jpg' # Make sure this image is in the same directory
    test_frame = cv2.imread(test_image_path)

    if test_frame is None:
        logging.error(f"Could not load test image: {test_image_path}")
    else:
        logging.info(f"Test image {test_image_path} loaded successfully.")
        # Mock camera matrix and distortion for testing detect_volleyball_2d (undistortion won't be real)
        # These should ideally come from your actual calibration file if you want to test undistortion too
        h, w = test_frame.shape[:2]
        mock_mtx = np.array([[w, 0, w/2], [0, h, h/2], [0,0,1]], dtype=float) # Focal length approx image width/height
        mock_dist = np.zeros((1,5), dtype=float) # Assume no distortion for this isolated test
        
        # Since detect_volleyball_2d expects an undistorted frame, we can pass the original if not testing undistortion
        # Or, pass it through undistort_frame with mock parameters (will effectively do nothing if dist is zero)
        undistorted_test_frame = undistort_frame(test_frame, mock_mtx, mock_dist, mock_mtx)

        logging.info("Detecting volleyball in test image...")
        u_ball, v_ball, r_ball, dbg_mask_out = detect_volleyball_2d(undistorted_test_frame, roi=None)

        if u_ball is not None:
            logging.info(f"Test: Ball detected at ({u_ball}, {v_ball}) with radius {r_ball}px.")
            draw_ball_on_frame(undistorted_test_frame, u_ball, v_ball, r_ball)
        else:
            logging.info("Test: Ball NOT detected.")

        # Display the original image with detection overlay
        cv2.imshow("Ball Detection Test on Image", undistorted_test_frame)
        
        # The dbg_mask_out from detect_volleyball_2d is the final mask used for contour detection
        if dbg_mask_out is not None:
             cv2.imshow("Returned Debug Mask (from detect_volleyball_2d)", dbg_mask_out)

        logging.info("Displaying test results. Press 'q' in any OpenCV window to quit.")
        
        # Keep windows open until 'q' is pressed
        while True:
            key = cv2.waitKey(100) & 0xFF
            if key == ord('q'):
                break
            # Check if any window was closed by user
            if cv2.getWindowProperty("Ball Detection Test on Image", cv2.WND_PROP_VISIBLE) < 1:
                break
            if DEBUG_MASKS: # if debug windows were created
                if cv2.getWindowProperty("Morphed (Final) Mask", cv2.WND_PROP_VISIBLE) < 1:
                    break


        cv2.destroyAllWindows()
        logging.info("cv_functions.py standalone test finished.")