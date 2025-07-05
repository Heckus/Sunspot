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
    mask_opened = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask_morphed = cv2.morphologyEx(mask_opened, cv2.MORPH_CLOSE, kernel, iterations=2)

    if DEBUG_MASKS:
        cv2.imshow("Morphed (Final) Mask", mask_morphed)

    contours, _ = cv2.findContours(mask_morphed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    ball_center_u, ball_center_v, ball_radius_px = None, None, None
    valid_contours_found = False

    if contours:
        valid_contours = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if Config.MIN_BALL_CONTOUR_AREA < area < Config.MAX_BALL_CONTOUR_AREA:
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
                    valid_contours_found = True
                    logging.debug(f"CV: Ball candidate detected at ({ball_center_u}, {ball_center_v}), radius ~{ball_radius_px}px.")

    final_mask_for_display_gray = np.zeros(undistorted_frame.shape[:2], dtype=np.uint8)
    if offset_x > 0 or offset_y > 0:
        final_mask_for_display_gray[offset_y:offset_y+frame_to_process.shape[0], offset_x:offset_x+frame_to_process.shape[1]] = mask_morphed
    else:
        final_mask_for_display_gray = mask_morphed
    
    dbg_mask_output = cv2.cvtColor(final_mask_for_display_gray, cv2.COLOR_GRAY2BGR)

    if not valid_contours_found:
        return None, None, None, dbg_mask_output

    return ball_center_u, ball_center_v, ball_radius_px, dbg_mask_output


def estimate_3d_position_from_radius(u, v, radius_px, mtx, dist, rvec, tvec):
    """
    Estimates the 3D position of the ball based on its apparent radius in pixels.
    This is the new core function for true 3D tracking.
    """
    if u is None or v is None or radius_px is None or radius_px <= 1:
        return None
    if mtx is None or dist is None or rvec is None or tvec is None:
        logging.error("CV: Missing camera parameters for 3D estimation.")
        return None

    try:
        # Get the focal lengths from the camera matrix [fx, fy]
        fx = mtx[0, 0]
        fy = mtx[1, 1]
        focal_length_px = (fx + fy) / 2.0  # Use an average focal length

        # --- Step 1: Estimate the distance (Z-depth in camera coordinates) to the ball ---
        # Formula: Distance = (Real_Object_Radius * Focal_Length_in_Pixels) / Apparent_Radius_in_Pixels
        distance_cam_z = (Config.VOLLEYBALL_RADIUS_M * focal_length_px) / radius_px
        logging.debug(f"CV: Estimated distance to ball (camera Z-depth): {distance_cam_z:.2f}m")

        # --- Step 2: Find the 3D point in the camera's coordinate system ---
        # First, convert the 2D pixel (u, v) to a normalized 3D ray from the camera
        cx = mtx[0, 2]
        cy = mtx[1, 2]
        # Undistort the single point (u,v) to get a more accurate direction vector
        undistorted_point = cv2.undistortPoints(np.array([[[float(u), float(v)]]], dtype=np.float32), mtx, dist, P=mtx)
        u_undistorted, v_undistorted = undistorted_point[0,0,0], undistorted_point[0,0,1]

        # Calculate the X and Y coordinates in the camera's 3D space
        cam_x = (u_undistorted - cx) / fx * distance_cam_z
        cam_y = (v_undistorted - cy) / fy * distance_cam_z
        
        point_in_cam_coords = np.array([cam_x, cam_y, distance_cam_z])
        logging.debug(f"CV: Ball position in camera coordinates: {point_in_cam_coords}")

        # --- Step 3: Convert the 3D point from camera coordinates to world coordinates ---
        # solvePnP gives rvec and tvec that transform WORLD points to CAMERA points:
        # P_camera = R * P_world + t
        # To go from camera to world, we must invert this transformation:
        # P_world = R_inverse * (P_camera - t)
        # Since R is a rotation matrix, R_inverse is R_transpose.
        R, _ = cv2.Rodrigues(rvec)
        R_transpose = R.T
        
        # Reshape for matrix multiplication
        point_in_cam_coords_col = point_in_cam_coords.reshape(3, 1)
        tvec_col = tvec.reshape(3, 1)

        world_coords = R_transpose @ (point_in_cam_coords_col - tvec_col)
        
        # Flatten the result to a 1D array [x, y, z]
        world_coords_final = world_coords.flatten()
        logging.debug(f"CV: Calculated 3D world coordinates: {world_coords_final}")
        
        return world_coords_final

    except Exception as e:
        logging.error(f"CV: Error during 3D position estimation: {e}", exc_info=True)
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

def draw_3d_info_on_frame(frame, world_coords_3d):
    """Modified to show the true 3D position."""
    if frame is None: return frame
    if world_coords_3d is not None:
        text_x = f"X: {world_coords_3d[0]:.2f}m"
        text_y = f"Y: {world_coords_3d[1]:.2f}m"
        text_z = f"Z: {world_coords_3d[2]:.2f}m" # Changed from Z_contact

        try:
            cv2.putText(frame, text_x, (10, frame.shape[0] - 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.putText(frame, text_y, (10, frame.shape[0] - 45), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.putText(frame, text_z, (10, frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
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
