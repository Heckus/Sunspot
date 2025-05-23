# main.py

import cv2
import numpy as np
import time
import logging
import threading
import os
import signal # For graceful shutdown

import Config
from CameraManager import CameraManager
import CvFunctions 
from WebUi import WebUIManager, OPEN3D_INITIALIZATION_FAILED, OPEN3D_AVAILABLE # Import new flags

# --- Global Variables ---
shutdown_event = threading.Event()
camera_manager = None
web_ui_manager = None
web_ui_thread = None # Keep a reference to the thread

# Camera extrinsic parameters (to be loaded or calibrated)
rvec_global = None
tvec_global = None
extrinsic_calibrated = False
EXTRINSIC_CALIB_FILE = os.path.join(Config.SCRIPT_DIR, "camera_extrinsic_data.npz")


def signal_handler(sig, frame):
    """Handles SIGINT and SIGTERM signals for graceful shutdown."""
    if shutdown_event.is_set():
        logging.warning(f"Shutdown already in progress (Signal {sig} received again).")
        return
    logging.warning(f"Signal {sig} received. Initiating graceful shutdown...")
    shutdown_event.set()

def setup_logging():
    """Configures the logging system."""
    log_level_str = Config.LOG_LEVEL.upper() if hasattr(Config, 'LOG_LEVEL') else "INFO"
    log_level_attr = getattr(logging, log_level_str, logging.INFO)
    
    logging.basicConfig(level=log_level_attr,
                        format=Config.LOG_FORMAT if hasattr(Config, 'LOG_FORMAT') else '%(asctime)s - %(levelname)s - [%(threadName)s:%(lineno)d] - %(message)s',
                        datefmt=Config.LOG_DATE_FORMAT if hasattr(Config, 'LOG_DATE_FORMAT') else '%Y-%m-%d %H:%M:%S',
                        handlers=[logging.StreamHandler()]) 
    
    logging.getLogger("werkzeug").setLevel(logging.WARNING) # Reduce Flask's default logging
    logging.getLogger("picamera2").setLevel(logging.WARNING) # Reduce Picamera2 INFO spam if not needed for debugging camera specifics

    app_title = Config.VIS_WINDOW_TITLE if hasattr(Config, 'VIS_WINDOW_TITLE') else "3D Volleyball Tracker"
    logging.info(f"--- Starting: {app_title} ---")
    logging.info(f"Log Level set to: {log_level_str}")


def load_extrinsic_calibration():
    """Loads extrinsic calibration data (rvec, tvec) from file."""
    global rvec_global, tvec_global, extrinsic_calibrated
    if os.path.exists(EXTRINSIC_CALIB_FILE):
        try:
            data = np.load(EXTRINSIC_CALIB_FILE)
            rvec_global = data['rvec']
            tvec_global = data['tvec']
            extrinsic_calibrated = True
            logging.info(f"Successfully loaded extrinsic calibration data from {EXTRINSIC_CALIB_FILE}")
            logging.debug(f"Loaded rvec:\n{rvec_global}")
            logging.debug(f"Loaded tvec:\n{tvec_global}")
            return True
        except Exception as e:
            logging.error(f"Error loading extrinsic calibration data from {EXTRINSIC_CALIB_FILE}: {e}", exc_info=True)
            rvec_global, tvec_global, extrinsic_calibrated = None, None, False
            return False
    logging.info(f"Extrinsic calibration file {EXTRINSIC_CALIB_FILE} not found. Calibration will be required.")
    return False

def save_extrinsic_calibration(rvec, tvec):
    """Saves extrinsic calibration data (rvec, tvec) to file."""
    try:
        np.savez(EXTRINSIC_CALIB_FILE, rvec=rvec, tvec=tvec)
        logging.info(f"Successfully saved extrinsic calibration data to {EXTRINSIC_CALIB_FILE}")
    except Exception as e:
        logging.error(f"Error saving extrinsic calibration data: {e}", exc_info=True)


# Global list to store clicked points for extrinsic calibration
clicked_image_points = []
MAX_CLICKS = 4 # For 4 corners of the box

def mouse_callback_extrinsic_calib(event, x, y, flags, param):
    """Mouse callback function to capture 4 points for extrinsic calibration."""
    global clicked_image_points
    frame_display = param['frame_display']
    window_name = param.get('window_name', "Extrinsic Calibration") # Get window name from params

    if event == cv2.EVENT_LBUTTONDOWN:
        if len(clicked_image_points) < MAX_CLICKS:
            clicked_image_points.append((x, y))
            # Draw a circle at the clicked point on the display frame
            cv2.circle(frame_display, (x,y), 7, (0,255,0), -1) # Larger circle
            cv2.circle(frame_display, (x,y), 7, (0,0,0), 1)   # Black outline
            cv2.putText(frame_display, f"P{len(clicked_image_points)}", (x+10, y-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (50,200,50), 2)
            
            # Update the specific window that the callback is tied to
            cv2.imshow(window_name, frame_display)
            logging.info(f"Clicked point {len(clicked_image_points)}: ({x}, {y})")
            if len(clicked_image_points) == MAX_CLICKS:
                logging.info("All 4 points collected. Press 'c' to calibrate or 'r' to restart points selection.")
        else:
            logging.info("Already collected 4 points. Press 'c' to calibrate, 'r' to restart, or 's' to skip.")


def perform_extrinsic_calibration(cam_manager_obj):
    """
    Performs extrinsic camera calibration using cv2.solvePnP().
    Args:
        cam_manager_obj (CameraManager): The camera manager instance.
    Returns:
        tuple: (rvec, tvec) or (None, None) if calibration fails or is skipped.
    """
    global clicked_image_points, rvec_global, tvec_global, extrinsic_calibrated

    if not cam_manager_obj or not cam_manager_obj.is_initialized:
        logging.error("Extrinsic Calib: Camera manager not valid or not initialized.")
        return None, None
    if cam_manager_obj.mtx is None or cam_manager_obj.dist is None:
        logging.error("Extrinsic Calib: Camera intrinsic parameters (mtx, dist) not loaded in camera manager.")
        return None, None

    logging.info("--- Starting Extrinsic Camera Calibration (Pose Estimation) ---")
    logging.info("Click on the 4 corners of the box in the 'Extrinsic Calibration' window.")
    logging.info("Order: 1. Origin (e.g., Bottom-Front-Left), 2. X-axis (Bottom-Front-Right), 3. X-Y (Bottom-Back-Right), 4. Y-axis (Bottom-Back-Left)")
    logging.info("Press 'c' to CALIBRATE after 4 points. 'r' to RESTART points. 's' to SKIP. 'q' to QUIT application.")

    window_name = "Extrinsic Calibration - Click Box Corners"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    # Sensible default size, user can resize
    cv2.resizeWindow(window_name, Config.CAM_REQUESTED_WIDTH // 2 if Config.CAM_REQUESTED_WIDTH > 0 else 960, 
                                  Config.CAM_REQUESTED_HEIGHT // 2 if Config.CAM_REQUESTED_HEIGHT > 0 else 540)

    frame_for_calibration = None
    # Capture a fresh frame for calibration
    for attempt in range(5): # Try a few times to get a good frame
        raw_frame = cam_manager_obj.capture_frame(undistort=False)
        if raw_frame is not None:
            frame_for_calibration = CvFunctions.undistort_frame(raw_frame, cam_manager_obj.mtx,
                                                                 cam_manager_obj.dist, cam_manager_obj.new_camera_mtx)
            if frame_for_calibration is not None:
                logging.info("Extrinsic Calib: Captured and undistorted frame for point selection.")
                break
            else:
                logging.warning(f"Extrinsic Calib: Undistortion failed (Attempt {attempt+1}). Retrying capture.")
        else:
            logging.warning(f"Extrinsic Calib: Failed to capture raw frame (Attempt {attempt+1}). Retrying.")
        time.sleep(0.2) # Wait a bit before retrying

    if frame_for_calibration is None:
        logging.error("Extrinsic Calib: Failed to get a usable camera frame after multiple attempts. Aborting calibration.")
        if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) >= 1:
            cv2.destroyWindow(window_name)
        return None, None

    frame_display = frame_for_calibration.copy()
    # Pass window_name to callback_params so it can update the correct window
    callback_params = {'frame_display': frame_display, 'window_name': window_name} 
    cv2.setMouseCallback(window_name, mouse_callback_extrinsic_calib, callback_params)
    clicked_image_points = [] # Reset points for this attempt

    while not shutdown_event.is_set():
        cv2.imshow(window_name, frame_display)
        key = cv2.waitKey(100) & 0xFF # Increased wait time for responsiveness

        if key == ord('q'): 
            logging.info("Quit requested during extrinsic calibration. Shutting down application.")
            shutdown_event.set()
            break 
        if key == ord('s'): 
            logging.info("Extrinsic calibration skipped by user.")
            rvec_global, tvec_global, extrinsic_calibrated = None, None, False # Explicitly mark as not calibrated
            break
        if key == ord('r'): 
            clicked_image_points = []
            frame_display = frame_for_calibration.copy() 
            callback_params['frame_display'] = frame_display # Update ref in params
            logging.info("Points reset. Re-click the 4 corners.")
            cv2.setMouseCallback(window_name, mouse_callback_extrinsic_calib, callback_params) # Re-set to ensure it has the latest frame_display

        if len(clicked_image_points) == MAX_CLICKS and key == ord('c'):
            image_points_np = np.array(clicked_image_points, dtype=np.float32)
            # Ensure WORLD_BOX_CORNERS_M is correctly defined and sliced if necessary for PnP
            object_points_np = Config.WORLD_BOX_CORNERS_M[:MAX_CLICKS] # Use only the number of points clicked

            logging.debug(f"Image Points (2D pixels) for solvePnP:\n{image_points_np}")
            logging.debug(f"Object Points (3D world meters) for solvePnP:\n{object_points_np}")

            try:
                # Use distortion coefficients from camera manager; ensure they are not None
                dist_coeffs_for_solvepnp = cam_manager_obj.dist if cam_manager_obj.dist is not None else np.zeros((1,5), dtype=np.float32)
                
                # solvePnP requires at least 4 points for many methods.
                # Using SOLVEPNP_ITERATIVE as a robust default if SQPNP has issues or fewer points.
                # SOLVEPNP_SQPNP is good for 4 coplanar points.
                # flags = cv2.SOLVEPNP_SQPNP (good for this case) or cv2.SOLVEPNP_IPPE (also for planar)
                # flags = cv2.SOLVEPNP_ITERATIVE (more general)
                
                # Using solvePnP which returns a single rvec/tvec for simplicity here
                # If multiple solutions are possible with solvePnPGeneric, logic would need to choose one.
                retval, rvec, tvec = cv2.solvePnP(
                    object_points_np, image_points_np,
                    cam_manager_obj.mtx, dist_coeffs_for_solvepnp,
                    flags=cv2.SOLVEPNP_SQPNP 
                )

                if retval:
                    # Reprojection error calculation (optional but good for validation)
                    projected_points, _ = cv2.projectPoints(object_points_np, rvec, tvec, cam_manager_obj.mtx, dist_coeffs_for_solvepnp)
                    error = cv2.norm(image_points_np, projected_points.reshape(-1,2), cv2.NORM_L2) / len(projected_points)
                    logging.info(f"solvePnP successful. Reprojection error: {error:.4f} pixels.")
                    logging.info(f"Rotation Vector (rvec):\n{rvec}")
                    logging.info(f"Translation Vector (tvec):\n{tvec}")

                    frame_with_axes = CvFunctions.draw_axes_on_frame(frame_for_calibration.copy(),
                                                                      rvec, tvec,
                                                                      cam_manager_obj.mtx, cam_manager_obj.dist)
                    cv2.putText(frame_with_axes, "SolvePnP OK. Press 'y' to accept, 'n' to retry.", (10,30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,200,0), 2)
                    cv2.imshow(window_name, frame_with_axes)

                    while not shutdown_event.is_set(): # Confirmation loop
                        confirm_key = cv2.waitKey(100) & 0xFF
                        if confirm_key == ord('y'):
                            rvec_global, tvec_global = rvec, tvec
                            extrinsic_calibrated = True
                            save_extrinsic_calibration(rvec, tvec)
                            logging.info("Extrinsic calibration accepted and saved.")
                            # This break is for the confirmation loop
                            break 
                        elif confirm_key == ord('n'):
                            logging.info("Extrinsic calibration rejected. Restarting point selection.")
                            clicked_image_points = [] 
                            frame_display = frame_for_calibration.copy()
                            callback_params['frame_display'] = frame_display
                            cv2.setMouseCallback(window_name, mouse_callback_extrinsic_calib, callback_params)
                            # This break is for the confirmation loop, will go back to main calib loop
                            break 
                        elif confirm_key == ord('q'):
                            logging.info("Quit requested during extrinsic calibration confirmation.")
                            shutdown_event.set()
                            break 
                    
                    if extrinsic_calibrated or shutdown_event.is_set(): # If 'y' was pressed or quit
                         break # Break from main calibration loop

                else:
                    logging.error("cv2.solvePnP failed (returned False).")
                    cv2.putText(frame_display, "SolvePnP FAILED. Press 'r' to retry.", (10,30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)

            except cv2.error as e: # Catch OpenCV specific errors
                logging.error(f"OpenCV Exception during solvePnP: {e}", exc_info=True)
                cv2.putText(frame_display, f"SolvePnP OpenCV Error! Press 'r'.", (10,30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)
            except Exception as e:
                logging.error(f"Generic Exception during solvePnP: {e}", exc_info=True)
                cv2.putText(frame_display, f"SolvePnP Generic Error! Press 'r'.", (10,30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)
        
        if extrinsic_calibrated: # If 'y' was pressed in confirmation
            break

    # Ensure window is closed if it was opened
    if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) >= 1:
        cv2.destroyWindow(window_name)
    
    if extrinsic_calibrated:
        return rvec_global, tvec_global
    return None, None # If skipped, quit, or failed


def main_loop():
    """Main operational loop for volleyball tracking."""
    global camera_manager, web_ui_manager, rvec_global, tvec_global, extrinsic_calibrated

    if not extrinsic_calibrated:
        logging.critical("Extrinsic parameters (rvec, tvec) are not available. Cannot proceed with 3D mapping.")
        shutdown_event.set() # Signal shutdown if critical calibration is missing
        return

    display_window_name = "2D Volleyball Tracking Overlay"
    if Config.DISPLAY_2D_FEED:
        cv2.namedWindow(display_window_name, cv2.WINDOW_AUTOSIZE) # AUTOSIZE or NORMAL
        # cv2.resizeWindow(display_window_name, 960, 540) # Optional fixed size

    frame_count = 0
    loop_start_time_total = time.monotonic()
    last_log_time = time.monotonic()

    while not shutdown_event.is_set():
        loop_iter_start_time = time.monotonic()

        raw_frame = camera_manager.capture_frame(undistort=False) # Capture raw
        if raw_frame is None:
            logging.warning("MainLoop: Failed to get frame from camera manager.")
            # Check if camera became uninitialized
            if not camera_manager.is_initialized:
                logging.error("MainLoop: CameraManager is no longer initialized. Shutting down.")
                shutdown_event.set()
                continue
            time.sleep(0.1) # Wait before retrying capture
            continue
        
        undistorted_frame = CvFunctions.undistort_frame(raw_frame, camera_manager.mtx,
                                                         camera_manager.dist, camera_manager.new_camera_mtx)
        if undistorted_frame is None:
            logging.warning("MainLoop: Frame became None after undistortion (original was not None). Using raw_frame for detection.")
            undistorted_frame = raw_frame.copy() # Fallback, though detection might be less accurate

        u, v, radius_px, dbg_mask = CvFunctions.detect_volleyball_2d(undistorted_frame,
                                                                      roi=Config.BALL_DETECTION_ROI)
        current_ball_2d = (u, v, radius_px) # Store tuple

        world_coords_3d_contact = None
        if u is not None and v is not None:
            world_coords_3d_contact = CvFunctions.map_2d_to_3d_on_ground(u, v,
                                                                  camera_manager.mtx, camera_manager.dist, 
                                                                  rvec_global, tvec_global)
        
        # For visualization, the ball's center is its radius above the contact point
        ball_pos_for_3d_vis = None
        if world_coords_3d_contact is not None:
            ball_pos_for_3d_vis = [world_coords_3d_contact[0], 
                                   world_coords_3d_contact[1], 
                                   world_coords_3d_contact[2] + Config.VOLLEYBALL_RADIUS_M] # Z is ground + radius

        # Update WebUI
        if web_ui_manager and Config.WEB_UI_ENABLED:
            cam_props = camera_manager.get_camera_properties()
            measured_fps_cam = cam_props.get('measured_fps', 0.0)
            resolution_text = f"{cam_props.get('frame_width',0)}x{cam_props.get('frame_height',0)}"

            web_ui_manager.update_data(
                raw_frame_for_stream=raw_frame, # Send raw frame for MJPEG
                ball_2d_coords=current_ball_2d[:2] if u is not None else None,
                ball_pixel_radius=radius_px,
                ball_3d_world_coords=ball_pos_for_3d_vis, # This is center of ball for 3D vis
                camera_intrinsics=camera_manager.mtx.tolist() if camera_manager.mtx is not None else "N/A",
                camera_extrinsics={'rvec': rvec_global.tolist() if rvec_global is not None else "N/A",
                                   'tvec': tvec_global.tolist() if tvec_global is not None else "N/A"},
                fps=measured_fps_cam if measured_fps_cam is not None else 0.0,
                resolution=resolution_text,
                status_message="Tracking..." if u is not None else "Searching for ball..."
            )

        # Display 2D Feed
        if Config.DISPLAY_2D_FEED:
            # Start with a fresh copy of the undistorted frame for drawing
            display_frame = undistorted_frame.copy() 
            
            if Config.LOG_LEVEL.upper() == "DEBUG" and dbg_mask is not None:
                 # Ensure dbg_mask is BGR and same size if adding weighted
                 if dbg_mask.shape[:2] == display_frame.shape[:2]:
                    if len(dbg_mask.shape) == 2: # if grayscale
                        dbg_mask_bgr = cv2.cvtColor(dbg_mask, cv2.COLOR_GRAY2BGR)
                    else: # already BGR
                        dbg_mask_bgr = dbg_mask
                    display_frame = cv2.addWeighted(display_frame, 0.7, dbg_mask_bgr, 0.3, 0)
                 else:
                    logging.warning("Debug mask size mismatch, cannot overlay.")


            if u is not None and v is not None and radius_px is not None:
                display_frame = CvFunctions.draw_ball_on_frame(display_frame, u, v, radius_px)
            
            if world_coords_3d_contact is not None: # Display contact point coordinates
                display_frame = CvFunctions.draw_3d_info_on_frame(display_frame, world_coords_3d_contact, Config.VOLLEYBALL_RADIUS_M)
            
            display_frame = CvFunctions.draw_axes_on_frame(display_frame, rvec_global, tvec_global,
                                                            camera_manager.mtx, camera_manager.dist)
            
            # Display processing FPS on the 2D window
            loop_duration_瞬时 = time.monotonic() - loop_iter_start_time
            processing_fps_瞬时 = 1.0 / loop_duration_瞬时 if loop_duration_瞬时 > 0 else 0
            cv2.putText(display_frame, f"Proc FPS: {processing_fps_瞬时:.1f}", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2) # Yellow text

            cv2.imshow(display_window_name, display_frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                logging.info("User pressed 'q' in 2D display window. Shutting down.")
                shutdown_event.set()
            elif key == ord('e'): # Re-run extrinsic calibration
                logging.info("User requested extrinsic calibration re-run from main loop.")
                old_rvec, old_tvec, old_extrinsic_calibrated = rvec_global, tvec_global, extrinsic_calibrated
                
                # Temporarily hide main display window to avoid conflicts
                cv2.destroyWindow(display_window_name) 
                
                new_rvec, new_tvec = perform_extrinsic_calibration(camera_manager)
                if new_rvec is not None and new_tvec is not None:
                    rvec_global, tvec_global = new_rvec, new_tvec
                    extrinsic_calibrated = True
                    logging.info("Extrinsic calibration updated from main loop.")
                else:
                    logging.info("Extrinsic calibration re-run was skipped or failed. Restoring previous values.")
                    rvec_global, tvec_global, extrinsic_calibrated = old_rvec, old_tvec, old_extrinsic_calibrated
                
                # Recreate the main display window if it was destroyed
                if Config.DISPLAY_2D_FEED and not shutdown_event.is_set(): # check shutdown again
                    cv2.namedWindow(display_window_name, cv2.WINDOW_AUTOSIZE)


        frame_count += 1
        # Main loop FPS control (capping)
        loop_duration = time.monotonic() - loop_iter_start_time
        if Config.MAIN_LOOP_MAX_FPS > 0:
            sleep_time = (1.0 / Config.MAIN_LOOP_MAX_FPS) - loop_duration
            if sleep_time > 0:
                time.sleep(sleep_time)
        
        # Log average FPS periodically
        if time.monotonic() - last_log_time > 10.0: # Log every 10 seconds
            current_total_duration = time.monotonic() - loop_start_time_total
            avg_loop_fps_so_far = frame_count / current_total_duration if current_total_duration > 0 else 0
            logging.debug(f"Average Main Loop FPS so far: {avg_loop_fps_so_far:.2f} ({frame_count} frames)")
            last_log_time = time.monotonic()


    total_duration = time.monotonic() - loop_start_time_total
    avg_loop_fps = frame_count / total_duration if total_duration > 0 else 0
    logging.info(f"Main loop finished. Processed {frame_count} frames in {total_duration:.2f}s. Overall Avg FPS: {avg_loop_fps:.2f}")
    
    if Config.DISPLAY_2D_FEED and cv2.getWindowProperty(display_window_name, cv2.WND_PROP_VISIBLE) >=1 :
        cv2.destroyAllWindows()

def main():
    global camera_manager, web_ui_manager, web_ui_thread, rvec_global, tvec_global, extrinsic_calibrated

    # Setup logging and signal handlers first
    setup_logging()
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Initialize CameraManager
    camera_manager = CameraManager(camera_index=Config.CAMERA_INDEX,
                                   width=Config.CAM_REQUESTED_WIDTH,
                                   height=Config.CAM_REQUESTED_HEIGHT,
                                   fps=Config.CAM_REQUESTED_FPS)

    if not camera_manager.load_calibration_data(Config.CALIBRATION_DATA_FILE):
        logging.error("Failed to load camera intrinsic calibration data. Cannot proceed.")
        if not os.path.exists(Config.CALIBRATION_DATA_FILE):
            logging.error(f"Ensure '{Config.CALIBRATION_DATA_FILE}' exists. Run camera_calibration.py if needed.")
        if camera_manager: camera_manager.shutdown()
        return # Exit if intrinsics fail

    if not camera_manager.initialize_camera():
        logging.error("Failed to initialize camera. Cannot proceed.")
        if camera_manager: camera_manager.shutdown()
        return # Exit if camera init fails

    # Extrinsic Calibration
    if not load_extrinsic_calibration():
        logging.warning("Extrinsic calibration data not found or failed to load. Attempting to perform live calibration.")
        # Give user a chance to prepare for calibration
        logging.info("Prepare for extrinsic calibration. Press Enter to continue or 's' to skip (not recommended)...")
        try:
            # Simple console input to pause before calibration window.
            # This is tricky with OpenCV windows potentially already active or planned.
            # For now, direct call. User should see logs.
            pass
        except: pass # Ignore if input fails (e.g. not interactive)
        
        rvec_calib, tvec_calib = perform_extrinsic_calibration(camera_manager)
        if rvec_calib is not None and tvec_calib is not None:
            rvec_global, tvec_global = rvec_calib, tvec_calib
            extrinsic_calibrated = True
            logging.info("Extrinsic calibration completed successfully.")
        elif shutdown_event.is_set(): # if 'q' was pressed during calib
            logging.warning("Application shutdown requested during extrinsic calibration.")
        else: # Skipped or failed without quit
            logging.error("Extrinsic calibration was skipped or failed. 3D mapping will not be accurate or available.")
            # Decide if application should exit if extrinsics are critical.
            # For now, we allow it to continue but 3D parts will be affected.
            extrinsic_calibrated = False 
    
    if shutdown_event.is_set(): # Check if quit was pressed during extrinsic calib.
        logging.info("Shutting down after extrinsic calibration phase due to user request.")
        if camera_manager: camera_manager.shutdown()
        return

    # Start Web UI if enabled and Open3D didn't have critical failures during its import/initial check
    if Config.WEB_UI_ENABLED:
        if OPEN3D_AVAILABLE and not OPEN3D_INITIALIZATION_FAILED:
            logging.info("Open3D library is available. WebUI will attempt to use it.")
        elif not OPEN3D_AVAILABLE:
            logging.warning("Open3D library NOT available. WebUI 3D features will be disabled.")
        elif OPEN3D_INITIALIZATION_FAILED:
             logging.warning("Open3D library available BUT INITIALIZATION FAILED (e.g. GLX context). WebUI 3D features will be disabled.")

        web_ui_manager = WebUIManager(
            host='0.0.0.0', # Listen on all interfaces
            port=Config.WEB_PORT,
            shutdown_event_ref=shutdown_event # Pass the main shutdown event
        )
        web_ui_thread = threading.Thread(target=web_ui_manager.run, name="WebUIServerThread", daemon=True)
        web_ui_thread.start()
        time.sleep(1) # Give the server a moment to start
        if not web_ui_thread.is_alive():
            logging.error("Web UI server thread failed to start. Check for port conflicts or other errors in WebUIManager.")
            # Depending on criticality, could set shutdown_event.set() here.
    else:
        logging.info("Web UI is disabled via Config.WEB_UI_ENABLED.")


    # Enter main processing loop if not already shutting down
    if not shutdown_event.is_set():
        try:
            main_loop()
        except KeyboardInterrupt: 
            logging.info("KeyboardInterrupt in main_loop. Setting shutdown event.")
            shutdown_event.set()
        except Exception as e:
            logging.error(f"Unhandled exception in main_loop: {e}", exc_info=True)
            shutdown_event.set() # Ensure shutdown on unexpected error

    # --- Cleanup ---
    logging.info("--- Main: Initiating Final Cleanup Sequence ---")
    if not shutdown_event.is_set(): # Ensure event is set if loop exited for other reasons
        shutdown_event.set()

    # Stop Web UI Manager and join its thread
    if web_ui_manager and Config.WEB_UI_ENABLED:
        logging.info("Main: Signaling WebUI Manager to stop...")
        if hasattr(web_ui_manager, 'stop') and callable(web_ui_manager.stop):
             web_ui_manager.stop() # This should use the shutdown_event primarily

        if web_ui_thread and web_ui_thread.is_alive():
            logging.info("Main: Waiting for WebUI server thread to join...")
            web_ui_thread.join(timeout=5.0) # Wait for the thread to finish
            if web_ui_thread.is_alive():
                logging.warning("Main: WebUI server thread did not exit cleanly after join().")
        else:
            logging.info("Main: WebUI thread was not alive or not started.")
    
    # Shutdown Camera Manager
    if camera_manager:
        logging.info("Main: Shutting down Camera Manager...")
        camera_manager.shutdown()

    app_title_final = Config.VIS_WINDOW_TITLE if hasattr(Config, 'VIS_WINDOW_TITLE') else "Application"
    logging.info(f"--- {app_title_final} Stopped ---")
    
    # Ensure all handlers are flushed, especially if there were file handlers.
    logging.shutdown() 
    
    # A small delay can sometimes help ensure all daemon threads (like Flask's internal ones)
    # have a moment to recognize the shutdown if they weren't perfectly joined.
    # time.sleep(0.1) # Usually not necessary if threads are joined properly.

if __name__ == "__main__":
    main()