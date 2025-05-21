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
import CvFunctions # Ensure this matches the filename and an __init__.py if it's a package
from WebUi import WebUIManager # Assuming web_ui.py will have a WebUIManager class
# from hardware_manager import HardwareManager # Placeholder if needed

# --- Global Variables ---
shutdown_event = threading.Event()
camera_manager = None
web_ui_manager = None
# hardware_manager = None # Placeholder

# Camera extrinsic parameters (to be loaded or calibrated)
rvec_global = None
tvec_global = None
extrinsic_calibrated = False
EXTRINSIC_CALIB_FILE = os.path.join(Config.SCRIPT_DIR, "camera_extrinsic_data.npz")


def signal_handler(sig, frame):
    """Handles SIGINT and SIGTERM signals for graceful shutdown."""
    if shutdown_event.is_set():
        logging.warning("Shutdown already in progress, ignoring additional signal.")
        return
    logging.warning(f"Received signal {sig}. Initiating graceful shutdown...")
    shutdown_event.set()

def setup_logging():
    """Configures the logging system."""
    log_level_attr = getattr(logging, Config.LOG_LEVEL.upper(), logging.INFO)
    logging.basicConfig(level=log_level_attr,
                        format=Config.LOG_FORMAT,
                        datefmt=Config.LOG_DATE_FORMAT,
                        handlers=[logging.StreamHandler()]) # Ensure logs go to console
    logging.getLogger("werkzeug").setLevel(logging.WARNING) # Reduce Flask's default logging
    if hasattr(Config, 'VIS_WINDOW_TITLE'): # Check if attribute exists
        logging.info(f"--- Starting: {Config.VIS_WINDOW_TITLE} ---")
    else:
        logging.info("--- Starting: 3D Volleyball Tracker ---")


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
            logging.info(f"Loaded rvec:\n{rvec_global}")
            logging.info(f"Loaded tvec:\n{tvec_global}")
            return True
        except Exception as e:
            logging.error(f"Error loading extrinsic calibration data: {e}", exc_info=True)
            rvec_global, tvec_global, extrinsic_calibrated = None, None, False
            return False
    logging.info(f"Extrinsic calibration file {EXTRINSIC_CALIB_FILE} not found.")
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

    if event == cv2.EVENT_LBUTTONDOWN:
        if len(clicked_image_points) < MAX_CLICKS:
            clicked_image_points.append((x, y))
            # Draw a circle at the clicked point on the display frame
            cv2.circle(frame_display, (x,y), 5, (0,255,0), -1)
            cv2.putText(frame_display, f"P{len(clicked_image_points)}", (x+10, y-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
            cv2.imshow("Extrinsic Calibration - Click Box Corners", frame_display)
            logging.info(f"Clicked point {len(clicked_image_points)}: ({x}, {y})")
            if len(clicked_image_points) == MAX_CLICKS:
                logging.info("All 4 points collected. Press 'c' to calibrate or 'r' to restart.")
        else:
            logging.info("Already collected 4 points. Press 'c' to calibrate or 'r' to restart.")


def perform_extrinsic_calibration(cam_manager_obj):
    """
    Performs extrinsic camera calibration using cv2.solvePnP().
    This function guides the user to click on the 4 corners of the 2x2m box.
    Args:
        cam_manager_obj (CameraManager): The camera manager instance.
    Returns:
        tuple: (rvec, tvec) or (None, None) if calibration fails or is skipped.
    """
    global clicked_image_points, rvec_global, tvec_global, extrinsic_calibrated

    if not cam_manager_obj.is_initialized:
        logging.error("Extrinsic Calib: Camera not initialized.")
        return None, None
    if cam_manager_obj.mtx is None or cam_manager_obj.dist is None:
        logging.error("Extrinsic Calib: Camera intrinsic parameters (mtx, dist) not loaded.")
        return None, None

    logging.info("--- Starting Extrinsic Camera Calibration (Pose Estimation) ---")
    logging.info("Please ensure the 2m x 2m box is clearly visible.")
    logging.info("Click on the 4 corners of the box in the 'Extrinsic Calibration' window.")
    logging.info("Order: 1. Origin (e.g., Bottom-Front-Left as defined in Config)")
    logging.info("       2. Corner along X-axis (e.g., Bottom-Front-Right)")
    logging.info("       3. Corner along X then Y (e.g., Bottom-Back-Right)")
    logging.info("       4. Corner along Y-axis (e.g., Bottom-Back-Left)")
    logging.info("Press 'c' to calculate pose after 4 points. 'r' to restart points. 's' to skip.")

    window_name = "Extrinsic Calibration - Click Box Corners"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 1280, 720) # Adjust size as needed

    frame_for_calibration = None
    max_frame_wait_attempts = 100 # Approx 10 seconds if sleep is 0.1s
    frame_wait_attempts = 0

    while frame_for_calibration is None and not shutdown_event.is_set():
        raw_frame = cam_manager_obj.capture_frame(undistort=False) # Get raw frame
        if raw_frame is not None:
            # Undistort this specific frame for accurate point selection
            # Corrected module name from cv_functions to CvFunctions
            frame_for_calibration = CvFunctions.undistort_frame(raw_frame, cam_manager_obj.mtx,
                                                                 cam_manager_obj.dist, cam_manager_obj.new_camera_mtx)
            if frame_for_calibration is None:
                 logging.warning("Extrinsic Calib: Raw frame captured, but undistortion failed. Retrying capture.")
                 # Consider a small delay here if undistortion failure is intermittent and recoverable
                 # time.sleep(0.05)
        else:
            logging.warning(f"Extrinsic Calib: Waiting for a valid camera frame... (Attempt {frame_wait_attempts + 1}/{max_frame_wait_attempts})")
            time.sleep(0.1)
            frame_wait_attempts += 1
            if frame_wait_attempts >= max_frame_wait_attempts:
                logging.error("Extrinsic Calib: Failed to get a camera frame after multiple attempts. Aborting calibration attempt.")
                if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) >= 1 : # Check if window exists and is visible
                    cv2.destroyWindow(window_name)
                return None, None # Exit calibration

        # Allow skipping even while waiting for the first frame
        # Place key check inside the loop to make it responsive
        key_event = cv2.waitKey(1) & 0xFF
        if key_event == ord('s'):
            logging.info("Extrinsic calibration skipped by user while waiting for frame.")
            if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) >= 1:
                 cv2.destroyWindow(window_name)
            return None, None
        elif key_event == ord('q'): # Allow quitting application
            logging.info("Quit requested during extrinsic calibration setup.")
            shutdown_event.set()
            if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) >= 1:
                cv2.destroyWindow(window_name)
            return None, None


    if frame_for_calibration is None: # Could happen if shutdown_event was set
        logging.info("Extrinsic calibration aborted due to shutdown or failure to get frame.")
        if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) >= 1:
            cv2.destroyWindow(window_name)
        return None, None

    frame_display = frame_for_calibration.copy()
    callback_params = {'frame_display': frame_display}
    cv2.setMouseCallback(window_name, mouse_callback_extrinsic_calib, callback_params)

    clicked_image_points = [] # Reset points

    while not shutdown_event.is_set():
        cv2.imshow(window_name, frame_display)
        key = cv2.waitKey(100) & 0xFF

        if key == ord('q'): # Allow quitting application
            logging.info("Quit requested during extrinsic calibration point selection.")
            shutdown_event.set()
            break
        if key == ord('s'): # Skip
            logging.info("Extrinsic calibration skipped by user.")
            break
        if key == ord('r'): # Restart points
            clicked_image_points = []
            frame_display = frame_for_calibration.copy() # Reset display frame
            callback_params['frame_display'] = frame_display
            logging.info("Points reset. Re-click the 4 corners.")
            # Re-assign the mouse callback in case the window context needs fresh frame_display
            cv2.setMouseCallback(window_name, mouse_callback_extrinsic_calib, callback_params)


        if len(clicked_image_points) == MAX_CLICKS and key == ord('c'):
            image_points_np = np.array(clicked_image_points, dtype=np.float32)
            object_points_np = Config.WORLD_BOX_CORNERS_M

            logging.info(f"Image Points (2D pixels) for solvePnP:\n{image_points_np}")
            logging.info(f"Object Points (3D world meters) for solvePnP:\n{object_points_np}")

            try:
                dist_coeffs_for_solvepnp = cam_manager_obj.dist if cam_manager_obj.dist is not None else np.zeros((5,1), dtype=np.float32)
                retval, rvecs, tvecs, reprojection_errors = cv2.solvePnPGeneric(
                    object_points_np, image_points_np,
                    cam_manager_obj.mtx, dist_coeffs_for_solvepnp,
                    flags=cv2.SOLVEPNP_SQPNP
                )

                if retval and rvecs is not None and len(rvecs) > 0:
                    rvec = rvecs[0]
                    tvec = tvecs[0]
                    if reprojection_errors is not None and len(reprojection_errors) > 0:
                        logging.info(f"solvePnP reprojection error: {reprojection_errors[0]}")

                    logging.info("cv2.solvePnP successful.")
                    logging.info(f"Rotation Vector (rvec):\n{rvec}")
                    logging.info(f"Translation Vector (tvec):\n{tvec}")

                    # Draw projected axes for verification
                    # Corrected module name from cv_functions to CvFunctions
                    frame_with_axes = CvFunctions.draw_axes_on_frame(frame_for_calibration.copy(),
                                                                      rvec, tvec,
                                                                      cam_manager_obj.mtx, cam_manager_obj.dist)
                    cv2.putText(frame_with_axes, "SolvePnP OK. Press 'y' to accept, 'n' to retry.", (10,30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
                    cv2.imshow(window_name, frame_with_axes)

                    while not shutdown_event.is_set():
                        confirm_key = cv2.waitKey(100) & 0xFF
                        if confirm_key == ord('y'):
                            rvec_global, tvec_global = rvec, tvec
                            extrinsic_calibrated = True
                            save_extrinsic_calibration(rvec, tvec)
                            logging.info("Extrinsic calibration accepted and saved.")
                            cv2.destroyWindow(window_name)
                            return rvec, tvec
                        elif confirm_key == ord('n'):
                            logging.info("Extrinsic calibration rejected. Restarting point selection.")
                            clicked_image_points = [] # Reset points
                            frame_display = frame_for_calibration.copy() # Reset display frame
                            callback_params['frame_display'] = frame_display
                            cv2.setMouseCallback(window_name, mouse_callback_extrinsic_calib, callback_params)
                            break # Break from confirmation loop to point selection loop
                        elif confirm_key == ord('q'):
                            logging.info("Quit requested during extrinsic calibration confirmation.")
                            shutdown_event.set()
                            break # Break from confirmation loop
                    if extrinsic_calibrated or shutdown_event.is_set(): break # Break from main calib loop if accepted or shutdown

                else:
                    logging.error("cv2.solvePnP failed or returned no valid solutions.")
                    cv2.putText(frame_display, "SolvePnP FAILED. Press 'r' to retry points.", (10,30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)

            except Exception as e:
                logging.error(f"Exception during solvePnP: {e}", exc_info=True)
                cv2.putText(frame_display, f"SolvePnP Error. Press 'r' to retry.", (10,30), # Simplified error for display
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)

    if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) >= 1:
        cv2.destroyWindow(window_name)
    if extrinsic_calibrated:
        return rvec_global, tvec_global
    return None, None


def main_loop():
    """Main operational loop for volleyball tracking."""
    global camera_manager, web_ui_manager, rvec_global, tvec_global, extrinsic_calibrated

    if not extrinsic_calibrated:
        logging.error("Extrinsic parameters (rvec, tvec) are not available. Cannot proceed with 3D mapping.")
        logging.info("Consider running extrinsic calibration or ensuring the data file is present.")
        # No longer setting shutdown_event here, main() will handle exit if perform_extrinsic_calibration failed critically.
        return

    display_window_name = "2D Volleyball Tracking"
    if Config.DISPLAY_2D_FEED:
        cv2.namedWindow(display_window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(display_window_name, 960, 540)

    frame_count = 0
    loop_start_time_total = time.monotonic()

    while not shutdown_event.is_set():
        loop_iter_start_time = time.monotonic()

        raw_frame = camera_manager.capture_frame(undistort=False)
        if raw_frame is None:
            logging.warning("MainLoop: Failed to get frame from camera manager.")
            if camera_manager.last_error and ("Cannot open camera" in camera_manager.last_error or "Failed to read frame" in camera_manager.last_error):
                logging.critical("Camera connection issue persisting or failed to grab frame. Consider shutting down or attempting re-initialization.")
                # Potentially add a counter for consecutive errors before shutting down
                # For now, just sleep and continue hoping it recovers.
            time.sleep(0.1) 
            continue
        
        # Corrected module name from cv_functions to CvFunctions
        undistorted_frame = CvFunctions.undistort_frame(raw_frame, camera_manager.mtx,
                                                         camera_manager.dist, camera_manager.new_camera_mtx)
        if undistorted_frame is None:
            logging.warning("MainLoop: Frame became None after undistortion.")
            time.sleep(0.1)
            continue

        # Corrected module name from cv_functions to CvFunctions
        u, v, radius_px, dbg_mask = CvFunctions.detect_volleyball_2d(undistorted_frame,
                                                                      roi=Config.BALL_DETECTION_ROI)
        current_ball_2d = (u,v,radius_px)

        world_coords_3d = None
        if u is not None and v is not None:
            # Corrected module name from cv_functions to CvFunctions
            world_coords_3d = CvFunctions.map_2d_to_3d_on_ground(u, v,
                                                                  camera_manager.mtx, camera_manager.dist, # mtx and dist should be from camera_manager
                                                                  rvec_global, tvec_global)
        current_ball_3d = world_coords_3d

        if web_ui_manager and Config.WEB_UI_ENABLED:
            ball_pos_for_vis = None
            if current_ball_3d is not None:
                ball_pos_for_vis = [current_ball_3d[0], current_ball_3d[1], Config.VOLLEYBALL_RADIUS_M]

            cam_props = camera_manager.get_camera_properties()
            measured_fps = cam_props.get('measured_fps', 0)
            resolution_text = f"{cam_props.get('frame_width',0)}x{cam_props.get('frame_height',0)}"

            web_ui_manager.update_data(
                raw_frame_for_stream=raw_frame,
                processed_frame_for_stream=None,
                ball_2d_coords=current_ball_2d[:2] if current_ball_2d[0] is not None else None,
                ball_pixel_radius=current_ball_2d[2] if current_ball_2d[2] is not None else None,
                ball_3d_world_coords=ball_pos_for_vis,
                box_corners_world=Config.WORLD_BOX_CORNERS_M.tolist(),
                box_dimensions=[Config.BOX_WIDTH_M, Config.BOX_DEPTH_M, Config.BOX_HEIGHT_M],
                camera_intrinsics=camera_manager.mtx.tolist() if camera_manager.mtx is not None else None,
                camera_extrinsics={'rvec': rvec_global.tolist() if rvec_global is not None else None,
                                   'tvec': tvec_global.tolist() if tvec_global is not None else None},
                fps=measured_fps if measured_fps is not None else 0.0,
                resolution=resolution_text,
                status_message="Tracking..." if u is not None else "Searching for ball..."
            )

        if Config.DISPLAY_2D_FEED:
            display_frame = undistorted_frame.copy()
            if dbg_mask is not None and Config.LOG_LEVEL.upper() == "DEBUG":
                 display_frame = cv2.addWeighted(display_frame, 0.7, dbg_mask, 0.3, 0)

            if u is not None:
                # Corrected module name from cv_functions to CvFunctions
                display_frame = CvFunctions.draw_ball_on_frame(display_frame, u, v, radius_px)
            if world_coords_3d is not None:
                # Corrected module name from cv_functions to CvFunctions
                display_frame = CvFunctions.draw_3d_info_on_frame(display_frame, world_coords_3d, Config.VOLLEYBALL_RADIUS_M)
            
            # Corrected module name from cv_functions to CvFunctions
            display_frame = CvFunctions.draw_axes_on_frame(display_frame, rvec_global, tvec_global,
                                                            camera_manager.mtx, camera_manager.dist)
            cv2.imshow(display_window_name, display_frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                shutdown_event.set()
            elif key == ord('e'):
                logging.info("User requested extrinsic calibration re-run.")
                # Store current state in case re-calibration is skipped/failed
                old_rvec, old_tvec = rvec_global, tvec_global
                old_extrinsic_calibrated = extrinsic_calibrated

                new_rvec, new_tvec = perform_extrinsic_calibration(camera_manager)
                if new_rvec is not None and new_tvec is not None:
                    rvec_global, tvec_global = new_rvec, new_tvec
                    extrinsic_calibrated = True
                    logging.info("Extrinsic calibration updated.")
                else:
                    logging.info("Extrinsic calibration re-run was skipped or failed. Restoring previous values if any.")
                    rvec_global, tvec_global = old_rvec, old_tvec # Restore previous
                    extrinsic_calibrated = old_extrinsic_calibrated # Restore previous status


        frame_count += 1
        loop_duration = time.monotonic() - loop_iter_start_time
        if Config.MAIN_LOOP_MAX_FPS > 0:
            sleep_time = (1.0 / Config.MAIN_LOOP_MAX_FPS) - loop_duration
            if sleep_time > 0:
                time.sleep(sleep_time)

    total_duration = time.monotonic() - loop_start_time_total
    avg_loop_fps = frame_count / total_duration if total_duration > 0 else 0
    logging.info(f"Main loop finished. Processed {frame_count} frames in {total_duration:.2f}s. Avg FPS: {avg_loop_fps:.2f}")
    if Config.DISPLAY_2D_FEED and cv2.getWindowProperty(display_window_name, cv2.WND_PROP_VISIBLE) >=1:
        cv2.destroyAllWindows() # More robust than destroyWindow

def main():
    global camera_manager, web_ui_manager, rvec_global, tvec_global, extrinsic_calibrated

    setup_logging()
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    camera_manager = CameraManager(camera_index=Config.CAMERA_INDEX,
                                   width=Config.CAM_REQUESTED_WIDTH,
                                   height=Config.CAM_REQUESTED_HEIGHT,
                                   fps=Config.CAM_REQUESTED_FPS)

    if not camera_manager.load_calibration_data(Config.CALIBRATION_DATA_FILE):
        logging.error("Failed to load camera intrinsic calibration data. Cannot proceed without intrinsics.")
        if not os.path.exists(Config.CALIBRATION_DATA_FILE):
            logging.error(f"Please run the camera_calibration.py script first to generate '{Config.CALIBRATION_DATA_FILE}'.")
        camera_manager.shutdown()
        return

    if not camera_manager.initialize_camera():
        logging.error("Failed to initialize camera. Cannot proceed.")
        camera_manager.shutdown()
        return

    if not load_extrinsic_calibration():
        logging.warning("Could not load extrinsic calibration. Attempting to perform it now.")
        rvec_calib, tvec_calib = perform_extrinsic_calibration(camera_manager)
        if rvec_calib is None or tvec_calib is None:
            logging.error("Extrinsic calibration failed, was skipped, or quit. Cannot proceed accurately with 3D mapping.")
            # No need to set shutdown_event here, main will exit.
        else:
            rvec_global, tvec_calib = rvec_calib, tvec_calib # Typo: tvec_global = tvec_calib
            extrinsic_calibrated = True
    else:
        logging.info("Extrinsic calibration loaded successfully.")
        # Optionally, could ask user if they want to re-calibrate
        # For now, assume loaded is fine.

    if not extrinsic_calibrated and not shutdown_event.is_set():
        logging.critical("Extrinsic calibration is required for 3D mapping but was not successful. Application will exit.")
        shutdown_event.set()


    if Config.WEB_UI_ENABLED and not shutdown_event.is_set():
        web_ui_manager = WebUIManager(
            host='0.0.0.0',
            port=Config.WEB_PORT,
            shutdown_event_ref=shutdown_event
        )
        web_thread = threading.Thread(target=web_ui_manager.run, name="WebUIServerThread", daemon=True)
        web_thread.start()
        time.sleep(1) 
        if not web_thread.is_alive():
            logging.error("Web UI server thread failed to start. Check for port conflicts or other errors.")
            # shutdown_event.set() # Optionally shut down if web UI is critical


    if not shutdown_event.is_set():
        try:
            main_loop()
        except Exception as e:
            logging.error(f"Unhandled exception in main_loop: {e}", exc_info=True)
            shutdown_event.set()
        except KeyboardInterrupt: 
            logging.info("KeyboardInterrupt in main_loop. Shutting down.")
            shutdown_event.set()

    logging.info("--- Initiating Final Cleanup Sequence ---")
    if not shutdown_event.is_set(): # Ensure it's set if loop exited for other reasons
        shutdown_event.set()

    if web_ui_manager and Config.WEB_UI_ENABLED and 'web_thread' in locals() and web_thread.is_alive():
        logging.info("Shutting down Web UI Manager...")
        # Web UI manager observes shutdown_event. Give it time to react.
        web_thread.join(timeout=5.0)
        if web_thread.is_alive():
            logging.warning("Web UI server thread did not exit cleanly. Attempting direct stop.")
            if hasattr(web_ui_manager, 'stop'): # Check if stop method exists
                web_ui_manager.stop()
        # Check again after stop
        if web_thread.is_alive():
             logging.error("Web UI server thread still alive after stop attempt.")


    if camera_manager:
        camera_manager.shutdown()

    logging.info("========================================================")
    logging.info(f"--- {(Config.VIS_WINDOW_TITLE if hasattr(Config, 'VIS_WINDOW_TITLE') else 'Application')} Stopped ---")
    logging.info("========================================================")
    
    logging.shutdown()
    # time.sleep(0.5) # Removed os._exit as it's generally not good practice unless absolutely necessary

if __name__ == "__main__":
    main()