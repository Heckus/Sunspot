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

    cv2.namedWindow("Extrinsic Calibration - Click Box Corners", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Extrinsic Calibration - Click Box Corners", 1280, 720) # Adjust size as needed

    frame_for_calibration = None
    while frame_for_calibration is None and not shutdown_event.is_set():
        raw_frame = cam_manager_obj.capture_frame(undistort=False) # Get raw frame
        if raw_frame is not None:
            # Undistort this specific frame for accurate point selection
            frame_for_calibration = cv_functions.undistort_frame(raw_frame, cam_manager_obj.mtx,
                                                                 cam_manager_obj.dist, cam_manager_obj.new_camera_mtx)
        else:
            logging.warning("Extrinsic Calib: Waiting for a valid camera frame...")
            time.sleep(0.1)
        if cv2.waitKey(1) & 0xFF == ord('s'):
            logging.info("Extrinsic calibration skipped by user.")
            cv2.destroyWindow("Extrinsic Calibration - Click Box Corners")
            return None, None


    frame_display = frame_for_calibration.copy()
    callback_params = {'frame_display': frame_display}
    cv2.setMouseCallback("Extrinsic Calibration - Click Box Corners", mouse_callback_extrinsic_calib, callback_params)

    clicked_image_points = [] # Reset points

    while not shutdown_event.is_set():
        cv2.imshow("Extrinsic Calibration - Click Box Corners", frame_display)
        key = cv2.waitKey(100) & 0xFF

        if key == ord('s'): # Skip
            logging.info("Extrinsic calibration skipped by user.")
            cv2.destroyWindow("Extrinsic Calibration - Click Box Corners")
            return None, None
        if key == ord('r'): # Restart points
            clicked_image_points = []
            frame_display = frame_for_calibration.copy() # Reset display frame
            callback_params['frame_display'] = frame_display
            logging.info("Points reset. Re-click the 4 corners.")
            cv2.setMouseCallback("Extrinsic Calibration - Click Box Corners", mouse_callback_extrinsic_calib, callback_params)


        if len(clicked_image_points) == MAX_CLICKS and key == ord('c'):
            image_points_np = np.array(clicked_image_points, dtype=np.float32)
            object_points_np = Config.WORLD_BOX_CORNERS_M

            logging.info(f"Image Points (2D pixels) for solvePnP:\n{image_points_np}")
            logging.info(f"Object Points (3D world meters) for solvePnP:\n{object_points_np}")

            try:
                # Ensure distortion coeffs are not None, use zeros if they are (though they should be loaded)
                dist_coeffs_for_solvepnp = cam_manager_obj.dist if cam_manager_obj.dist is not None else np.zeros((5,1), dtype=np.float32)

                # retval, rvec, tvec = cv2.solvePnP(object_points_np, image_points_np,
                #                                  cam_manager_obj.mtx, dist_coeffs_for_solvepnp,
                #                                  flags=cv2.SOLVEPNP_ITERATIVE)
                # Use SOLVEPNP_IPPE_SQUARE if corners are ordered correctly relative to center,
                # or SOLVEPNP_SQPNP for a robust planar solution. Iterative is generally good.
                retval, rvecs, tvecs, reprojection_errors = cv2.solvePnPGeneric(
                    object_points_np, image_points_np,
                    cam_manager_obj.mtx, dist_coeffs_for_solvepnp,
                    flags=cv2.SOLVEPNP_SQPNP # A robust PnP for planar targets
                )


                if retval and rvecs is not None and len(rvecs) > 0:
                    # solvePnPGeneric can return multiple solutions, take the first one / one with lowest error
                    rvec = rvecs[0] # Take the first solution
                    tvec = tvecs[0]
                    if reprojection_errors is not None and len(reprojection_errors) > 0:
                        logging.info(f"solvePnP reprojection error: {reprojection_errors[0]}")


                    logging.info("cv2.solvePnP successful.")
                    logging.info(f"Rotation Vector (rvec):\n{rvec}")
                    logging.info(f"Translation Vector (tvec):\n{tvec}")

                    # Draw projected axes for verification
                    frame_with_axes = cv_functions.draw_axes_on_frame(frame_for_calibration.copy(),
                                                                      rvec, tvec,
                                                                      cam_manager_obj.mtx, cam_manager_obj.dist)
                    cv2.putText(frame_with_axes, "SolvePnP OK. Press 'y' to accept, 'n' to retry.", (10,30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
                    cv2.imshow("Extrinsic Calibration - Click Box Corners", frame_with_axes)

                    while not shutdown_event.is_set():
                        confirm_key = cv2.waitKey(100) & 0xFF
                        if confirm_key == ord('y'):
                            rvec_global, tvec_global = rvec, tvec
                            extrinsic_calibrated = True
                            save_extrinsic_calibration(rvec, tvec)
                            logging.info("Extrinsic calibration accepted and saved.")
                            cv2.destroyWindow("Extrinsic Calibration - Click Box Corners")
                            return rvec, tvec
                        elif confirm_key == ord('n'):
                            logging.info("Extrinsic calibration rejected. Restarting point selection.")
                            clicked_image_points = [] # Reset points
                            frame_display = frame_for_calibration.copy() # Reset display frame
                            callback_params['frame_display'] = frame_display
                            cv2.setMouseCallback("Extrinsic Calibration - Click Box Corners", mouse_callback_extrinsic_calib, callback_params)
                            break # Break from confirmation loop to point selection loop
                    if extrinsic_calibrated: break # Break from main calib loop if accepted

                else:
                    logging.error("cv2.solvePnP failed or returned no valid solutions.")
                    cv2.putText(frame_display, "SolvePnP FAILED. Press 'r' to retry points.", (10,30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)

            except Exception as e:
                logging.error(f"Exception during solvePnP: {e}", exc_info=True)
                cv2.putText(frame_display, f"SolvePnP Error: {e}. Press 'r' to retry.", (10,30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)

    cv2.destroyAllWindows()
    return None, None


def main_loop():
    """Main operational loop for volleyball tracking."""
    global camera_manager, web_ui_manager, rvec_global, tvec_global, extrinsic_calibrated

    if not extrinsic_calibrated:
        logging.error("Extrinsic parameters (rvec, tvec) are not available. Cannot proceed with 3D mapping.")
        logging.info("Consider running extrinsic calibration or ensuring the data file is present.")
        shutdown_event.set() # Stop if no pose
        return

    if Config.DISPLAY_2D_FEED:
        cv2.namedWindow("2D Volleyball Tracking", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("2D Volleyball Tracking", 960, 540)

    frame_count = 0
    loop_start_time_total = time.monotonic()

    # Variables for web UI update
    last_ball_2d = (None, None, None) # u, v, radius
    last_ball_3d = None # X, Y, Z_vis

    while not shutdown_event.is_set():
        loop_iter_start_time = time.monotonic()

        # 1. Capture Frame
        raw_frame = camera_manager.capture_frame(undistort=False) # Get raw frame
        if raw_frame is None:
            logging.warning("MainLoop: Failed to get frame from camera manager.")
            if camera_manager.last_error and "Cannot open camera" in camera_manager.last_error:
                logging.critical("Camera connection lost. Shutting down.")
                shutdown_event.set()
            time.sleep(0.1) # Wait a bit before retrying
            continue

        # 2. Undistort Frame
        # The camera_manager.capture_frame can do this, or we can do it here explicitly.
        # For clarity, let's assume camera_manager.capture_frame(undistort=True) is preferred
        # or we call cv_functions.undistort_frame here.
        # undistorted_frame = camera_manager.capture_frame(undistort=True) # Simpler if CM handles it
        undistorted_frame = cv_functions.undistort_frame(raw_frame, camera_manager.mtx,
                                                         camera_manager.dist, camera_manager.new_camera_mtx)
        if undistorted_frame is None:
            logging.warning("MainLoop: Frame became None after undistortion.")
            time.sleep(0.1)
            continue

        # 3. Detect Volleyball 2D
        # Pass ROI from Config if set
        u, v, radius_px, dbg_mask = cv_functions.detect_volleyball_2d(undistorted_frame,
                                                                      roi=Config.BALL_DETECTION_ROI)
        current_ball_2d = (u,v,radius_px)

        # 4. 2D to 3D Mapping
        world_coords_3d = None
        if u is not None and v is not None:
            world_coords_3d = cv_functions.map_2d_to_3d_on_ground(u, v,
                                                                  camera_manager.mtx, camera_manager.dist,
                                                                  rvec_global, tvec_global)
        current_ball_3d = world_coords_3d

        # --- Update Web UI Data ---
        if web_ui_manager and Config.WEB_UI_ENABLED:
            # For the 3D model of the ball, its center should be at (X, Y, BallRadius)
            ball_pos_for_vis = None
            if current_ball_3d is not None:
                ball_pos_for_vis = [current_ball_3d[0], current_ball_3d[1], Config.VOLLEYBALL_RADIUS_M]

            # Prepare other variables for UI
            cam_props = camera_manager.get_camera_properties()
            measured_fps = cam_props.get('measured_fps', 0)
            resolution_text = f"{cam_props.get('frame_width',0)}x{cam_props.get('frame_height',0)}"

            # Update data in WebUIManager
            web_ui_manager.update_data(
                raw_frame_for_stream=raw_frame, # Send raw frame for web stream
                processed_frame_for_stream=None, # Placeholder for CV processed frame if needed for separate stream
                ball_2d_coords=current_ball_2d[:2] if current_ball_2d[0] is not None else None,
                ball_pixel_radius=current_ball_2d[2] if current_ball_2d[2] is not None else None,
                ball_3d_world_coords=ball_pos_for_vis, # This is (X,Y,Radius) for sphere center
                box_corners_world=Config.WORLD_BOX_CORNERS_M.tolist(), # For drawing the box
                box_dimensions=[Config.BOX_WIDTH_M, Config.BOX_DEPTH_M, Config.BOX_HEIGHT_M],
                camera_intrinsics=camera_manager.mtx.tolist() if camera_manager.mtx is not None else None,
                camera_extrinsics={'rvec': rvec_global.tolist() if rvec_global is not None else None,
                                   'tvec': tvec_global.tolist() if tvec_global is not None else None},
                fps=measured_fps,
                resolution=resolution_text,
                status_message="Tracking..." if u is not None else "Searching for ball..."
            )
            # The Web UI itself will handle the Open3D updates in its own thread/loop based on this data.


        # --- Display 2D Feed (Optional) ---
        if Config.DISPLAY_2D_FEED:
            display_frame = undistorted_frame.copy()
            if dbg_mask is not None and Config.LOG_LEVEL == "DEBUG": # Show mask only in debug
                 display_frame = cv2.addWeighted(display_frame, 0.7, dbg_mask, 0.3, 0)

            if u is not None:
                display_frame = cv_functions.draw_ball_on_frame(display_frame, u, v, radius_px)
            if world_coords_3d is not None:
                display_frame = cv_functions.draw_3d_info_on_frame(display_frame, world_coords_3d, Config.VOLLEYBALL_RADIUS_M)

            # Draw world axes if extrinsics are available
            display_frame = cv_functions.draw_axes_on_frame(display_frame, rvec_global, tvec_global,
                                                            camera_manager.mtx, camera_manager.dist)
            cv2.imshow("2D Volleyball Tracking", display_frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                shutdown_event.set()
            elif key == ord('e'): # Add a key to re-run extrinsic calibration
                logging.info("User requested extrinsic calibration re-run.")
                new_rvec, new_tvec = perform_extrinsic_calibration(camera_manager)
                if new_rvec is not None and new_tvec is not None:
                    rvec_global, tvec_global = new_rvec, new_tvec
                    extrinsic_calibrated = True
                    logging.info("Extrinsic calibration updated.")
                else:
                    logging.info("Extrinsic calibration re-run was skipped or failed.")


        frame_count += 1
        loop_duration = time.monotonic() - loop_iter_start_time
        if Config.MAIN_LOOP_MAX_FPS > 0:
            sleep_time = (1.0 / Config.MAIN_LOOP_MAX_FPS) - loop_duration
            if sleep_time > 0:
                time.sleep(sleep_time)

    # --- End of Main Loop ---
    total_duration = time.monotonic() - loop_start_time_total
    avg_loop_fps = frame_count / total_duration if total_duration > 0 else 0
    logging.info(f"Main loop finished. Processed {frame_count} frames in {total_duration:.2f}s. Avg FPS: {avg_loop_fps:.2f}")
    if Config.DISPLAY_2D_FEED:
        cv2.destroyAllWindows()

def main():
    global camera_manager, web_ui_manager, rvec_global, tvec_global, extrinsic_calibrated
    # hardware_manager placeholder

    setup_logging()
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Initialize Camera Manager
    camera_manager = CameraManager(camera_index=Config.CAMERA_INDEX,
                                   width=Config.CAM_REQUESTED_WIDTH,
                                   height=Config.CAM_REQUESTED_HEIGHT,
                                   fps=Config.CAM_REQUESTED_FPS)

    # Load intrinsic calibration
    if not camera_manager.load_calibration_data(Config.CALIBRATION_DATA_FILE):
        logging.error("Failed to load camera intrinsic calibration data. Cannot proceed.")
        if not os.path.exists(Config.CALIBRATION_DATA_FILE):
            logging.error(f"Please run the camera_calibration.py script first to generate '{Config.CALIBRATION_DATA_FILE}'.")
        camera_manager.shutdown()
        return

    if not camera_manager.initialize_camera():
        logging.error("Failed to initialize camera. Cannot proceed.")
        camera_manager.shutdown()
        return

    # Load or Perform Extrinsic Calibration
    if not load_extrinsic_calibration():
        logging.warning("Could not load extrinsic calibration. Attempting to perform it now.")
        rvec_calib, tvec_calib = perform_extrinsic_calibration(camera_manager)
        if rvec_calib is None or tvec_calib is None:
            logging.error("Extrinsic calibration failed or was skipped. Cannot proceed accurately with 3D mapping.")
            # Decide if to exit or continue with degraded functionality (no 3D mapping)
            # For this project, 3D mapping is key, so we might exit.
            shutdown_event.set() # Signal shutdown
        else:
            rvec_global, tvec_global = rvec_calib, tvec_calib
            extrinsic_calibrated = True
    else: # Loaded successfully
        # Optionally, could ask user if they want to re-calibrate
        # For now, assume loaded is fine.
        pass


    # Initialize Hardware Manager (if needed for battery, etc.)
    # hardware_manager = HardwareManager() # Basic init

    # Initialize and Start Web UI Manager in a separate thread
    if Config.WEB_UI_ENABLED:
        web_ui_manager = WebUIManager(
            host='0.0.0.0',
            port=Config.WEB_PORT,
            shutdown_event_ref=shutdown_event # Pass shutdown event
        )
        web_thread = threading.Thread(target=web_ui_manager.run, name="WebUIServerThread", daemon=True)
        web_thread.start()
        time.sleep(1) # Give server a moment to start
        if not web_thread.is_alive():
            logging.error("Web UI server thread failed to start. Check for port conflicts or other errors.")
            # Optionally shut down if web UI is critical
            # shutdown_event.set()


    # Start the main processing loop
    if not shutdown_event.is_set(): # Only start if no critical init errors
        try:
            main_loop()
        except Exception as e:
            logging.error(f"Unhandled exception in main_loop: {e}", exc_info=True)
            shutdown_event.set() # Trigger shutdown on unhandled error
        except KeyboardInterrupt: # Should be caught by signal_handler, but as a fallback
            logging.info("KeyboardInterrupt in main_loop. Shutting down.")
            shutdown_event.set()

    # --- Cleanup ---
    logging.info("--- Initiating Final Cleanup Sequence ---")
    if not shutdown_event.is_set(): # Ensure it's set if loop exited for other reasons
        shutdown_event.set()

    if web_ui_manager and Config.WEB_UI_ENABLED:
        logging.info("Shutting down Web UI Manager...")
        # Web UI manager should observe shutdown_event, or have its own stop method
        # If using Flask's development server, it might need a request to stop
        # or os.kill(os.getpid(), signal.SIGINT) if run in main thread and daemon=False
        # For a threaded server, just joining the thread after signaling might be enough
        if 'web_thread' in locals() and web_thread.is_alive():
            logging.info("Waiting for Web UI server thread to join...")
            web_thread.join(timeout=5.0)
            if web_thread.is_alive():
                logging.warning("Web UI server thread did not exit cleanly.")
        web_ui_manager.stop() # Add a stop method to WebUIManager if needed

    if camera_manager:
        camera_manager.shutdown()

    # if hardware_manager: # Placeholder
    #     hardware_manager.cleanup()

    logging.info("========================================================")
    logging.info(f"--- {Config.VIS_WINDOW_TITLE if hasattr(Config, 'VIS_WINDOW_TITLE') else 'Application'} Stopped ---")
    logging.info("========================================================")
    # Ensure all logs are flushed
    logging.shutdown()
    # A small delay to ensure all threads might see the shutdown event and attempt to close
    time.sleep(0.5)
    # Forcibly exit if anything is stuck, as a last resort for complex thread hangs.
    # os._exit(0) # Use with caution, skips normal Python exit handlers.

if __name__ == "__main__":
    main()