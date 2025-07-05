# main.py

import cv2
import numpy as np
import time
import logging
import threading
import os
import signal 

import Config
from CameraManager import CameraManager
import CvFunctions 
from WebUi import WebUIManager

# --- Global Variables ---
shutdown_event = threading.Event()
camera_manager = None
web_ui_manager = None
web_ui_thread = None 

rvec_global = None
tvec_global = None
extrinsic_calibrated = False 
EXTRINSIC_CALIB_FILE = os.path.join(Config.SCRIPT_DIR, "camera_extrinsic_data.npz")


def signal_handler(sig, frame):
    if shutdown_event.is_set():
        logging.warning(f"Shutdown already in progress (Signal {sig} received again).")
        return
    logging.warning(f"Signal {sig} received. Initiating graceful shutdown...")
    shutdown_event.set()

def setup_logging():
    log_level_str = Config.LOG_LEVEL.upper() if hasattr(Config, 'LOG_LEVEL') else "INFO"
    log_level_attr = getattr(logging, log_level_str, logging.INFO)
    
    log_format_str = Config.LOG_FORMAT if hasattr(Config, 'LOG_FORMAT') else '%(asctime)s - %(levelname)s - [%(threadName)s:%(lineno)d] - %(message)s'
    log_date_fmt_str = Config.LOG_DATE_FORMAT if hasattr(Config, 'LOG_DATE_FORMAT') else '%Y-%m-%d %H:%M:%S'

    logging.basicConfig(level=log_level_attr,
                        format=log_format_str,
                        datefmt=log_date_fmt_str,
                        handlers=[logging.StreamHandler()]) 
    
    logging.getLogger("werkzeug").setLevel(logging.WARNING) 
    logging.getLogger("picamera2").setLevel(logging.WARNING) 

    app_title = "3D Volleyball Tracker"
    logging.info(f"--- Starting: {app_title} ---")
    logging.info(f"Log Level set to: {log_level_str}")


def save_extrinsic_calibration(rvec, tvec):
    try:
        np.savez(EXTRINSIC_CALIB_FILE, rvec=rvec, tvec=tvec)
        logging.info(f"Successfully saved extrinsic calibration data to {EXTRINSIC_CALIB_FILE}")
    except Exception as e:
        logging.error(f"Error saving extrinsic calibration data: {e}", exc_info=True)


clicked_image_points = []
# MODIFIED: Increased to 8 for the cube's corners
MAX_CLICKS = 8

def mouse_callback_extrinsic_calib(event, x, y, flags, param):
    global clicked_image_points
    frame_display = param['frame_display']
    window_name = param.get('window_name', "Extrinsic Calibration")

    if event == cv2.EVENT_LBUTTONDOWN:
        if len(clicked_image_points) < MAX_CLICKS:
            clicked_image_points.append((x, y))
            cv2.circle(frame_display, (x,y), 7, (0,255,0), -1)
            cv2.circle(frame_display, (x,y), 7, (0,0,0), 1)
            cv2.putText(frame_display, f"P{len(clicked_image_points)}", (x+10, y-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (50,200,50), 2)
            cv2.imshow(window_name, frame_display)
            logging.info(f"Clicked point {len(clicked_image_points)}: ({x}, {y})")
            if len(clicked_image_points) == MAX_CLICKS:
                logging.info(f"All {MAX_CLICKS} points collected. Press 'c' to calibrate or 'r' to restart points selection.")
        else:
            logging.info(f"Already collected {MAX_CLICKS} points. Press 'c' to calibrate, 'r' to restart, or 's' to skip.")


def perform_extrinsic_calibration(cam_manager_obj):
    global clicked_image_points, rvec_global, tvec_global, extrinsic_calibrated

    if not cam_manager_obj or not cam_manager_obj.is_initialized:
        logging.error("Extrinsic Calib: Camera manager not valid or not initialized.")
        extrinsic_calibrated = False
        return None, None
    if cam_manager_obj.mtx is None or cam_manager_obj.dist is None:
        logging.error("Extrinsic Calib: Camera intrinsic parameters (mtx, dist) not loaded in camera manager.")
        extrinsic_calibrated = False
        return None, None

    # MODIFIED: Updated instructions for 3D cube calibration
    logging.info("--- Starting Extrinsic Camera Calibration (3D Pose Estimation) ---")
    logging.info(f"Click on the {MAX_CLICKS} corners of the 1x1x1m reference cube in the window.")
    logging.info("Order: Click the 4 BOTTOM corners first, then the 4 TOP corners.")
    logging.info("Bottom Face: 1.Origin(0,0,0), 2.(1,0,0), 3.(1,1,0), 4.(0,1,0)")
    logging.info("Top Face:    5.Origin(0,0,1), 6.(1,0,1), 7.(1,1,1), 8.(0,1,1)")
    logging.info("Press 'c' to CALIBRATE. 'r' to RESTART points. 's' to SKIP. 'q' to QUIT application.")

    window_name = "Extrinsic Calibration - Click Cube Corners"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    
    default_w = Config.CAM_REQUESTED_WIDTH // 2 if hasattr(Config, 'CAM_REQUESTED_WIDTH') and Config.CAM_REQUESTED_WIDTH > 0 else 960
    default_h = Config.CAM_REQUESTED_HEIGHT // 2 if hasattr(Config, 'CAM_REQUESTED_HEIGHT') and Config.CAM_REQUESTED_HEIGHT > 0 else 540
    cv2.resizeWindow(window_name, default_w, default_h)


    frame_for_calibration = None
    for attempt in range(5):
        raw_frame = cam_manager_obj.capture_frame(undistort=False)
        if raw_frame is not None:
            frame_for_calibration = CvFunctions.undistort_frame(raw_frame, cam_manager_obj.mtx,
                                                                 cam_manager_obj.dist, cam_manager_obj.new_camera_mtx)
            if frame_for_calibration is not None:
                logging.info("Extrinsic Calib: Captured and undistorted frame for point selection.")
                break
        time.sleep(0.2)

    if frame_for_calibration is None:
        logging.error("Extrinsic Calib: Failed to get a usable camera frame. Aborting calibration.")
        if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) >= 1: cv2.destroyWindow(window_name)
        extrinsic_calibrated = False
        return None, None

    frame_display = frame_for_calibration.copy()
    callback_params = {'frame_display': frame_display, 'window_name': window_name} 
    cv2.setMouseCallback(window_name, mouse_callback_extrinsic_calib, callback_params)
    clicked_image_points = [] 
    
    extrinsic_calibrated = False 
    temp_rvec, temp_tvec = None, None

    while not shutdown_event.is_set():
        cv2.imshow(window_name, frame_display)
        key = cv2.waitKey(100) & 0xFF

        if key == ord('q'): 
            shutdown_event.set()
            break 
        if key == ord('s'): 
            logging.info("Extrinsic calibration skipped by user.")
            break
        if key == ord('r'): 
            clicked_image_points = []
            frame_display = frame_for_calibration.copy() 
            callback_params['frame_display'] = frame_display
            logging.info("Points reset. Re-click the 8 corners.")
            cv2.setMouseCallback(window_name, mouse_callback_extrinsic_calib, callback_params)

        if len(clicked_image_points) == MAX_CLICKS and key == ord('c'):
            image_points_np = np.array(clicked_image_points, dtype=np.float32)
            # Use all defined corners now
            object_points_np = Config.WORLD_BOX_CORNERS_M[:MAX_CLICKS]

            logging.debug(f"Image Points (2D pixels) for solvePnP:\n{image_points_np}")
            logging.debug(f"Object Points (3D world meters) for solvePnP:\n{object_points_np}")

            try:
                dist_coeffs_for_solvepnp = cam_manager_obj.dist if cam_manager_obj.dist is not None else np.zeros((1,5), dtype=np.float32)
                # Using a more robust solver like SOLVEPNP_IPPE_SQUARE might be good for planar objects,
                # but for a 3D cube, the default or SQPNP are fine.
                retval, rvec, tvec = cv2.solvePnP(
                    object_points_np, image_points_np,
                    cam_manager_obj.mtx, dist_coeffs_for_solvepnp,
                    flags=cv2.SOLVEPNP_SQPNP 
                )

                if retval:
                    projected_points, _ = cv2.projectPoints(object_points_np, rvec, tvec, cam_manager_obj.mtx, dist_coeffs_for_solvepnp)
                    error = cv2.norm(image_points_np, projected_points.reshape(-1,2), cv2.NORM_L2) / len(projected_points)
                    logging.info(f"solvePnP successful. Reprojection error: {error:.4f} pixels.")
                    
                    frame_with_axes = CvFunctions.draw_axes_on_frame(frame_for_calibration.copy(), rvec, tvec, cam_manager_obj.mtx, cam_manager_obj.dist)
                    cv2.putText(frame_with_axes, "SolvePnP OK. Press 'y' to accept, 'n' to retry.", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,200,0), 2)
                    cv2.imshow(window_name, frame_with_axes)

                    confirmation_pending = True
                    while confirmation_pending and not shutdown_event.is_set():
                        confirm_key = cv2.waitKey(100) & 0xFF
                        if confirm_key == ord('y'):
                            temp_rvec, temp_tvec = rvec, tvec 
                            save_extrinsic_calibration(rvec, tvec) 
                            logging.info("Extrinsic calibration accepted and saved.")
                            confirmation_pending = False
                            break 
                        elif confirm_key == ord('n'):
                            clicked_image_points = [] 
                            frame_display = frame_for_calibration.copy()
                            callback_params['frame_display'] = frame_display
                            cv2.setMouseCallback(window_name, mouse_callback_extrinsic_calib, callback_params)
                            confirmation_pending = False 
                    
                    if temp_rvec is not None or shutdown_event.is_set(): 
                         break 
                else:
                    logging.error("cv2.solvePnP failed (returned False).")
            except Exception as e:
                logging.error(f"Exception during solvePnP: {e}", exc_info=True)
        
        if temp_rvec is not None: 
            break 

    if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) >= 1:
        cv2.destroyWindow(window_name)
    
    if temp_rvec is not None and temp_tvec is not None and not shutdown_event.is_set():
        rvec_global, tvec_global = temp_rvec, temp_tvec
        extrinsic_calibrated = True 
        return rvec_global, tvec_global
    
    extrinsic_calibrated = False 
    return None, None


def main_loop():
    global camera_manager, web_ui_manager, rvec_global, tvec_global, extrinsic_calibrated

    if not extrinsic_calibrated: 
        logging.critical("Extrinsic parameters NOT available. Cannot run main_loop.")
        shutdown_event.set() 
        return

    display_window_name = "2D Volleyball Tracking Overlay"
    if Config.DISPLAY_2D_FEED:
        cv2.namedWindow(display_window_name, cv2.WINDOW_AUTOSIZE)

    while not shutdown_event.is_set():
        raw_frame = camera_manager.capture_frame(undistort=False)
        if raw_frame is None:
            time.sleep(0.1)
            continue
        
        undistorted_frame = CvFunctions.undistort_frame(raw_frame, camera_manager.mtx,
                                                         camera_manager.dist, camera_manager.new_camera_mtx)
        if undistorted_frame is None:
            undistorted_frame = raw_frame.copy()

        u, v, radius_px, dbg_mask = CvFunctions.detect_volleyball_2d(undistorted_frame,
                                                                      roi=Config.BALL_DETECTION_ROI)
        
        # --- MODIFIED: 3D Position Calculation ---
        # This is the main logic change. We now call the new function.
        world_coords_3d = None
        if u is not None and v is not None and radius_px is not None:
            world_coords_3d = CvFunctions.estimate_3d_position_from_radius(u, v, radius_px,
                                                                  camera_manager.mtx, camera_manager.dist, 
                                                                  rvec_global, tvec_global)
        
        # The result from the new function is the final position for visualization.
        ball_pos_for_3d_vis = world_coords_3d.tolist() if world_coords_3d is not None else None

        # --- Prepare the display_frame with all overlays ---
        display_frame = undistorted_frame.copy() 

        if u is not None: 
            display_frame = CvFunctions.draw_ball_on_frame(display_frame, u, v, radius_px if radius_px else 10)
        
        # MODIFIED: Call the updated drawing function
        if world_coords_3d is not None:
            display_frame = CvFunctions.draw_3d_info_on_frame(display_frame, world_coords_3d)
        
        display_frame = CvFunctions.draw_axes_on_frame(display_frame, rvec_global, tvec_global,
                                                        camera_manager.mtx, camera_manager.dist)
        
        if web_ui_manager and Config.WEB_UI_ENABLED:
            cam_props = camera_manager.get_camera_properties()
            measured_fps_cam = cam_props.get('measured_fps', 0.0)
            resolution_text = f"{cam_props.get('frame_width',0)}x{cam_props.get('frame_height',0)}"
            web_ui_manager.update_data(
                frame_for_stream=display_frame, 
                ball_2d_coords=(u,v) if u is not None else None,
                ball_pixel_radius=radius_px,
                # Pass the new true 3D coordinates
                ball_3d_world_coords=ball_pos_for_3d_vis,
                camera_intrinsics=camera_manager.mtx.tolist() if camera_manager.mtx is not None else "N/A",
                camera_extrinsics={'rvec': rvec_global.tolist() if rvec_global is not None else "N/A",
                                   'tvec': tvec_global.tolist() if tvec_global is not None else "N/A"},
                fps=measured_fps_cam if measured_fps_cam is not None else 0.0,
                resolution=resolution_text,
                status_message="Tracking..." if u is not None else "Searching for ball..."
            )

        if Config.DISPLAY_2D_FEED:
            cv2.imshow(display_window_name, display_frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                shutdown_event.set()

    if Config.DISPLAY_2D_FEED:
        if cv2.getWindowProperty(display_window_name, cv2.WND_PROP_VISIBLE) >=1 :
             cv2.destroyWindow(display_window_name)
        cv2.destroyAllWindows() 


def main():
    global camera_manager, web_ui_manager, web_ui_thread, extrinsic_calibrated 

    setup_logging()
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    camera_manager = CameraManager(camera_index=Config.CAMERA_INDEX,
                                   width=Config.CAM_REQUESTED_WIDTH,
                                   height=Config.CAM_REQUESTED_HEIGHT,
                                   fps=Config.CAM_REQUESTED_FPS)

    if not camera_manager.load_calibration_data(Config.CALIBRATION_DATA_FILE):
        logging.error("Failed to load camera intrinsic calibration data. Cannot proceed.")
        if camera_manager: camera_manager.shutdown()
        return 

    if not camera_manager.initialize_camera():
        logging.error("Failed to initialize camera. Cannot proceed.")
        if camera_manager: camera_manager.shutdown()
        return

    perform_extrinsic_calibration(camera_manager) 

    if shutdown_event.is_set(): 
        logging.info("Shutdown requested during initial extrinsic calibration.")
    elif not extrinsic_calibrated:
        logging.critical("Extrinsic calibration FAILED or was SKIPPED at startup. Application cannot proceed.")
        shutdown_event.set()
    
    if Config.WEB_UI_ENABLED and not shutdown_event.is_set():
        web_ui_manager = WebUIManager(port=Config.WEB_PORT, shutdown_event_ref=shutdown_event)
        web_ui_thread = threading.Thread(target=web_ui_manager.run, name="WebUIServerThread", daemon=True)
        web_ui_thread.start()
        time.sleep(1.5) 
    
    if not shutdown_event.is_set():
        try:
            main_loop()
        except Exception as e:
            logging.error(f"Unhandled exception in main_loop: {e}", exc_info=True)
            shutdown_event.set()

    logging.info("--- Main: Initiating Final Cleanup Sequence ---")
    if not shutdown_event.is_set(): 
        shutdown_event.set()

    if web_ui_manager and Config.WEB_UI_ENABLED:
        if hasattr(web_ui_manager, 'stop') and callable(web_ui_manager.stop):
             web_ui_manager.stop() 
        if web_ui_thread and web_ui_thread.is_alive():
            web_ui_thread.join(timeout=3.0) 
    
    if camera_manager:
        camera_manager.shutdown()

    logging.info("--- Application Stopped ---")
    logging.shutdown() 

if __name__ == "__main__":
    main()
