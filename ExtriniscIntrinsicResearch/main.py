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
    if shutdown_event.is_set(): return
    logging.warning(f"Signal {sig} received. Initiating graceful shutdown...")
    shutdown_event.set()

def setup_logging():
    log_level_str = Config.LOG_LEVEL.upper()
    log_level_attr = getattr(logging, log_level_str, logging.INFO)
    logging.basicConfig(level=log_level_attr, format=Config.LOG_FORMAT, datefmt=Config.LOG_DATE_FORMAT, handlers=[logging.StreamHandler()]) 
    logging.getLogger("werkzeug").setLevel(logging.WARNING) 
    logging.getLogger("picamera2").setLevel(logging.WARNING)
    logging.getLogger("ultralytics").setLevel(logging.WARNING) # Quieten YOLO logs
    logging.info("--- Starting: 3D Volleyball Tracker (YOLO Version) ---")

def save_extrinsic_calibration(rvec, tvec):
    try:
        np.savez(EXTRINSIC_CALIB_FILE, rvec=rvec, tvec=tvec)
        logging.info(f"Successfully saved extrinsic calibration data to {EXTRINSIC_CALIB_FILE}")
    except Exception as e:
        logging.error(f"Error saving extrinsic calibration data: {e}", exc_info=True)


clicked_image_points = []
# MODIFIED: Changed to 6 for the updated calibration process
MAX_CLICKS = 6

def mouse_callback_extrinsic_calib(event, x, y, flags, param):
    global clicked_image_points
    frame_display = param['frame_display']
    if event == cv2.EVENT_LBUTTONDOWN and len(clicked_image_points) < MAX_CLICKS:
        clicked_image_points.append((x, y))
        cv2.circle(frame_display, (x,y), 7, (0,255,0), -1)
        cv2.putText(frame_display, f"P{len(clicked_image_points)}", (x+10, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (50,200,50), 2)
        cv2.imshow(param['window_name'], frame_display)
        logging.info(f"Clicked point {len(clicked_image_points)}: ({x}, {y})")
        if len(clicked_image_points) == MAX_CLICKS:
            logging.info(f"All {MAX_CLICKS} points collected. Press 'c' to calibrate.")

def perform_extrinsic_calibration(cam_manager_obj):
    global clicked_image_points, rvec_global, tvec_global, extrinsic_calibrated
    if not cam_manager_obj or not cam_manager_obj.is_initialized or cam_manager_obj.mtx is None:
        logging.error("Extrinsic Calib: Camera not ready or intrinsics not loaded.")
        return None, None

    # MODIFIED: Updated instructions for 6-point calibration
    logging.info("--- Starting 6-Point Extrinsic Camera Calibration ---")
    logging.info(f"Click on the {MAX_CLICKS} specified corners of the reference cube.")
    logging.info("Order: Click the 4 BOTTOM corners, then 2 specified TOP corners.")
    logging.info("1. Bottom-Front-Left  (0,0,0)")
    logging.info("2. Bottom-Front-Right (1,0,0)")
    logging.info("3. Bottom-Back-Right  (1,1,0)")
    logging.info("4. Bottom-Back-Left   (0,1,0)")
    logging.info("5. Top-Back-Left      (0,1,1)")
    logging.info("6. Top-Back-Right     (1,1,1)")
    logging.info("Press 'c' to CALIBRATE. 'r' to RESTART. 's' to SKIP. 'q' to QUIT.")

    window_name = "Extrinsic Calibration - 6 Points"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    
    frame_for_calibration = None
    for _ in range(5):
        raw_frame = cam_manager_obj.capture_frame(undistort=False)
        if raw_frame is not None:
            frame_for_calibration = CvFunctions.undistort_frame(raw_frame, cam_manager_obj.mtx, cam_manager_obj.dist, cam_manager_obj.new_camera_mtx)
            if frame_for_calibration is not None: break
    if frame_for_calibration is None:
        logging.error("Extrinsic Calib: Failed to get a usable camera frame.")
        return None, None

    frame_display = frame_for_calibration.copy()
    cv2.setMouseCallback(window_name, mouse_callback_extrinsic_calib, {'frame_display': frame_display, 'window_name': window_name})
    clicked_image_points = [] 
    
    temp_rvec, temp_tvec = None, None
    while not shutdown_event.is_set() and temp_rvec is None:
        cv2.imshow(window_name, frame_display)
        key = cv2.waitKey(100) & 0xFF

        if key == ord('q'): shutdown_event.set(); break
        if key == ord('s'): logging.info("Extrinsic calibration skipped."); break
        if key == ord('r'):
            clicked_image_points = []
            frame_display = frame_for_calibration.copy()
            logging.info("Points reset. Re-click the 6 corners.")

        if len(clicked_image_points) == MAX_CLICKS and key == ord('c'):
            image_points_np = np.array(clicked_image_points, dtype=np.float32)
            
            # MODIFIED: Define the 6 world points corresponding to the clicking order
            object_points_np = np.array([
                Config.WORLD_BOX_CORNERS_M[0], # (0,0,0)
                Config.WORLD_BOX_CORNERS_M[1], # (1,0,0)
                Config.WORLD_BOX_CORNERS_M[2], # (1,1,0)
                Config.WORLD_BOX_CORNERS_M[3], # (0,1,0)
                Config.WORLD_BOX_CORNERS_M[7], # (0,1,1)
                Config.WORLD_BOX_CORNERS_M[6]  # (1,1,1)
            ], dtype=np.float32)

            try:
                retval, rvec, tvec = cv2.solvePnP(object_points_np, image_points_np, cam_manager_obj.mtx, cam_manager_obj.dist, flags=cv2.SOLVEPNP_SQPNP)
                if retval:
                    projected_points, _ = cv2.projectPoints(object_points_np, rvec, tvec, cam_manager_obj.mtx, cam_manager_obj.dist)
                    error = cv2.norm(image_points_np, projected_points.reshape(-1,2), cv2.NORM_L2) / len(projected_points)
                    logging.info(f"solvePnP successful. Reprojection error: {error:.4f} pixels.")
                    frame_with_axes = CvFunctions.draw_axes_on_frame(frame_for_calibration.copy(), rvec, tvec, cam_manager_obj.mtx, cam_manager_obj.dist)
                    cv2.putText(frame_with_axes, "OK. Press 'y' to accept, 'n' to retry.", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,200,0), 2)
                    cv2.imshow(window_name, frame_with_axes)
                    
                    while not shutdown_event.is_set():
                        confirm_key = cv2.waitKey(100) & 0xFF
                        if confirm_key == ord('y'):
                            temp_rvec, temp_tvec = rvec, tvec
                            save_extrinsic_calibration(rvec, tvec)
                            break
                        elif confirm_key == ord('n'):
                            clicked_image_points = []
                            frame_display = frame_for_calibration.copy()
                            break
            except Exception as e:
                logging.error(f"Exception during solvePnP: {e}", exc_info=True)
    
    cv2.destroyAllWindows()
    if temp_rvec is not None and temp_tvec is not None:
        rvec_global, tvec_global = temp_rvec, temp_tvec
        extrinsic_calibrated = True
    return rvec_global, tvec_global

def main_loop():
    if not extrinsic_calibrated:
        logging.critical("Extrinsic parameters NOT available. Shutting down.")
        shutdown_event.set()
        return

    display_window_name = "3D Volleyball Tracker (YOLO)"
    if Config.DISPLAY_2D_FEED:
        cv2.namedWindow(display_window_name, cv2.WINDOW_AUTOSIZE)

    while not shutdown_event.is_set():
        raw_frame = camera_manager.capture_frame(undistort=False)
        if raw_frame is None: time.sleep(0.1); continue
        
        undistorted_frame = CvFunctions.undistort_frame(raw_frame, camera_manager.mtx, camera_manager.dist, camera_manager.new_camera_mtx)
        
        # --- MODIFIED: Use YOLO for detection ---
        u, v, radius_px, debug_frame = CvFunctions.detect_volleyball_yolo(undistorted_frame)
        
        world_coords_3d = None
        if u is not None:
            world_coords_3d = CvFunctions.estimate_3d_position_from_radius(u, v, radius_px, camera_manager.mtx, camera_manager.dist, rvec_global, tvec_global)
        
        # --- Prepare display frame with overlays ---
        # The debug_frame from YOLO already has the bounding box
        display_frame = debug_frame.copy()
        if world_coords_3d is not None:
            display_frame = CvFunctions.draw_3d_info_on_frame(display_frame, world_coords_3d)
        display_frame = CvFunctions.draw_axes_on_frame(display_frame, rvec_global, tvec_global, camera_manager.mtx, camera_manager.dist)
        
        if web_ui_manager and Config.WEB_UI_ENABLED:
            web_ui_manager.update_data(
                frame_for_stream=display_frame,
                ball_2d_coords=(u,v) if u is not None else None,
                ball_pixel_radius=radius_px,
                ball_3d_world_coords=world_coords_3d.tolist() if world_coords_3d is not None else None
            )

        if Config.DISPLAY_2D_FEED:
            cv2.imshow(display_window_name, display_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'): shutdown_event.set()

    cv2.destroyAllWindows()

def main():
    global camera_manager, web_ui_manager, web_ui_thread
    setup_logging()
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Initialize YOLO model
    CvFunctions.initialize_yolo_model()

    camera_manager = CameraManager()
    if not camera_manager.load_calibration_data(Config.CALIBRATION_DATA_FILE) or not camera_manager.initialize_camera():
        logging.error("Failed to initialize camera or load intrinsics. Cannot proceed.")
        return

    perform_extrinsic_calibration(camera_manager) 

    if extrinsic_calibrated and Config.WEB_UI_ENABLED:
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

    logging.info("--- Main: Initiating Final Cleanup ---")
    if camera_manager: camera_manager.shutdown()
    if web_ui_thread and web_ui_thread.is_alive(): web_ui_thread.join(timeout=2.0)
    logging.info("--- Application Stopped ---")

if __name__ == "__main__":
    main()
