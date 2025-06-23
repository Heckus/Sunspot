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

    app_title = Config.VIS_WINDOW_TITLE if hasattr(Config, 'VIS_WINDOW_TITLE') else "3D Volleyball Tracker"
    logging.info(f"--- Starting: {app_title} ---")
    logging.info(f"Log Level set to: {log_level_str}")


def save_extrinsic_calibration(rvec, tvec):
    try:
        np.savez(EXTRINSIC_CALIB_FILE, rvec=rvec, tvec=tvec)
        logging.info(f"Successfully saved extrinsic calibration data to {EXTRINSIC_CALIB_FILE}")
    except Exception as e:
        logging.error(f"Error saving extrinsic calibration data: {e}", exc_info=True)


clicked_image_points = []
MAX_CLICKS = 4

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
                logging.info("All 4 points collected. Press 'c' to calibrate or 'r' to restart points selection.")
        else:
            logging.info("Already collected 4 points. Press 'c' to calibrate, 'r' to restart, or 's' to skip.")


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

    logging.info("--- Starting Extrinsic Camera Calibration (Pose Estimation) ---")
    logging.info("Click on the 4 corners of the box in the 'Extrinsic Calibration' window.")
    logging.info("Order: 1. Origin (Bottom-Front-Left), 2. X-axis (Bottom-Front-Right), 3. X-Y (Bottom-Back-Right), 4. Y-axis (Bottom-Back-Left)")
    logging.info("Press 'c' to CALIBRATE. 'r' to RESTART points. 's' to SKIP. 'q' to QUIT application.")

    window_name = "Extrinsic Calibration - Click Box Corners"
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
            else:
                logging.warning(f"Extrinsic Calib: Undistortion failed (Attempt {attempt+1}). Retrying capture.")
        else:
            logging.warning(f"Extrinsic Calib: Failed to capture raw frame (Attempt {attempt+1}). Retrying.")
        if shutdown_event.is_set(): return None,None 
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
            logging.info("Quit requested during extrinsic calibration. Shutting down application.")
            shutdown_event.set()
            break 
        if key == ord('s'): 
            logging.info("Extrinsic calibration skipped by user.")
            break
        if key == ord('r'): 
            clicked_image_points = []
            frame_display = frame_for_calibration.copy() 
            callback_params['frame_display'] = frame_display
            logging.info("Points reset. Re-click the 4 corners.")
            cv2.setMouseCallback(window_name, mouse_callback_extrinsic_calib, callback_params)

        if len(clicked_image_points) == MAX_CLICKS and key == ord('c'):
            image_points_np = np.array(clicked_image_points, dtype=np.float32)
            object_points_np = Config.WORLD_BOX_CORNERS_M[:MAX_CLICKS]

            logging.debug(f"Image Points (2D pixels) for solvePnP:\n{image_points_np}")
            logging.debug(f"Object Points (3D world meters) for solvePnP:\n{object_points_np}")

            try:
                dist_coeffs_for_solvepnp = cam_manager_obj.dist if cam_manager_obj.dist is not None else np.zeros((1,5), dtype=np.float32)
                retval, rvec, tvec = cv2.solvePnP(
                    object_points_np, image_points_np,
                    cam_manager_obj.mtx, dist_coeffs_for_solvepnp,
                    flags=cv2.SOLVEPNP_SQPNP 
                )

                if retval:
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
                            logging.info("Extrinsic calibration rejected. Restarting point selection.")
                            clicked_image_points = [] 
                            frame_display = frame_for_calibration.copy()
                            callback_params['frame_display'] = frame_display
                            cv2.setMouseCallback(window_name, mouse_callback_extrinsic_calib, callback_params)
                            confirmation_pending = False 
                        elif confirm_key == ord('q'):
                            logging.info("Quit requested during extrinsic calibration confirmation.")
                            shutdown_event.set()
                            confirmation_pending = False 
                    
                    if temp_rvec is not None or shutdown_event.is_set(): 
                         break 
                else:
                    logging.error("cv2.solvePnP failed (returned False).")
                    cv2.putText(frame_display, "SolvePnP FAILED. Press 'r' to retry.", (10,30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
            except cv2.error as e:
                logging.error(f"OpenCV Exception during solvePnP: {e}", exc_info=True)
                cv2.putText(frame_display, f"SolvePnP OpenCV Error! Press 'r'.", (10,30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)
            except Exception as e:
                logging.error(f"Generic Exception during solvePnP: {e}", exc_info=True)
                cv2.putText(frame_display, f"SolvePnP Generic Error! Press 'r'.", (10,30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)
        
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

    frame_count = 0
    loop_start_time_total = time.monotonic()
    last_log_time = time.monotonic()

    # This will be the frame passed to the web UI for streaming
    annotated_frame_for_web_stream = None 

    while not shutdown_event.is_set():
        loop_iter_start_time = time.monotonic()

        raw_frame = camera_manager.capture_frame(undistort=False)
        if raw_frame is None:
            logging.warning("MainLoop: Failed to get frame.")
            if not camera_manager.is_initialized:
                logging.error("MainLoop: CameraManager uninitialized. Shutting down.")
                shutdown_event.set()
                continue
            time.sleep(0.1)
            continue
        
        undistorted_frame = CvFunctions.undistort_frame(raw_frame, camera_manager.mtx,
                                                         camera_manager.dist, camera_manager.new_camera_mtx)
        if undistorted_frame is None:
            logging.warning("MainLoop: Undistortion failed. Using raw_frame.")
            undistorted_frame = raw_frame.copy()

        u, v, radius_px, dbg_mask = CvFunctions.detect_volleyball_2d(undistorted_frame,
                                                                      roi=Config.BALL_DETECTION_ROI)
        current_ball_2d = (u, v, radius_px)

        world_coords_3d_contact = None
        if u is not None and v is not None:
            world_coords_3d_contact = CvFunctions.map_2d_to_3d_on_ground(u, v,
                                                                  camera_manager.mtx, camera_manager.dist, 
                                                                  rvec_global, tvec_global)
        
        ball_pos_for_3d_vis = None
        if world_coords_3d_contact is not None:
            ball_pos_for_3d_vis = [world_coords_3d_contact[0], 
                                   world_coords_3d_contact[1], 
                                   world_coords_3d_contact[2] + Config.VOLLEYBALL_RADIUS_M]

        # --- Prepare the display_frame with all overlays ---
        # This frame will be used for BOTH cv2.imshow() AND the web stream
        display_frame_for_imshow_and_web = undistorted_frame.copy() 

        if hasattr(Config, 'LOG_LEVEL') and Config.LOG_LEVEL.upper() == "DEBUG" and dbg_mask is not None:
            if dbg_mask.shape[:2] == display_frame_for_imshow_and_web.shape[:2]:
                current_dbg_mask = dbg_mask
                if len(dbg_mask.shape) == 2 or dbg_mask.shape[2] == 1:
                    current_dbg_mask = cv2.cvtColor(dbg_mask, cv2.COLOR_GRAY2BGR)
                elif dbg_mask.shape[2] == 4: 
                    current_dbg_mask = cv2.cvtColor(dbg_mask, cv2.COLOR_BGRA2BGR)
                display_frame_for_imshow_and_web = cv2.addWeighted(display_frame_for_imshow_and_web, 0.7, current_dbg_mask, 0.3, 0)

        if u is not None: 
            display_frame_for_imshow_and_web = CvFunctions.draw_ball_on_frame(display_frame_for_imshow_and_web, u, v, radius_px if radius_px else 10)
        if world_coords_3d_contact is not None:
            display_frame_for_imshow_and_web = CvFunctions.draw_3d_info_on_frame(display_frame_for_imshow_and_web, world_coords_3d_contact, Config.VOLLEYBALL_RADIUS_M)
        
        display_frame_for_imshow_and_web = CvFunctions.draw_axes_on_frame(display_frame_for_imshow_and_web, rvec_global, tvec_global,
                                                        camera_manager.mtx, camera_manager.dist)
        
        current_loop_duration = time.monotonic() - loop_iter_start_time
        current_proc_fps = 1.0 / current_loop_duration if current_loop_duration > 0 else 0
        cv2.putText(display_frame_for_imshow_and_web, f"Proc FPS: {current_proc_fps:.1f}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        
        # Assign to the variable that will be passed to the web UI
        annotated_frame_for_web_stream = display_frame_for_imshow_and_web 
        # --- End of display_frame preparation ---

        if web_ui_manager and Config.WEB_UI_ENABLED:
            cam_props = camera_manager.get_camera_properties()
            measured_fps_cam = cam_props.get('measured_fps', 0.0)
            resolution_text = f"{cam_props.get('frame_width',0)}x{cam_props.get('frame_height',0)}"
            web_ui_manager.update_data(
                # Pass the ANNOTATED frame for the stream
                frame_for_stream=annotated_frame_for_web_stream, 
                ball_2d_coords=current_ball_2d[:2] if u is not None else None,
                ball_pixel_radius=radius_px,
                ball_3d_world_coords=ball_pos_for_3d_vis,
                camera_intrinsics=camera_manager.mtx.tolist() if camera_manager.mtx is not None else "N/A",
                camera_extrinsics={'rvec': rvec_global.tolist() if rvec_global is not None else "N/A",
                                   'tvec': tvec_global.tolist() if tvec_global is not None else "N/A"},
                fps=measured_fps_cam if measured_fps_cam is not None else 0.0,
                resolution=resolution_text,
                status_message="Tracking..." if u is not None else "Searching for ball..."
            )

        if Config.DISPLAY_2D_FEED:
            cv2.imshow(display_window_name, annotated_frame_for_web_stream) # Show the same annotated frame
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                logging.info("User pressed 'q'. Shutting down.")
                shutdown_event.set()
            elif key == ord('e'):
                logging.info("User requested extrinsic calibration re-run.")
                if cv2.getWindowProperty(display_window_name, cv2.WND_PROP_VISIBLE) >= 1:
                    cv2.destroyWindow(display_window_name) 
                
                perform_extrinsic_calibration(camera_manager) 
                
                if not extrinsic_calibrated: 
                    logging.error("Re-calibration failed/skipped. 3D mapping may be compromised.")
                else:
                    logging.info("Extrinsic calibration re-run successful.")

                if Config.DISPLAY_2D_FEED and not shutdown_event.is_set():
                    cv2.namedWindow(display_window_name, cv2.WINDOW_AUTOSIZE)


        frame_count += 1
        loop_duration_actual = time.monotonic() - loop_iter_start_time
        if hasattr(Config, 'MAIN_LOOP_MAX_FPS') and Config.MAIN_LOOP_MAX_FPS > 0:
            sleep_time = (1.0 / Config.MAIN_LOOP_MAX_FPS) - loop_duration_actual
            if sleep_time > 0:
                time.sleep(sleep_time)
        
        if time.monotonic() - last_log_time > 10.0: 
            current_total_duration = time.monotonic() - loop_start_time_total
            avg_loop_fps_so_far = frame_count / current_total_duration if current_total_duration > 0 else 0
            logging.debug(f"Avg Main Loop FPS: {avg_loop_fps_so_far:.2f} ({frame_count} frames)")
            last_log_time = time.monotonic()

    total_duration_final = time.monotonic() - loop_start_time_total
    avg_loop_fps_final = frame_count / total_duration_final if total_duration_final > 0 else 0
    logging.info(f"Main loop ended. Processed {frame_count} frames in {total_duration_final:.2f}s. Overall Avg FPS: {avg_loop_fps_final:.2f}")
    
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
        if not os.path.exists(Config.CALIBRATION_DATA_FILE):
            logging.error(f"Ensure '{Config.CALIBRATION_DATA_FILE}' exists or run camera_calibration.py.")
        if camera_manager: camera_manager.shutdown()
        return 

    if not camera_manager.initialize_camera():
        logging.error("Failed to initialize camera. Cannot proceed.")
        if camera_manager: camera_manager.shutdown()
        return

    logging.info("Startup: Forcing extrinsic calibration.")
    perform_extrinsic_calibration(camera_manager) 

    if shutdown_event.is_set(): 
        logging.info("Shutdown requested during initial extrinsic calibration.")
    elif not extrinsic_calibrated:
        logging.critical("Extrinsic calibration FAILED or was SKIPPED at startup. Application cannot proceed with 3D mapping.")
        shutdown_event.set()
    else:
        logging.info("Initial extrinsic calibration successful.")
    
    if Config.WEB_UI_ENABLED and not shutdown_event.is_set():
        logging.info("WebUI is enabled. Initializing WebUIManager.")
        web_ui_manager = WebUIManager(
            host='0.0.0.0',
            port=Config.WEB_PORT,
            shutdown_event_ref=shutdown_event
        )
        web_ui_thread = threading.Thread(target=web_ui_manager.run, name="WebUIServerThread", daemon=True)
        web_ui_thread.start()
        time.sleep(1.5) 
        if not web_ui_thread.is_alive():
            logging.error("Web UI server thread failed to start. Check for port conflicts.")
    elif not Config.WEB_UI_ENABLED:
        logging.info("Web UI is disabled in Config.")


    if not shutdown_event.is_set():
        try:
            main_loop()
        except KeyboardInterrupt: 
            logging.info("KeyboardInterrupt in main_loop. Initiating shutdown.")
            shutdown_event.set()
        except Exception as e:
            logging.error(f"Unhandled exception in main_loop: {e}", exc_info=True)
            shutdown_event.set()

    logging.info("--- Main: Initiating Final Cleanup Sequence ---")
    if not shutdown_event.is_set(): 
        logging.info("Setting shutdown_event during final cleanup.")
        shutdown_event.set()

    if web_ui_manager and Config.WEB_UI_ENABLED:
        logging.info("Main: Signaling WebUI Manager to stop...")
        if hasattr(web_ui_manager, 'stop') and callable(web_ui_manager.stop):
             web_ui_manager.stop() 

        if web_ui_thread and web_ui_thread.is_alive():
            logging.info("Main: Waiting for WebUI server thread to join...")
            web_ui_thread.join(timeout=3.0) 
            if web_ui_thread.is_alive():
                logging.warning("Main: WebUI server thread did not exit cleanly after join timeout.")
    
    if camera_manager:
        logging.info("Main: Shutting down Camera Manager...")
        camera_manager.shutdown()

    app_title_final = Config.VIS_WINDOW_TITLE if hasattr(Config, 'VIS_WINDOW_TITLE') else "Application"
    logging.info(f"--- {app_title_final} Stopped ---")
    logging.info("========================================================")
    
    logging.shutdown() 

if __name__ == "__main__":
    main()