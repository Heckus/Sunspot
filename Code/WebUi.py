# web_ui.py

import threading
import time
import logging
import queue # For passing data between main thread and Open3D thread
import numpy as np
import cv2 # For MJPEG streaming
from flask import Flask, Response, render_template_string, jsonify

import Config # Assuming Config.py has visualization settings

OPEN3D_AVAILABLE = False
OPEN3D_INITIALIZATION_FAILED = False # New flag

try:
    import open3d as o3d
    OPEN3D_AVAILABLE = True
except ImportError:
    logging.error("Open3D library not found. 3D visualization will be disabled.")
    logging.error("Please install it: pip install open3d")
    OPEN3D_INITIALIZATION_FAILED = True # Mark as failed if import fails
except Exception as e:
    logging.error(f"Open3D import failed with an unexpected error: {e}. 3D visualization will be disabled.")
    OPEN3D_INITIALIZATION_FAILED = True # Mark as failed


class WebUIManager:
    """
    Manages the Flask web server for displaying variables and streaming 2D video,
    and runs the Open3D visualizer in a separate thread for 3D model display.
    """

    def __init__(self, host='0.0.0.0', port=Config.WEB_PORT, shutdown_event_ref=None):
        global OPEN3D_INITIALIZATION_FAILED # Ensure we're referencing the global flag
        self.host = host
        self.port = port
        self.app = Flask(__name__)
        self.shutdown_event = shutdown_event_ref if shutdown_event_ref else threading.Event()

        self.latest_raw_frame = None
        self.ball_3d_coords_vis = None
        self.box_corners_world = Config.WORLD_BOX_CORNERS_M.tolist()
        self.box_dimensions = [Config.BOX_WIDTH_M, Config.BOX_DEPTH_M, Config.BOX_HEIGHT_M]
        self.display_variables = {
            "status": "Initializing...",
            "ball_2d_px": "N/A",
            "ball_3d_m": "N/A",
            "ball_pixel_radius_px": "N/A",
            "fps": 0.0,
            "resolution": "N/A",
            "cam_intrinsics": "N/A",
            "cam_extrinsics": "N/A",
            "o3d_status": "Open3D Not Initialized"
        }
        self.data_lock = threading.Lock()

        self.o3d_vis = None
        self.o3d_box_mesh = None
        self.o3d_volleyball_mesh = None
        self.o3d_world_axes = None
        self.o3d_initialized_successfully = False
        self.o3d_update_queue = queue.Queue(maxsize=5)

        if not OPEN3D_AVAILABLE: # This global flag would be True if import failed
            logging.warning("WebUI: Open3D library not available, 3D visualization will not run.")
            self.display_variables["o3d_status"] = "Open3D Not Available (Import Failed)"
            OPEN3D_INITIALIZATION_FAILED = True # Explicitly ensure it's set
        else:
            # Open3D is available, attempt to start its thread.
            # The thread itself will handle further initialization checks.
            self.display_variables["o3d_status"] = "Open3D Initializing..."
            self.o3d_thread = threading.Thread(target=self._run_o3d_visualizer, name="Open3DThread", daemon=True)
            self.o3d_thread.start()

        self._setup_routes()

    def _setup_routes(self):
        """Defines Flask routes."""
        @self.app.route('/')
        def index():
            html = """
            <!DOCTYPE html>
            <html>
            <head>
                <title>3D Volleyball Tracker</title>
                <style>
                    body { font-family: sans-serif; margin: 20px; background-color: #f4f4f4; color: #333; display: flex; flex-direction: column; align-items: center;}
                    .main-container { display: flex; flex-direction: row; max-width: 1400px; width:100%; background: #fff; padding: 20px; box-shadow: 0 0 10px rgba(0,0,0,0.1); }
                    .stream-data-container { flex: 3; display: flex; flex-direction: column; padding-right: 20px; }
                    .stream-container { flex: 2; /* Takes up 2/3 of stream-data-container */ }
                    .data-tables-container { flex: 1; display: flex; flex-direction: column; padding-top: 10px; /* Takes 1/3 */ }
                    .controls-container { flex: 1; border-left: 1px solid #ddd; padding-left: 20px; min-width: 300px; }
                    h1, h2 { color: #333; margin-top:0; }
                    img#video_stream { display: block; border: 1px solid #ccc; margin-bottom:10px; max-width:100%; background-color: #222; min-height: 360px;}
                    p { margin: 5px 0; }
                    strong { color: #555; }
                    table { width: 100%; border-collapse: collapse; margin-bottom:15px; }
                    th, td { text-align: left; padding: 6px; border-bottom: 1px solid #eee; }
                    th { background-color: #f9f9f9; }
                    pre { font-size:0.8em; white-space:pre-wrap; background-color: #efefef; padding: 5px; border-radius: 3px; border: 1px solid #ddd; margin:0; }
                    .log-container { margin-top: 20px; width: 100%; max-width: 1400px; background: #fff; padding: 15px; box-shadow: 0 0 10px rgba(0,0,0,0.1); }
                    #log_messages { height: 150px; overflow-y: scroll; border: 1px solid #ccc; padding: 10px; background-color: #fdfdfd; font-family: monospace; font-size: 0.9em;}
                </style>
            </head>
            <body>
                <h1>3D Volleyball Tracker Dashboard</h1>
                <div class="main-container">
                    <div class="stream-data-container">
                        <div class="stream-container">
                            <h2>Camera Stream (2D)</h2>
                            <img id="video_stream" src="/video_feed" alt="Loading camera stream...">
                        </div>
                        <div class="data-tables-container">
                            <h2>Live Data</h2>
                            <table>
                                <tr><th>Parameter</th><th>Value</th></tr>
                                <tr><td><strong>Status:</strong></td><td><span id="status">N/A</span></td></tr>
                                <tr><td><strong>FPS:</strong></td><td><span id="fps">N/A</span></td></tr>
                                <tr><td><strong>Resolution:</strong></td><td><span id="resolution">N/A</span></td></tr>
                                <tr><td><strong>Open3D Status:</strong></td><td><span id="o3d_status">N/A</span></td></tr>
                            </table>
                             <h2>Ball Data</h2>
                            <table>
                                <tr><th>Parameter</th><th>Value</th></tr>
                                <tr><td><strong>Ball 2D (px):</strong></td><td><span id="ball_2d_px">N/A</span></td></tr>
                                <tr><td><strong>Ball Radius (px):</strong></td><td><span id="ball_pixel_radius_px">N/A</span></td></tr>
                                <tr><td><strong>Ball 3D (m):</strong></td><td><span id="ball_3d_m">N/A</span></td></tr>
                            </table>
                        </div>
                    </div>
                    <div class="controls-container">
                        <h2>Camera Parameters</h2>
                        <p><strong>Intrinsics:</strong> <pre id="cam_intrinsics">N/A</pre></p>
                        <p><strong>Extrinsics:</strong> <pre id="cam_extrinsics">N/A</pre></p>
                        <p><em>Note: The 3D visualization (if active) runs in a separate Open3D window.</em></p>
                    </div>
                </div>

                <div class="log-container">
                    <h2>System Log (placeholder)</h2>
                    <div id="log_messages">
                        <p>Log messages would appear here...</p>
                    </div>
                </div>

                <script>
                    const streamImg = document.getElementById('video_stream');
                    if (streamImg) {
                        streamImg.onerror = function() {
                            this.alt = 'Camera stream failed to load or is unavailable.';
                            this.src = ''; 
                            console.error('Error loading video stream.');
                        };
                    }

                    function updateDataFields() {
                        fetch('/status_json')
                            .then(response => response.json())
                            .then(data => {
                                document.getElementById('status').textContent = data.status || 'N/A';
                                document.getElementById('fps').textContent = data.fps !== undefined ? parseFloat(data.fps).toFixed(2) : 'N/A';
                                document.getElementById('resolution').textContent = data.resolution || 'N/A';
                                document.getElementById('o3d_status').textContent = data.o3d_status || 'N/A';

                                document.getElementById('ball_2d_px').textContent = data.ball_2d_px || 'N/A';
                                document.getElementById('ball_pixel_radius_px').textContent = data.ball_pixel_radius_px || 'N/A';
                                document.getElementById('ball_3d_m').textContent = data.ball_3d_m || 'N/A';
                                
                                document.getElementById('cam_intrinsics').textContent = data.cam_intrinsics || 'N/A';
                                document.getElementById('cam_extrinsics').textContent = data.cam_extrinsics || 'N/A';
                            })
                            .catch(error => console.error('Error fetching status JSON:', error));
                    }

                    setInterval(updateDataFields, 1000);
                    document.addEventListener('DOMContentLoaded', updateDataFields);
                </script>
            </body>
            </html>
            """
            with self.data_lock:
                return render_template_string(html, variables=self.display_variables.copy())

        @self.app.route('/video_feed')
        def video_feed():
            return Response(self._generate_mjpeg_stream(),
                            mimetype='multipart/x-mixed-replace; boundary=frame')

        @self.app.route('/status_json')
        def status_json():
            with self.data_lock:
                return jsonify(self.display_variables.copy())

    def _generate_mjpeg_stream(self):
        logging.info("MJPEG stream client connected.")
        frame_num = 0
        while not self.shutdown_event.is_set():
            frame_to_encode = None
            with self.data_lock:
                if self.latest_raw_frame is not None:
                    frame_to_encode = self.latest_raw_frame.copy()

            if frame_to_encode is None:
                ph_height = Config.CAM_REQUESTED_HEIGHT // 2 if Config.CAM_REQUESTED_HEIGHT else 240
                ph_width = Config.CAM_REQUESTED_WIDTH // 2 if Config.CAM_REQUESTED_WIDTH else 320
                frame_to_encode = np.zeros((ph_height, ph_width, 3), dtype=np.uint8)
                cv2.putText(frame_to_encode, "No Stream", (30, ph_height // 2), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                time.sleep(0.1)
            else:
                frame_num+=1

            try:
                jpeg_quality = Config.WEB_STREAM_JPEG_QUALITY if hasattr(Config, 'WEB_STREAM_JPEG_QUALITY') else 75
                flag, encoded_image = cv2.imencode('.jpg', frame_to_encode, [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])
                if not flag:
                    logging.warning("MJPEG: Could not encode frame.")
                    time.sleep(0.1)
                    continue
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + bytearray(encoded_image) + b'\r\n')
            except GeneratorExit:
                logging.info("MJPEG stream client disconnected (GeneratorExit).")
                break
            except (ConnectionResetError, BrokenPipeError) as e:
                logging.info(f"MJPEG stream client connection issue ({type(e).__name__}).")
                break
            except Exception as e:
                logging.error(f"Error in MJPEG stream: {e}", exc_info=True)
                break
            
            max_fps = Config.WEB_STREAM_MAX_FPS if hasattr(Config, 'WEB_STREAM_MAX_FPS') else (Config.CAM_REQUESTED_FPS if hasattr(Config, 'CAM_REQUESTED_FPS') else 15)
            sleep_duration = 1.0 / max_fps
            time.sleep(max(0.01, sleep_duration))
        logging.info("MJPEG stream generator stopped.")


    def _init_o3d_scene(self):
        global OPEN3D_INITIALIZATION_FAILED # Refer to the global flag

        if not OPEN3D_AVAILABLE: # Already checked at __init__, but good for safety
            OPEN3D_INITIALIZATION_FAILED = True
            with self.data_lock:
                self.display_variables["o3d_status"] = "Open3D Not Available (Import Failed)"
            return

        # self.o3d_vis is initialized here
        self.o3d_vis = o3d.visualization.Visualizer()
        created_successfully = False
        try:
            created_successfully = self.o3d_vis.create_window(
                                   window_name=Config.VIS_WINDOW_TITLE,
                                   width=Config.VIS_WINDOW_WIDTH,
                                   height=Config.VIS_WINDOW_HEIGHT,
                                   visible=True
                                   )
            if not created_successfully:
                logging.error("Open3D: o3d_vis.create_window() returned False. Window creation failed.")
                # Don't call destroy_window here if create_window itself returned False.
                # The vis object might be in an invalid state.
                self.o3d_vis = None # Nullify the object
                OPEN3D_INITIALIZATION_FAILED = True
                with self.data_lock:
                    self.display_variables["o3d_status"] = "Open3D Window Creation Failed (API ret False)"
                return
        except RuntimeError as e:
            logging.error(f"Open3D: RuntimeError during o3d_vis.create_window(): {e}")
            self.o3d_vis = None # Nullify
            OPEN3D_INITIALIZATION_FAILED = True
            with self.data_lock:
                self.display_variables["o3d_status"] = f"Open3D Window Error (Runtime: {e})"
            return

        # Proceed only if window creation was successful AND self.o3d_vis is not None
        if self.o3d_vis is None: # Should have been caught by above, but defensive check
            OPEN3D_INITIALIZATION_FAILED = True
            with self.data_lock:
                self.display_variables["o3d_status"] = "Open3D Visualizer became None after create_window call"
            return

        opt = self.o3d_vis.get_render_option()
        if opt is None:
            logging.error("Open3D: get_render_option() returned None. Disabling Open3D.")
            # If opt is None, the window might still exist but is unusable for rendering options.
            # It's safer to destroy it if it was created.
            if created_successfully and self.o3d_vis: # Check if vis object is still valid
                self.o3d_vis.destroy_window()
            self.o3d_vis = None # Nullify
            OPEN3D_INITIALIZATION_FAILED = True
            with self.data_lock:
                self.display_variables["o3d_status"] = "Open3D Render Options Failed"
            return

        opt.background_color = np.asarray([0.1, 0.1, 0.1])
        opt.light_on = True
        opt.point_size = 2.0

        box_w, box_d, box_h_vis = self.box_dimensions[0], self.box_dimensions[1], self.box_dimensions[2]
        self.o3d_box_mesh = o3d.geometry.TriangleMesh.create_box(width=box_w, height=box_d, depth=box_h_vis, create_uv_map=True, map_texture_to_each_face=True)
        self.o3d_box_mesh.translate([box_w / 2, box_d / 2, box_h_vis / 2], relative=False)
        self.o3d_box_mesh.paint_uniform_color(Config.VIS_BOX_COLOR)
        self.o3d_vis.add_geometry(self.o3d_box_mesh, reset_bounding_box=True)

        self.o3d_volleyball_mesh = o3d.geometry.TriangleMesh.create_sphere(radius=Config.VOLLEYBALL_RADIUS_M)
        self.o3d_volleyball_mesh.paint_uniform_color(Config.VIS_VOLLEYBALL_COLOR)
        initial_ball_pos = [box_w / 2, box_d / 2, Config.VOLLEYBALL_RADIUS_M + box_h_vis]
        self.o3d_volleyball_mesh.translate(initial_ball_pos, relative=False)
        self.o3d_vis.add_geometry(self.o3d_volleyball_mesh, reset_bounding_box=False)

        if Config.VIS_AXES_ENABLED:
            self.o3d_world_axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0,0,0])
            self.o3d_vis.add_geometry(self.o3d_world_axes, reset_bounding_box=False)

        view_control = self.o3d_vis.get_view_control()
        if view_control:
            look_at_target = np.array([box_w / 2, box_d / 2, 0.0])
            view_control.set_lookat(look_at_target)
            view_control.set_front(np.array(Config.VIS_CAMERA_FRONT))
            view_control.set_up(np.array(Config.VIS_CAMERA_UP))
            view_control.set_zoom(Config.VIS_CAMERA_ZOOM)
        
        self.o3d_initialized_successfully = True # Mark success here
        with self.data_lock:
            self.display_variables["o3d_status"] = "Open3D Scene Initialized and Running"
        logging.info("Open3D scene initialized successfully.")


    def _run_o3d_visualizer(self):
        global OPEN3D_INITIALIZATION_FAILED # Refer to the global flag

        if not OPEN3D_AVAILABLE: # If import failed, this thread shouldn't have started, but check again.
            logging.warning("O3D Visualizer: Aborting run as Open3D is not available.")
            OPEN3D_INITIALIZATION_FAILED = True # Ensure it is set
            with self.data_lock:
                 self.display_variables["o3d_status"] = "Open3D Not Available (Thread Check)"
            return

        self._init_o3d_scene()
        
        if OPEN3D_INITIALIZATION_FAILED or not self.o3d_initialized_successfully or self.o3d_vis is None:
            logging.error("Open3D visualizer: Initialization failed or self.o3d_vis is None. Thread exiting.")
            # display_variables should have been updated in _init_o3d_scene
            OPEN3D_INITIALIZATION_FAILED = True # Redundant but safe
            return

        logging.info("Open3D visualizer thread started, polling for events.")
        
        try:
            while not self.shutdown_event.is_set():
                if OPEN3D_INITIALIZATION_FAILED or self.o3d_vis is None: # Check again in loop
                    logging.info("O3D Visualizer: Loop check found o3d_vis is None or init failed. Stopping poll loop.")
                    break
                
                new_ball_pos_update = None
                try:
                    new_ball_pos_update = self.o3d_update_queue.get_nowait()
                except queue.Empty:
                    pass
                except Exception as e:
                    logging.warning(f"O3D: Error getting from o3d_update_queue: {e}")

                if new_ball_pos_update is not None and self.o3d_volleyball_mesh is not None:
                    current_center = self.o3d_volleyball_mesh.get_center()
                    target_center = np.array(new_ball_pos_update)
                    translation_needed = target_center - current_center
                    self.o3d_volleyball_mesh.translate(translation_needed, relative=True)
                    self.o3d_vis.update_geometry(self.o3d_volleyball_mesh)

                if not self.o3d_vis.poll_events():
                    logging.info("O3D: Poll_events indicated window closed by user.")
                    OPEN3D_INITIALIZATION_FAILED = True # Treat as failure
                    with self.data_lock:
                         self.display_variables["o3d_status"] = "Open3D Window Closed by User"
                    break 
                self.o3d_vis.update_renderer()
                time.sleep(0.01)
        except RuntimeError as e:
            logging.error(f"O3D: Runtime error during poll/render loop (window likely closed): {e}")
            OPEN3D_INITIALIZATION_FAILED = True
            with self.data_lock:
                self.display_variables["o3d_status"] = "Open3D Window Error (Runtime Loop)"
        except Exception as e:
            logging.error(f"O3D: Unexpected error during poll/render loop: {e}", exc_info=True)
            OPEN3D_INITIALIZATION_FAILED = True
            with self.data_lock:
                self.display_variables["o3d_status"] = "Open3D Window Error (Loop Other)"
        finally:
            logging.info("O3D visualizer: Poll loop ended.")
            if self.o3d_vis is not None and self.o3d_initialized_successfully:
                logging.info("O3D: Attempting to destroy Open3D window.")
                try:
                    self.o3d_vis.destroy_window()
                except Exception as e_destroy:
                    logging.error(f"O3D: Error destroying window: {e_destroy}")
            self.o3d_vis = None # Ensure it's None after trying to destroy
            self.o3d_initialized_successfully = False # Mark as no longer successfully running
            if not OPEN3D_INITIALIZATION_FAILED: # If it ended normally (e.g. shutdown_event)
                with self.data_lock:
                    self.display_variables["o3d_status"] = "Open3D Visualizer Stopped"
            logging.info("Open3D visualizer thread resources released.")


    def update_data(self, raw_frame_for_stream=None, processed_frame_for_stream=None,
                    ball_2d_coords=None, ball_pixel_radius=None, ball_3d_world_coords=None,
                    box_corners_world=None, box_dimensions=None,
                    camera_intrinsics=None, camera_extrinsics=None,
                    fps=None, resolution=None, status_message=None):
        with self.data_lock:
            if raw_frame_for_stream is not None:
                self.latest_raw_frame = raw_frame_for_stream
            # if processed_frame_for_stream is not None: # Not used currently
            #     self.latest_processed_frame = processed_frame_for_stream 

            if status_message is not None: self.display_variables["status"] = status_message
            if fps is not None: self.display_variables["fps"] = fps
            if resolution is not None: self.display_variables["resolution"] = resolution

            # o3d_status is mostly managed by the O3D thread itself or __init__
            # Only update here if it's in a generic "running" state and no error flags are set
            if OPEN3D_AVAILABLE and not OPEN3D_INITIALIZATION_FAILED and self.o3d_initialized_successfully:
                if self.display_variables["o3d_status"] != "Open3D Window Closed by User" and \
                   "Error" not in self.display_variables["o3d_status"]:
                   self.display_variables["o3d_status"] = "Open3D Running"
            elif OPEN3D_INITIALIZATION_FAILED and "Error" not in self.display_variables["o3d_status"] and "Failed" not in self.display_variables["o3d_status"]:
                # Update if an error occurred but status wasn't reflecting it yet from this side
                 self.display_variables["o3d_status"] = "Open3D Error Detected"


            if ball_2d_coords and len(ball_2d_coords) >=2 and ball_2d_coords[0] is not None:
                self.display_variables["ball_2d_px"] = f"({ball_2d_coords[0]:.0f}, {ball_2d_coords[1]:.0f})"
            else:
                self.display_variables["ball_2d_px"] = "N/A"
            
            if ball_pixel_radius is not None:
                self.display_variables["ball_pixel_radius_px"] = f"{ball_pixel_radius:.0f}px"
            else:
                self.display_variables["ball_pixel_radius_px"] = "N/A"

            if ball_3d_world_coords and len(ball_3d_world_coords) == 3:
                self.display_variables["ball_3d_m"] = f"X:{ball_3d_world_coords[0]:.2f}, Y:{ball_3d_world_coords[1]:.2f}, Z_vis:{ball_3d_world_coords[2]:.2f}"
                if OPEN3D_AVAILABLE and not OPEN3D_INITIALIZATION_FAILED and self.o3d_initialized_successfully and self.o3d_update_queue is not None:
                    try:
                        self.o3d_update_queue.put_nowait(list(ball_3d_world_coords))
                    except queue.Full:
                        logging.debug("O3D update queue full, dropping position update.")
                    except Exception as e:
                        logging.warning(f"WebUI: Error putting to o3d_update_queue: {e}")
            else:
                self.display_variables["ball_3d_m"] = "N/A"

            if camera_intrinsics:
                if isinstance(camera_intrinsics, str): self.display_variables["cam_intrinsics"] = camera_intrinsics
                else:
                    try: self.display_variables["cam_intrinsics"] = np.array2string(np.array(camera_intrinsics), precision=2, separator=', ', suppress_small=True)
                    except Exception: self.display_variables["cam_intrinsics"] = "Error formatting intrinsics"
            if camera_extrinsics:
                if isinstance(camera_extrinsics, str): self.display_variables["cam_extrinsics"] = camera_extrinsics
                else:
                    rvec_str, tvec_str = "N/A", "N/A"
                    try:
                        if camera_extrinsics.get('rvec') is not None: rvec_str = np.array2string(np.array(camera_extrinsics['rvec']), precision=3, suppress_small=True)
                        if camera_extrinsics.get('tvec') is not None: tvec_str = np.array2string(np.array(camera_extrinsics['tvec']), precision=3, suppress_small=True)
                        self.display_variables["cam_extrinsics"] = f"rvec: {rvec_str}\ntvec: {tvec_str}"
                    except Exception: self.display_variables["cam_extrinsics"] = "Error formatting extrinsics"

    def run(self):
        if not self.app:
            logging.error("Flask app not initialized in WebUIManager.")
            return
        logging.info(f"Starting Flask web server on http://{self.host}:{self.port}")
        try:
            self.app.run(host=self.host, port=self.port, debug=False, use_reloader=False, threaded=True)
        except SystemExit: # Can be raised by Flask on shutdown signals
            logging.info("Flask server SystemExit (likely controlled shutdown).")
        except Exception as e:
            logging.error(f"Failed to start Flask web server: {e}", exc_info=True)
        finally:
            logging.info("Flask web server has stopped.")
            # self.shutdown_event.set() # Ensure main knows server stopped, if not already set.
            self.stop() # Call own stop to ensure O3D thread is handled.

    def stop(self):
        logging.info("WebUIManager stop sequence initiated.")
        if not self.shutdown_event.is_set(): # Signal other parts if this stop is called first
            self.shutdown_event.set() 
        
        if OPEN3D_AVAILABLE and hasattr(self, 'o3d_thread') and self.o3d_thread.is_alive():
            logging.info("Waiting for Open3D thread to join...")
            self.o3d_thread.join(timeout=2.0) # Shorter timeout
            if self.o3d_thread.is_alive():
                logging.warning("Open3D thread did not exit cleanly after join().")
        
        logging.info("WebUIManager stop sequence completed.")


if __name__ == '__main__':
    log_format = Config.LOG_FORMAT if hasattr(Config, 'LOG_FORMAT') else '%(asctime)s - %(levelname)s - %(message)s'
    log_date_format = Config.LOG_DATE_FORMAT if hasattr(Config, 'LOG_DATE_FORMAT') else '%Y-%m-%d %H:%M:%S'
    logging.basicConfig(level=logging.DEBUG, format=log_format, datefmt=log_date_format) # DEBUG for example

    if not OPEN3D_AVAILABLE or OPEN3D_INITIALIZATION_FAILED:
        print("Open3D is not available or failed initial import. Web UI will run without 3D visualization.")

    example_shutdown_event = threading.Event()
    ui_manager = WebUIManager(port=Config.WEB_PORT if hasattr(Config, 'WEB_PORT') else 8000,
                              shutdown_event_ref=example_shutdown_event)
    
    flask_thread = threading.Thread(target=ui_manager.run, name="TestFlaskThread", daemon=True)
    flask_thread.start()
    
    time.sleep(1.5) # Give server a moment to start
    if not flask_thread.is_alive():
        logging.error("Failed to start Flask thread in example. Exiting.")
        exit()

    logging.info(f"Web UI example started. Open3D status: {ui_manager.display_variables['o3d_status']}")
    logging.info(f"Access web page at http://localhost:{ui_manager.port}")

    try:
        count = 0
        cam_res_w = Config.CAM_REQUESTED_WIDTH if hasattr(Config,'CAM_REQUESTED_WIDTH') else 640
        cam_res_h = Config.CAM_REQUESTED_HEIGHT if hasattr(Config,'CAM_REQUESTED_HEIGHT') else 480
        cam_fps = Config.CAM_REQUESTED_FPS if hasattr(Config, 'CAM_REQUESTED_FPS') else 30
        vball_rad_m = Config.VOLLEYBALL_RADIUS_M if hasattr(Config, 'VOLLEYBALL_RADIUS_M') else 0.105
        cam_mtx_list = Config.CAMERA_INTRINSIC_MTX.tolist() if hasattr(Config, 'CAMERA_INTRINSIC_MTX') else "N/A"

        for _ in range(60): # Run for about 60 seconds
            if example_shutdown_event.is_set(): break
            time.sleep(1)
            count +=1

            sim_ball_x = 1.0 + 0.5 * np.sin(count * 0.2) 
            sim_ball_y = 1.0 + 0.5 * np.cos(count * 0.3) 
            sim_ball_z_vis = vball_rad_m 

            dummy_frame = np.random.randint(0, 255, (240, 320, 3), dtype=np.uint8)
            cv2.putText(dummy_frame, f"Frame {count}", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)

            ui_manager.update_data(
                raw_frame_for_stream=dummy_frame,
                ball_3d_world_coords=[sim_ball_x, sim_ball_y, sim_ball_z_vis],
                status_message=f"Simulating frame {count}",
                fps= cam_fps - (count % 5), 
                resolution=f"{cam_res_w}x{cam_res_h}",
                ball_2d_coords=(int(sim_ball_x*100 + 50), int(sim_ball_y*100 + 50)), 
                ball_pixel_radius= 20 + (count%10),
                camera_intrinsics=cam_mtx_list,
                camera_extrinsics={'rvec': [0.1,0.2,0.3], 'tvec': [1.0,0.5,2.0]}
            )
            if count % 5 == 0: # Log less frequently
                 logging.debug(f"Example: Sent update {count}. O3D Status: {ui_manager.display_variables['o3d_status']}")
        logging.info("Example: Simulated run finished.")

    except KeyboardInterrupt:
        logging.info("Keyboard interrupt received in example. Shutting down.")
    finally:
        if not example_shutdown_event.is_set():
            example_shutdown_event.set()
        
        # ui_manager.stop() # Flask server is daemon, main thread exit will trigger its shutdown via event
        # Flask thread stop is better handled by main thread ending or specific shutdown route in real app.
        # For this example, signaling the event and letting daemon threads exit is okay.
        
        if flask_thread.is_alive():
            logging.info("Example: Flask thread should stop as it is a daemon and main is ending.")
        
        # If O3D thread was started and might be alive (e.g. if shutdown event wasn't passed or it's stuck)
        if OPEN3D_AVAILABLE and hasattr(ui_manager, 'o3d_thread') and ui_manager.o3d_thread.is_alive():
            logging.info("Example: Waiting for O3D thread to join explicitly if it's still up...")
            ui_manager.o3d_thread.join(timeout=2.0)
            if ui_manager.o3d_thread.is_alive():
                 logging.warning("Example: O3D thread still alive after explicit join.")

        logging.info("Web UI example completed.")