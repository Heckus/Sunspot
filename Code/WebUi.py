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
    # Attempt to set environment for headless rendering if issues persist with VNC/GLX
    # This should ideally be set BEFORE open3d is imported, e.g., via os.environ
    # For now, we'll rely on catching the window creation error.
    # import os
    # if os.environ.get("DISPLAY") is None or "socket" in os.environ.get("DISPLAY", ""): # Basic check for headless env
    #     logging.info("Open3D: Attempting to enable CPU rendering for headless/VNC environment.")
    #     o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)
        # The following are ways to attempt headless but might need to be set earlier
        # or might not work depending on Open3D version and build.
        # try:
        #    o3d.visualization.webrtc_server.enable_webrtc() # If WebRTC based view is an option
        # except AttributeError:
        #    pass # Older Open3D

    OPEN3D_AVAILABLE = True
except ImportError:
    logging.error("Open3D library not found. 3D visualization will be disabled.")
    logging.error("Please install it: pip install open3d")
except Exception as e:
    logging.error(f"Open3D import failed with an unexpected error: {e}. 3D visualization will be disabled.")


class WebUIManager:
    """
    Manages the Flask web server for displaying variables and streaming 2D video,
    and runs the Open3D visualizer in a separate thread for 3D model display.
    """

    def __init__(self, host='0.0.0.0', port=Config.WEB_PORT, shutdown_event_ref=None):
        global OPEN3D_INITIALIZATION_FAILED
        self.host = host
        self.port = port
        self.app = Flask(__name__)
        self.shutdown_event = shutdown_event_ref if shutdown_event_ref else threading.Event()

        # Data attributes to be updated by main.py
        self.latest_raw_frame = None # For MJPEG stream
        self.latest_processed_frame = None # Optional, for a second stream
        self.ball_3d_coords_vis = None # Expected [X, Y, Z_center_of_sphere] for Open3D
        self.box_corners_world = Config.WORLD_BOX_CORNERS_M.tolist() # Use from Config
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
            "o3d_status": "Open3D Not Initialized" if OPEN3D_AVAILABLE else "Open3D Not Available"
        }
        self.data_lock = threading.Lock() # To protect access to shared data

        # Open3D specific attributes
        self.o3d_vis = None
        self.o3d_box_mesh = None
        self.o3d_volleyball_mesh = None
        self.o3d_world_axes = None
        self.o3d_initialized_successfully = False # More specific flag
        self.o3d_update_queue = queue.Queue(maxsize=5) # Queue for 3D position updates

        if not OPEN3D_AVAILABLE:
            logging.warning("WebUI: Open3D library not found or failed to import, 3D visualization will not run.")
            self.display_variables["o3d_status"] = "Open3D Not Available"
        elif OPEN3D_INITIALIZATION_FAILED: # Check global flag set during import
            logging.warning("WebUI: Open3D import previously failed, 3D visualization will not run.")
            self.display_variables["o3d_status"] = "Open3D Import Error"
        else:
            # Start Open3D in a separate thread
            self.o3d_thread = threading.Thread(target=self._run_o3d_visualizer, name="Open3DThread", daemon=True)
            self.o3d_thread.start()

        self._setup_routes()

    def _setup_routes(self):
        """Defines Flask routes."""
        @self.app.route('/')
        def index():
            # Removed: <meta http-equiv="refresh" content="1">
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
                            this.src = ''; // Clear broken icon
                            console.error('Error loading video stream.');
                            // Optionally, display a more user-friendly message on the image placeholder
                            // const ctx = this.getContext('2d'); // This won't work directly on <img>
                            // Need to replace img with a canvas or use a placeholder div
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

                    // Update data every second
                    setInterval(updateDataFields, 1000);
                    // Initial call to populate fields
                    document.addEventListener('DOMContentLoaded', updateDataFields);
                </script>
            </body>
            </html>
            """
            with self.data_lock:
                # o3d_status is now part of display_variables, so it's included
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
        """Generates MJPEG stream from the latest raw frame."""
        logging.info("MJPEG stream client connected.")
        frame_num = 0
        while not self.shutdown_event.is_set():
            frame_to_encode = None
            with self.data_lock:
                if self.latest_raw_frame is not None:
                    frame_to_encode = self.latest_raw_frame.copy()

            if frame_to_encode is None:
                # Create a placeholder if no frame is available
                # Use CAM_REQUESTED_WIDTH/HEIGHT from Config for placeholder size
                ph_height = Config.CAM_REQUESTED_HEIGHT // 2 if Config.CAM_REQUESTED_HEIGHT else 240
                ph_width = Config.CAM_REQUESTED_WIDTH // 2 if Config.CAM_REQUESTED_WIDTH else 320
                frame_to_encode = np.zeros((ph_height, ph_width, 3), dtype=np.uint8)
                cv2.putText(frame_to_encode, "No Stream", (30, ph_height // 2), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                time.sleep(0.1) # Wait a bit if no frame
            else:
                # Optionally draw frame number or timestamp for debugging stream
                # cv2.putText(frame_to_encode, f"F: {frame_num}", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0),2)
                frame_num+=1


            try:
                flag, encoded_image = cv2.imencode('.jpg', frame_to_encode, [cv2.IMWRITE_JPEG_QUALITY, Config.WEB_STREAM_JPEG_QUALITY if hasattr(Config, 'WEB_STREAM_JPEG_QUALITY') else 75])
                if not flag:
                    logging.warning("MJPEG: Could not encode frame.")
                    time.sleep(0.1)
                    continue
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + bytearray(encoded_image) + b'\r\n')
            except GeneratorExit:
                logging.info("MJPEG stream client disconnected (GeneratorExit).")
                break # Exit loop if client disconnects
            except ConnectionResetError:
                logging.info("MJPEG stream client connection reset.")
                break
            except BrokenPipeError:
                logging.info("MJPEG stream client connection broken pipe.")
                break
            except Exception as e:
                logging.error(f"Error in MJPEG stream: {e}", exc_info=True)
                break
            
            # Crude FPS limit for stream - use a config value if precise control needed
            sleep_duration = 1.0 / (Config.WEB_STREAM_MAX_FPS if hasattr(Config, 'WEB_STREAM_MAX_FPS') else Config.CAM_REQUESTED_FPS)
            time.sleep(max(0.01, sleep_duration)) 
        logging.info("MJPEG stream generator stopped.")


    def _init_o3d_scene(self):
        """Initializes the Open3D scene with the box and volleyball."""
        global OPEN3D_INITIALIZATION_FAILED
        if not OPEN3D_AVAILABLE or OPEN3D_INITIALIZATION_FAILED:
            self.display_variables["o3d_status"] = "Open3D Not Available or Failed Previous Init"
            return

        self.o3d_vis = o3d.visualization.Visualizer()
        # Attempt to create window. This is the critical part that might fail.
        try:
            created_successfully = self.o3d_vis.create_window(
                                   window_name=Config.VIS_WINDOW_TITLE,
                                   width=Config.VIS_WINDOW_WIDTH,
                                   height=Config.VIS_WINDOW_HEIGHT,
                                   visible=True # Explicitly true, though default
                                   )
            if not created_successfully:
                logging.error("Open3D: o3d_vis.create_window() returned False. Window creation failed.")
                OPEN3D_INITIALIZATION_FAILED = True
                self.display_variables["o3d_status"] = "Open3D Window Creation Failed (ret False)"
                if self.o3d_vis: self.o3d_vis.destroy_window() # Clean up if partially created
                self.o3d_vis = None
                return
        except RuntimeError as e:
            # This often catches GLFW/GLX errors like 'GLFW Error: GLX: Failed to create context'
            logging.error(f"Open3D: Runtime_Error during o3d_vis.create_window(): {e}")
            logging.error("This often occurs in VNC/headless environments without proper GL context.")
            logging.error("Try running with a direct display or configure VNC for OpenGL (e.g., VirtualGL).")
            OPEN3D_INITIALIZATION_FAILED = True
            self.display_variables["o3d_status"] = f"Open3D Window Creation Error (Runtime: {e})"
            if self.o3d_vis: self.o3d_vis.destroy_window() # Clean up
            self.o3d_vis = None
            return # Do not proceed with scene setup

        # Check if render option is available AFTER successful window creation
        opt = self.o3d_vis.get_render_option()
        if opt is None:
            logging.error("Open3D: get_render_option() returned None even after window creation attempt. Disabling Open3D.")
            OPEN3D_INITIALIZATION_FAILED = True
            self.display_variables["o3d_status"] = "Open3D Render Options Failed"
            if self.o3d_vis: self.o3d_vis.destroy_window()
            self.o3d_vis = None
            return

        opt.background_color = np.asarray([0.1, 0.1, 0.1]) # Dark background
        opt.light_on = True
        opt.point_size = 2.0


        box_w, box_d, box_h_vis = self.box_dimensions[0], self.box_dimensions[1], self.box_dimensions[2]
        # Create box centered at (0,0,0), then translate it to (W/2, D/2, H_vis/2) in world.
        self.o3d_box_mesh = o3d.geometry.TriangleMesh.create_box(width=box_w, height=box_d, depth=box_h_vis, create_uv_map=True, map_texture_to_each_face=True)
        self.o3d_box_mesh.translate([box_w / 2, box_d / 2, box_h_vis / 2], relative=False)
        self.o3d_box_mesh.paint_uniform_color(Config.VIS_BOX_COLOR)
        self.o3d_vis.add_geometry(self.o3d_box_mesh, reset_bounding_box=True)


        self.o3d_volleyball_mesh = o3d.geometry.TriangleMesh.create_sphere(radius=Config.VOLLEYBALL_RADIUS_M)
        self.o3d_volleyball_mesh.paint_uniform_color(Config.VIS_VOLLEYBALL_COLOR)
        initial_ball_pos = [box_w / 2, box_d / 2, Config.VOLLEYBALL_RADIUS_M + box_h_vis] # slightly above box
        self.o3d_volleyball_mesh.translate(initial_ball_pos, relative=False)
        self.o3d_vis.add_geometry(self.o3d_volleyball_mesh, reset_bounding_box=False)

        if Config.VIS_AXES_ENABLED:
            self.o3d_world_axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0,0,0])
            self.o3d_vis.add_geometry(self.o3d_world_axes, reset_bounding_box=False)


        view_control = self.o3d_vis.get_view_control()
        if view_control:
            # Look at the center of the box base
            look_at_target = np.array([box_w / 2, box_d / 2, 0.0]) # center of box base
            # Camera position relative to the front of the box
            # Example: move back along Y, slightly to the side (X), and up (Z)
            camera_pos = look_at_target + np.array(Config.VIS_CAMERA_FRONT) * (-2) # Config.VIS_CAMERA_FRONT is direction vector, invert and scale for pos
            
            view_control.set_lookat(look_at_target)
            # set_front is the direction vector from eye to lookat point.
            # If VIS_CAMERA_FRONT is defined as (eye - lookat), then it's correct.
            # If it's (lookat - eye), then it needs to be negated.
            # Let's assume Config.VIS_CAMERA_FRONT is (eye_direction_vector_from_lookat)
            view_control.set_front(np.array(Config.VIS_CAMERA_FRONT)) # Direction camera is pointing
            view_control.set_up(np.array(Config.VIS_CAMERA_UP))
            view_control.set_zoom(Config.VIS_CAMERA_ZOOM)
        
        self.o3d_initialized_successfully = True
        self.display_variables["o3d_status"] = "Open3D Scene Initialized"
        logging.info("Open3D scene initialized successfully.")


    def _run_o3d_visualizer(self):
        """Runs the Open3D visualization loop. Should be run in a separate thread."""
        global OPEN3D_INITIALIZATION_FAILED
        if not OPEN3D_AVAILABLE or OPEN3D_INITIALIZATION_FAILED: # Check global flag too
            logging.warning("O3D Visualizer: Aborting run as Open3D is not available or failed init.")
            self.display_variables["o3d_status"] = "Open3D Not Run (Unavailable or Failed Init)"
            return

        self._init_o3d_scene() # Initialize scene elements
        
        # Check if initialization was successful within _init_o3d_scene
        if not self.o3d_initialized_successfully or not self.o3d_vis:
            logging.error("Open3D visualizer could not be initialized or window creation failed. Thread exiting.")
            # display_variables updated within _init_o3d_scene
            OPEN3D_INITIALIZATION_FAILED = True # Ensure this is set
            return

        logging.info("Open3D visualizer thread started, polling for events.")
        # last_ball_pos_o3d = None # Not currently used

        while not self.shutdown_event.is_set():
            if not self.o3d_vis or OPEN3D_INITIALIZATION_FAILED: # Double check, e.g. if window closed by user
                logging.info("O3D Visualizer: o3d_vis is None or init failed, stopping poll loop.")
                break
            
            new_ball_pos_update = None
            try:
                # Check queue for new ball position data from main thread
                new_ball_pos_update = self.o3d_update_queue.get_nowait()
            except queue.Empty:
                pass # No new update
            except Exception as e: # Catch other potential queue errors
                logging.warning(f"O3D: Error getting from o3d_update_queue: {e}")


            if new_ball_pos_update is not None:
                if self.o3d_volleyball_mesh is not None:
                    current_center = self.o3d_volleyball_mesh.get_center()
                    target_center = np.array(new_ball_pos_update)
                    translation_needed = target_center - current_center
                    self.o3d_volleyball_mesh.translate(translation_needed, relative=True)
                    self.o3d_vis.update_geometry(self.o3d_volleyball_mesh)
                    # logging.debug(f"O3D: Ball moved to {target_center}")

            # Poll events and update renderer
            try:
                if not self.o3d_vis.poll_events(): # Returns false if window closed
                    logging.info("O3D: Poll_events indicated window closed by user.")
                    OPEN3D_INITIALIZATION_FAILED = True # Treat as failure to continue
                    self.display_variables["o3d_status"] = "Open3D Window Closed by User"
                    break 
                self.o3d_vis.update_renderer()
            except RuntimeError as e: # Catch errors if window is unexpectedly closed/destroyed
                logging.error(f"O3D: Runtime error during poll_events/update_renderer (window likely closed): {e}")
                OPEN3D_INITIALIZATION_FAILED = True
                self.display_variables["o3d_status"] = "Open3D Window Error (Runtime)"
                break
            except Exception as e:
                logging.error(f"O3D: Unexpected error during poll_events/update_renderer: {e}")
                OPEN3D_INITIALIZATION_FAILED = True
                self.display_variables["o3d_status"] = "Open3D Window Error (Other)"
                break


            time.sleep(0.01) # Small sleep to yield CPU, ~100Hz polling

        # Cleanup Open3D window
        if self.o3d_vis:
            logging.info("O3D: Closing Open3D window.")
            self.o3d_vis.destroy_window()
            self.o3d_vis = None
        if not OPEN3D_INITIALIZATION_FAILED and self.o3d_initialized_successfully: # Only if it was running fine
             self.display_variables["o3d_status"] = "Open3D Visualizer Stopped"
        logging.info("Open3D visualizer thread stopped and window closed.")


    def update_data(self, raw_frame_for_stream=None, processed_frame_for_stream=None,
                    ball_2d_coords=None, ball_pixel_radius=None, ball_3d_world_coords=None,
                    box_corners_world=None, box_dimensions=None,
                    camera_intrinsics=None, camera_extrinsics=None,
                    fps=None, resolution=None, status_message=None):
        """
        Thread-safe method for the main application to update data for the WebUI and Open3D.
        Args:
            ball_3d_world_coords (list or np.ndarray): [X, Y, Z_sphere_center] for Open3D ball.
        """
        with self.data_lock:
            if raw_frame_for_stream is not None:
                self.latest_raw_frame = raw_frame_for_stream
            if processed_frame_for_stream is not None:
                self.latest_processed_frame = processed_frame_for_stream 

            if status_message is not None: self.display_variables["status"] = status_message
            if fps is not None: self.display_variables["fps"] = fps
            if resolution is not None: self.display_variables["resolution"] = resolution

            # Update o3d_status if it's not one of the error/terminal states
            current_o3d_status = self.display_variables.get("o3d_status", "")
            if OPEN3D_INITIALIZATION_FAILED or not OPEN3D_AVAILABLE:
                pass # Keep the error status
            elif self.o3d_initialized_successfully and (self.o3d_vis and self.o3d_vis.get_window_name() != "") : # Check if window seems alive
                 self.display_variables["o3d_status"] = "Open3D Running"
            elif not self.o3d_initialized_successfully and "Initialized" not in current_o3d_status and "Error" not in current_o3d_status:
                 self.display_variables["o3d_status"] = "Open3D Initializing..."


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
                # Check if it's already a string (e.g. from a previous N/A or manual set)
                if isinstance(camera_intrinsics, str):
                    self.display_variables["cam_intrinsics"] = camera_intrinsics
                else: # Assume numpy array or list of lists
                    try:
                        self.display_variables["cam_intrinsics"] = np.array2string(np.array(camera_intrinsics), precision=2, separator=', ', suppress_small=True)
                    except Exception:
                        self.display_variables["cam_intrinsics"] = "Error formatting intrinsics"

            if camera_extrinsics:
                if isinstance(camera_extrinsics, str):
                     self.display_variables["cam_extrinsics"] = camera_extrinsics
                else:
                    rvec_str = "N/A"
                    tvec_str = "N/A"
                    try:
                        if camera_extrinsics.get('rvec') is not None:
                            rvec_str = np.array2string(np.array(camera_extrinsics['rvec']), precision=3, suppress_small=True)
                        if camera_extrinsics.get('tvec') is not None:
                            tvec_str = np.array2string(np.array(camera_extrinsics['tvec']), precision=3, suppress_small=True)
                        self.display_variables["cam_extrinsics"] = f"rvec: {rvec_str}\ntvec: {tvec_str}"
                    except Exception:
                        self.display_variables["cam_extrinsics"] = "Error formatting extrinsics"


            if box_corners_world: self.box_corners_world = box_corners_world
            if box_dimensions: self.box_dimensions = box_dimensions


    def run(self):
        """Starts the Flask web server."""
        if not self.app:
            logging.error("Flask app not initialized in WebUIManager.")
            return
        logging.info(f"Starting Flask web server on http://{self.host}:{self.port}")
        try:
            # Using Flask's dev server with threaded=True. For production, use Waitress or Gunicorn.
            self.app.run(host=self.host, port=self.port, debug=False, use_reloader=False, threaded=True)
        except SystemExit:
            logging.info("Flask server SystemExit (likely due to shutdown signal).")
        except Exception as e:
            logging.error(f"Failed to start Flask web server: {e}", exc_info=True)
        finally:
            logging.info("Flask web server has stopped.")
            self.stop() 

    def stop(self):
        """Signals all components to shut down."""
        logging.info("WebUIManager stop sequence initiated.")
        if not self.shutdown_event.is_set():
            self.shutdown_event.set()
        
        # Wait for O3D thread to finish if it was started and is alive
        if OPEN3D_AVAILABLE and hasattr(self, 'o3d_thread') and self.o3d_thread.is_alive():
            logging.info("Waiting for Open3D thread to join...")
            self.o3d_thread.join(timeout=3.0) # Reduced timeout
            if self.o3d_thread.is_alive():
                logging.warning("Open3D thread did not exit cleanly after join().")
                # The thread itself should handle o3d_vis.destroy_window() on shutdown_event
        
        # Additional cleanup for Flask server if needed, though self.app.run usually handles its own shutdown.
        # For dev server, sending a request to a shutdown route is one way, but daemon threads + event is often enough.
        logging.info("WebUIManager stop sequence completed.")


if __name__ == '__main__':
    # BasicConfig for logging if run standalone
    # Ensure Config.py is in the same directory or python path for these Config vars
    log_format = Config.LOG_FORMAT if hasattr(Config, 'LOG_FORMAT') else '%(asctime)s - %(levelname)s - %(message)s'
    log_date_format = Config.LOG_DATE_FORMAT if hasattr(Config, 'LOG_DATE_FORMAT') else '%Y-%m-%d %H:%M:%S'
    logging.basicConfig(level=logging.INFO, format=log_format, datefmt=log_date_format)

    # Check OPEN3D_AVAILABLE more robustly
    if not OPEN3D_AVAILABLE or OPEN3D_INITIALIZATION_FAILED: # Check global flag
        print("Open3D is not available or failed to initialize. This example will run the web server part without 3D visualization.")
        if not OPEN3D_AVAILABLE:
            print("Please install Open3D: pip install open3d")

    # Create a dummy shutdown event for standalone testing
    example_shutdown_event = threading.Event()

    ui_manager = WebUIManager(port=Config.WEB_PORT if hasattr(Config, 'WEB_PORT') else 8000,
                              shutdown_event_ref=example_shutdown_event)
    
    # Start Flask in a separate thread for standalone testing
    flask_thread = threading.Thread(target=ui_manager.run, name="TestFlaskThread", daemon=True)
    flask_thread.start()
    
    time.sleep(1) # Give server a moment to start
    if not flask_thread.is_alive():
        logging.error("Failed to start Flask thread in example. Exiting.")
        exit()

    logging.info(f"Web UI example started. Open3D window should appear if library is installed and initialized correctly.")
    logging.info(f"Access web page at http://localhost:{ui_manager.port}")

    try:
        count = 0
        cam_res_w = Config.CAM_REQUESTED_WIDTH if hasattr(Config,'CAM_REQUESTED_WIDTH') else 640
        cam_res_h = Config.CAM_REQUESTED_HEIGHT if hasattr(Config,'CAM_REQUESTED_HEIGHT') else 480
        cam_fps = Config.CAM_REQUESTED_FPS if hasattr(Config, 'CAM_REQUESTED_FPS') else 30
        vball_rad_m = Config.VOLLEYBALL_RADIUS_M if hasattr(Config, 'VOLLEYBALL_RADIUS_M') else 0.105
        cam_mtx_list = Config.CAMERA_INTRINSIC_MTX.tolist() if hasattr(Config, 'CAMERA_INTRINSIC_MTX') else "N/A"


        while not example_shutdown_event.is_set():
            time.sleep(1) # Update data every second
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
            if count % 10 == 0:
                 logging.info(f"Example: Sent update {count} to UI. Ball at ({sim_ball_x:.2f}, {sim_ball_y:.2f}, {sim_ball_z_vis:.2f})")
            if count > 60: # Run for a minute then shutdown for test
                logging.info("Example: Reached simulated end. Shutting down.")
                # example_shutdown_event.set() # This would be for external shutdown

    except KeyboardInterrupt:
        logging.info("Keyboard interrupt received in example. Shutting down.")
    finally:
        if not example_shutdown_event.is_set(): # Ensure it's set
            example_shutdown_event.set()
        
        ui_manager.stop() # Call the manager's stop method
        
        if flask_thread.is_alive():
            logging.info("Example: Waiting for Flask thread to join...")
            flask_thread.join(timeout=3.0) # Give it some time
            if flask_thread.is_alive():
                logging.warning("Example: Flask thread still alive after join.")
        
        logging.info("Web UI example finished.")