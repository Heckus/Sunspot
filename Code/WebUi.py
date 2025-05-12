# web_ui.py

import threading
import time
import logging
import queue # For passing data between main thread and Open3D thread
import numpy as np
import cv2 # For MJPEG streaming
from flask import Flask, Response, render_template_string, jsonify

import Config # Assuming Config.py has visualization settings

try:
    import open3d as o3d
    OPEN3D_AVAILABLE = True
except ImportError:
    OPEN3D_AVAILABLE = False
    logging.error("Open3D library not found. 3D visualization will be disabled.")
    logging.error("Please install it: pip install open3d")


class WebUIManager:
    """
    Manages the Flask web server for displaying variables and streaming 2D video,
    and runs the Open3D visualizer in a separate thread for 3D model display.
    """

    def __init__(self, host='0.0.0.0', port=Config.WEB_PORT, shutdown_event_ref=None):
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
            "fps": 0.0,
            "resolution": "N/A",
            "cam_intrinsics": "N/A",
            "cam_extrinsics": "N/A"
        }
        self.data_lock = threading.Lock() # To protect access to shared data

        # Open3D specific attributes
        self.o3d_vis = None
        self.o3d_box_mesh = None
        self.o3d_volleyball_mesh = None
        self.o3d_world_axes = None
        self.o3d_initialized = False
        self.o3d_update_queue = queue.Queue(maxsize=5) # Queue for 3D position updates

        if not OPEN3D_AVAILABLE:
            logging.warning("WebUI: Open3D not available, 3D visualization will not run.")
        else:
            # Start Open3D in a separate thread
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
                <meta http-equiv="refresh" content="1"> {# Refresh page every 1 second to update variables #}
                <style>
                    body { font-family: sans-serif; margin: 20px; background-color: #f4f4f4; color: #333; }
                    .container { display: flex; max-width: 1200px; margin: auto; background: #fff; padding: 20px; box-shadow: 0 0 10px rgba(0,0,0,0.1); }
                    .stream-container { flex: 2; padding-right: 20px; }
                    .data-container { flex: 1; border-left: 1px solid #ddd; padding-left: 20px; }
                    h1, h2 { color: #333; }
                    img { display: block; border: 1px solid #ccc; margin-bottom:10px; max-width:100%; }
                    p { margin: 5px 0; }
                    strong { color: #555; }
                </style>
            </head>
            <body>
                <div class="container">
                    <div class="stream-container">
                        <h1>Camera Stream (2D)</h1>
                        <img id="video_stream" src="/video_feed" alt="Loading camera stream...">
                        <p><em>Note: The 3D visualization runs in a separate Open3D window.</em></p>
                    </div>
                    <div class="data-container">
                        <h2>Live Data</h2>
                        <p><strong>Status:</strong> <span id="status">{{ variables.status }}</span></p>
                        <p><strong>FPS:</strong> <span id="fps">{{ "%.2f" | format(variables.fps) }}</span></p>
                        <p><strong>Resolution:</strong> <span id="resolution">{{ variables.resolution }}</span></p>
                        <hr>
                        <p><strong>Ball 2D (px):</strong> <span id="ball_2d_px">{{ variables.ball_2d_px }}</span></p>
                        <p><strong>Ball 3D (m):</strong> <span id="ball_3d_m">{{ variables.ball_3d_m }}</span></p>
                        <hr>
                        <p><strong>Cam Intrinsics:</strong> <pre id="cam_intrinsics" style="font-size:0.8em; white-space:pre-wrap;">{{ variables.cam_intrinsics }}</pre></p>
                        <p><strong>Cam Extrinsics:</strong> <pre id="cam_extrinsics" style="font-size:0.8em; white-space:pre-wrap;">{{ variables.cam_extrinsics }}</pre></p>
                    </div>
                </div>
                <script>
                    // Basic error handling for the stream image
                    const streamImg = document.getElementById('video_stream');
                    if (streamImg) {
                        streamImg.onerror = function() {
                            this.alt = 'Camera stream failed to load or is unavailable.';
                            console.error('Error loading video stream.');
                        };
                    }
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
        """Generates MJPEG stream from the latest raw frame."""
        logging.info("MJPEG stream client connected.")
        while not self.shutdown_event.is_set():
            frame_to_encode = None
            with self.data_lock:
                if self.latest_raw_frame is not None:
                    frame_to_encode = self.latest_raw_frame.copy()

            if frame_to_encode is None:
                # Create a placeholder if no frame is available
                frame_to_encode = np.zeros((Config.CAM_REQUESTED_HEIGHT // 2, Config.CAM_REQUESTED_WIDTH // 2, 3), dtype=np.uint8)
                cv2.putText(frame_to_encode, "No Stream", (30, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                time.sleep(0.1) # Wait a bit if no frame

            try:
                flag, encoded_image = cv2.imencode('.jpg', frame_to_encode, [cv2.IMWRITE_JPEG_QUALITY, 75])
                if not flag:
                    logging.warning("MJPEG: Could not encode frame.")
                    time.sleep(0.1)
                    continue
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + bytearray(encoded_image) + b'\r\n')
            except GeneratorExit:
                logging.info("MJPEG stream client disconnected.")
                break
            except Exception as e:
                logging.error(f"Error in MJPEG stream: {e}")
                break
            time.sleep(1.0 / Config.CAM_REQUESTED_FPS) # Crude FPS limit for stream
        logging.info("MJPEG stream generator stopped.")


    def _init_o3d_scene(self):
        """Initializes the Open3D scene with the box and volleyball."""
        if not OPEN3D_AVAILABLE: return

        self.o3d_vis = o3d.visualization.Visualizer()
        self.o3d_vis.create_window(window_name=Config.VIS_WINDOW_TITLE,
                                   width=Config.VIS_WINDOW_WIDTH,
                                   height=Config.VIS_WINDOW_HEIGHT)
        opt = self.o3d_vis.get_render_option()
        opt.background_color = np.asarray([0.1, 0.1, 0.1]) # Dark background

        # Create the 2x2m box on the ground (Z=0 plane)
        # PDF Sec 5.2: Box from (0,0,-0.025) to (2,2,0.025) to have base at Z=0
        # Open3D create_box is centered at origin by default.
        box_w, box_d, box_h_vis = self.box_dimensions[0], self.box_dimensions[1], self.box_dimensions[2]
        self.o3d_box_mesh = o3d.geometry.TriangleMesh.create_box(width=box_w, height=box_d, depth=box_h_vis)
        # Translate so one corner (0,0,0 in world) is at (-box_w/2, -box_d/2, -box_h_vis/2) in its local coords
        # Then translate its center to (box_w/2, box_d/2, -box_h_vis/2 + box_h_vis/2) = (1,1,0) for its base origin
        # More simply: translate its center to (world_box_center_X, world_box_center_Y, world_box_base_Z_center)
        # The box mesh is created centered at (0,0,0). We want its base at Z=0,
        # and spanning from world (0,0,0) to (2,2,0).
        # So, its center needs to be at (1, 1, box_h_vis/2 - box_h_vis) = (1,1, -box_h_vis/2) if depth is height.
        # If create_box uses width, height, depth as X, Y, Z dimensions respectively:
        # To place it with one corner at (0,0,0) and extending to (W,D,H_vis),
        # translate its center from (0,0,0) to (W/2, D/2, H_vis/2).
        # However, the PDF states the box is ON the ground (Z=0).
        # Let's follow PDF: center it at (1,1,0) for its base, then translate so Z=0 is the bottom face.
        # This requires the created box to be translated by (W/2, D/2, H_vis/2)
        # The pdf example (sec 5.2) seems to create a box centered at origin, then translate it
        # by (1.0, 1.0, -0.025) if box_depth is 0.05. This seems to place its center at (1,1,-0.025)
        # which would make its bottom face at Z=-0.05.
        # Let's make the box base at Z=0.
        # Create box centered at (0,0,0), then translate it to (W/2, D/2, H_vis/2) in world.
        self.o3d_box_mesh.translate([box_w / 2, box_d / 2, box_h_vis / 2], relative=False)
        self.o3d_box_mesh.paint_uniform_color(Config.VIS_BOX_COLOR)
        self.o3d_vis.add_geometry(self.o3d_box_mesh, reset_bounding_box=True)


        # Create the volleyball model (sphere)
        self.o3d_volleyball_mesh = o3d.geometry.TriangleMesh.create_sphere(radius=Config.VOLLEYBALL_RADIUS_M)
        self.o3d_volleyball_mesh.paint_uniform_color(Config.VIS_VOLLEYBALL_COLOR)
        # Initial position (e.g., center of the box, raised by its radius)
        initial_ball_pos = [box_w / 2, box_d / 2, Config.VOLLEYBALL_RADIUS_M]
        self.o3d_volleyball_mesh.translate(initial_ball_pos, relative=False)
        self.o3d_vis.add_geometry(self.o3d_volleyball_mesh, reset_bounding_box=False)

        # Optional: Add world coordinate axes
        if Config.VIS_AXES_ENABLED:
            self.o3d_world_axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0,0,0])
            self.o3d_vis.add_geometry(self.o3d_world_axes, reset_bounding_box=False)


        # Setup initial camera view for Open3D
        view_control = self.o3d_vis.get_view_control()
        if view_control:
            view_control.set_lookat(np.array(Config.VIS_CAMERA_LOOKAT))
            view_control.set_front(np.array(Config.VIS_CAMERA_FRONT))
            view_control.set_up(np.array(Config.VIS_CAMERA_UP))
            view_control.set_zoom(Config.VIS_CAMERA_ZOOM)

        self.o3d_initialized = True
        logging.info("Open3D scene initialized.")


    def _run_o3d_visualizer(self):
        """Runs the Open3D visualization loop. Should be run in a separate thread."""
        if not OPEN3D_AVAILABLE:
            return

        self._init_o3d_scene() # Initialize scene elements
        if not self.o3d_initialized or not self.o3d_vis:
            logging.error("Open3D visualizer could not be initialized.")
            return

        logging.info("Open3D visualizer thread started.")
        last_ball_pos_o3d = None

        while not self.shutdown_event.is_set():
            new_ball_pos_update = None
            try:
                # Check queue for new ball position data from main thread
                new_ball_pos_update = self.o3d_update_queue.get_nowait()
            except queue.Empty:
                pass # No new update

            if new_ball_pos_update is not None:
                if self.o3d_volleyball_mesh is not None:
                    current_center = self.o3d_volleyball_mesh.get_center()
                    target_center = np.array(new_ball_pos_update)

                    # Calculate translation needed and apply it
                    # This is more robust than assuming it's always relative to (0,0,0)
                    translation_needed = target_center - current_center
                    self.o3d_volleyball_mesh.translate(translation_needed, relative=True)
                    self.o3d_vis.update_geometry(self.o3d_volleyball_mesh)
                    last_ball_pos_o3d = target_center.copy()
                    # logging.debug(f"O3D: Ball moved to {target_center}")

            # Poll events and update renderer
            if self.o3d_vis:
                self.o3d_vis.poll_events()
                self.o3d_vis.update_renderer()
            else: # Should not happen if initialized
                break

            time.sleep(0.01) # Small sleep to yield CPU, ~100Hz polling

        # Cleanup Open3D window
        if self.o3d_vis:
            self.o3d_vis.destroy_window()
            self.o3d_vis = None
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
                self.latest_processed_frame = processed_frame_for_stream # For potential second stream

            if status_message is not None: self.display_variables["status"] = status_message
            if fps is not None: self.display_variables["fps"] = fps
            if resolution is not None: self.display_variables["resolution"] = resolution

            if ball_2d_coords:
                self.display_variables["ball_2d_px"] = f"({ball_2d_coords[0]:.0f}, {ball_2d_coords[1]:.0f})"
                if ball_pixel_radius:
                     self.display_variables["ball_2d_px"] += f", r~{ball_pixel_radius:.0f}px"
            else:
                self.display_variables["ball_2d_px"] = "N/A"

            if ball_3d_world_coords: # This is [X,Y,Z_vis_center]
                self.display_variables["ball_3d_m"] = f"X:{ball_3d_world_coords[0]:.2f}, Y:{ball_3d_world_coords[1]:.2f}, Z_vis:{ball_3d_world_coords[2]:.2f}"
                # Send to Open3D thread queue
                if OPEN3D_AVAILABLE and self.o3d_update_queue is not None:
                    try:
                        self.o3d_update_queue.put_nowait(list(ball_3d_world_coords))
                    except queue.Full:
                        logging.debug("O3D update queue full, dropping position update.")
            else:
                self.display_variables["ball_3d_m"] = "N/A"
                # Optionally send a "None" or special value to O3D queue to hide the ball
                # if OPEN3D_AVAILABLE and self.o3d_update_queue is not None:
                # try: self.o3d_update_queue.put_nowait(None) # To hide/move ball out of view
                # except queue.Full: pass


            if camera_intrinsics:
                self.display_variables["cam_intrinsics"] = np.array2string(np.array(camera_intrinsics), precision=2, separator=', ')
            if camera_extrinsics:
                rvec_str = np.array2string(np.array(camera_extrinsics.get('rvec')), precision=3) if camera_extrinsics.get('rvec') is not None else "N/A"
                tvec_str = np.array2string(np.array(camera_extrinsics.get('tvec')), precision=3) if camera_extrinsics.get('tvec') is not None else "N/A"
                self.display_variables["cam_extrinsics"] = f"rvec: {rvec_str}\ntvec: {tvec_str}"

            # Box data is mostly static from Config, but can be updated if dynamic
            if box_corners_world: self.box_corners_world = box_corners_world
            if box_dimensions: self.box_dimensions = box_dimensions


    def run(self):
        """Starts the Flask web server."""
        if not self.app:
            logging.error("Flask app not initialized in WebUIManager.")
            return
        logging.info(f"Starting Flask web server on http://{self.host}:{self.port}")
        try:
            # Use waitress or gunicorn for production instead of Flask's dev server
            # For simplicity here, using Flask's dev server.
            # Note: Flask's dev server is not suitable for production.
            self.app.run(host=self.host, port=self.port, debug=False, use_reloader=False, threaded=True)
        except Exception as e:
            logging.error(f"Failed to start Flask web server: {e}", exc_info=True)
        finally:
            logging.info("Flask web server has stopped.")
            self.stop() # Ensure shutdown event is set if server stops for other reasons

    def stop(self):
        """Signals all components to shut down."""
        logging.info("WebUIManager stop called.")
        if not self.shutdown_event.is_set():
            self.shutdown_event.set()
        # Wait for O3D thread to finish if it was started
        if OPEN3D_AVAILABLE and hasattr(self, 'o3d_thread') and self.o3d_thread.is_alive():
            logging.info("Waiting for Open3D thread to join...")
            self.o3d_thread.join(timeout=5.0)
            if self.o3d_thread.is_alive():
                logging.warning("Open3D thread did not exit cleanly.")
        logging.info("WebUIManager stop completed.")


if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO, format=Config.LOG_FORMAT, datefmt=Config.LOG_DATE_FORMAT)
    if not OPEN3D_AVAILABLE:
        print("Open3D is not installed. This example will only run the web server part without 3D visualization.")
        print("Please install Open3D: pip install open3d")
        # Fallback to not run o3d if not available for the example
        # exit() # Or allow running without o3d for web part test


    ui_manager = WebUIManager(port=Config.WEB_PORT)
    flask_thread = threading.Thread(target=ui_manager.run, daemon=True)
    flask_thread.start()
    logging.info(f"Web UI example started. Open3D window should appear if library is installed.")
    logging.info(f"Access web page at http://localhost:{Config.WEB_PORT}")

    # Simulate data updates from main.py
    try:
        count = 0
        while True:
            if ui_manager.shutdown_event.is_set():
                break
            time.sleep(1) # Update data every second
            count +=1

            # Simulate ball movement
            sim_ball_x = 1.0 + 0.5 * np.sin(count * 0.2) # Moves between 0.5 and 1.5
            sim_ball_y = 1.0 + 0.5 * np.cos(count * 0.3) # Moves between 0.5 and 1.5
            sim_ball_z_vis = Config.VOLLEYBALL_RADIUS_M # Center of sphere for visualization

            # Create a dummy raw frame
            dummy_frame = np.random.randint(0, 255, (240, 320, 3), dtype=np.uint8)
            cv2.putText(dummy_frame, f"Frame {count}", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)

            ui_manager.update_data(
                raw_frame_for_stream=dummy_frame,
                ball_3d_world_coords=[sim_ball_x, sim_ball_y, sim_ball_z_vis],
                status_message=f"Simulating frame {count}",
                fps=Config.CAM_REQUESTED_FPS - (count % 5), # Simulate FPS fluctuation
                resolution=f"{Config.CAM_REQUESTED_WIDTH}x{Config.CAM_REQUESTED_HEIGHT}",
                ball_2d_coords=(int(sim_ball_x*100), int(sim_ball_y*100)), # Dummy 2D
                camera_intrinsics=Config.CAMERA_INTRINSIC_MTX.tolist()
            )
            if count % 10 == 0:
                 logging.info(f"Example: Sent update {count} to UI Manager. Ball at ({sim_ball_x:.2f}, {sim_ball_y:.2f}, {sim_ball_z_vis:.2f})")

    except KeyboardInterrupt:
        logging.info("Keyboard interrupt received in example. Shutting down.")
    finally:
        ui_manager.stop()
        if flask_thread.is_alive():
            flask_thread.join(timeout=2)
        logging.info("Web UI example finished.")