# web_ui.py

import threading
import time
import logging
# import queue # No longer directly used by WebUIManager for Open3D
import numpy as np
import cv2 
from flask import Flask, Response, render_template_string, jsonify

import Config

class WebUIManager:
    def __init__(self, host='0.0.0.0', port=Config.WEB_PORT, shutdown_event_ref=None):
        self.host = host
        self.port = port
        self.app = Flask(__name__)
        self.shutdown_event = shutdown_event_ref if shutdown_event_ref else threading.Event()

        self.latest_frame_for_stream = None 
        self.display_variables = {
            "status": "Initializing...",
            "ball_2d_px": "N/A",
            "ball_3d_m": "N/A", 
            "ball_pixel_radius_px": "N/A",
            "fps": 0.0,
            "resolution": "N/A",
            "cam_intrinsics": "N/A",
            "cam_extrinsics": "N/A",
            "box_dimensions": [Config.BOX_WIDTH_M, Config.BOX_DEPTH_M, Config.BOX_HEIGHT_M],
            "volleyball_radius": Config.VOLLEYBALL_RADIUS_M,
            # Add world box corners for JS to use for corner markers
            "world_box_corners": Config.WORLD_BOX_CORNERS_M.tolist() 
        }
        self.data_lock = threading.Lock()
        self._setup_routes()

    def _setup_routes(self):
        @self.app.route('/')
        def index():
            # HTML template with refactored layout
            html_template = """
            <!DOCTYPE html>
            <html>
            <head>
                <title>3D Volleyball Tracker</title>
                <script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
                <style>
                    body { 
                        margin: 0; 
                        font-family: sans-serif; 
                        background-color: #f0f0f0; 
                        color: #333; 
                        display: flex; 
                        flex-direction: column; 
                        align-items: center;
                        padding: 10px; 
                        box-sizing: border-box;
                    }
                    h1, h2 { 
                        text-align: center; 
                        color: #333; 
                        margin-top: 10px;
                        margin-bottom: 15px;
                    }
                    .dashboard-container {
                        width: 100%;
                        max-width: 1400px; 
                        display: flex;
                        flex-direction: column;
                        gap: 20px; 
                    }
                    .visualizations-row {
                        display: flex;
                        flex-wrap: wrap; 
                        gap: 20px; 
                        width: 100%;
                    }
                    .visualization-panel {
                        flex: 1; 
                        min-width: 400px; 
                        background: #fff; 
                        padding: 15px; 
                        border-radius: 8px; 
                        box-shadow: 0 2px 4px rgba(0,0,0,0.1);
                        display: flex;
                        flex-direction: column;
                        align-items: center; 
                    }
                    #threejs-canvas { 
                        display: block; 
                        width: 100%; 
                        max-width: 600px; 
                        height: 450px; 
                        background-color: #222; 
                        border: 1px solid #ccc;
                    }
                    img#video_stream { 
                        display: block; 
                        width: 100%; 
                        max-width: 640px; 
                        height: auto; 
                        background-color: #222; 
                        border: 1px solid #ccc;
                    }
                    .data-section {
                        width: 100%;
                        background: #fff; 
                        padding: 20px; 
                        border-radius: 8px; 
                        box-shadow: 0 2px 4px rgba(0,0,0,0.1);
                        display: flex;
                        flex-wrap: wrap; 
                        gap: 20px; 
                        justify-content: space-around; 
                        box-sizing: border-box;
                    }
                    .data-group {
                        flex: 1; 
                        min-width: 280px; 
                    }
                    .data-group h2 {
                        text-align: left;
                        margin-top: 0;
                        margin-bottom: 10px;
                        font-size: 1.3em;
                        border-bottom: 1px solid #eee;
                        padding-bottom: 5px;
                    }
                    table { width: 100%; border-collapse: collapse; } 
                    th, td { text-align: left; padding: 8px; border-bottom: 1px solid #eee;}
                    td:first-child { font-weight: bold; color: #555; width: 40%;} 
                    pre { 
                        font-size:0.9em; 
                        white-space:pre-wrap; 
                        background-color: #f8f8f8; 
                        padding:8px; 
                        border-radius:4px; 
                        border:1px solid #ddd;
                        margin-top: 5px;
                    }
                </style>
            </head>
            <body>
                <h1>3D Volleyball Tracker Dashboard</h1>
                <div class="dashboard-container">
                    <div class="visualizations-row">
                        <div class="visualization-panel">
                            <h2>Annotated Stream (2D)</h2>
                            <img id="video_stream" src="/video_feed" alt="Loading camera stream...">
                        </div>
                        <div class="visualization-panel">
                            <h2>3D Visualization (Browser/WebGL)</h2>
                            <canvas id="threejs-canvas"></canvas>
                        </div>
                    </div>

                    <div class="data-section">
                        <div class="data-group">
                            <h2>Live Data</h2>
                            <table>
                                <tr><td>Status:</td><td><span id="status">N/A</span></td></tr>
                                <tr><td>FPS:</td><td><span id="fps">N/A</span></td></tr>
                                <tr><td>Resolution:</td><td><span id="resolution">N/A</span></td></tr>
                            </table>
                        </div>
                        <div class="data-group">
                            <h2>Ball Data</h2>
                            <table>
                                <tr><td>2D (px):</td><td><span id="ball_2d_px">N/A</span></td></tr>
                                <tr><td>Radius (px):</td><td><span id="ball_pixel_radius_px">N/A</span></td></tr>
                                <tr><td>3D (m):</td><td><span id="ball_3d_m">N/A</span></td></tr>
                            </table>
                        </div>
                        <div class="data-group">
                            <h2>Camera Parameters</h2>
                            <p><strong>Intrinsics:</strong> <pre id="cam_intrinsics">N/A</pre></p>
                            <p><strong>Extrinsics:</strong> <pre id="cam_extrinsics">N/A</pre></p>
                        </div>
                    </div>
                </div>

                <script>
                    function updateDataFields() {
                        fetch('/status_json')
                            .then(response => response.json())
                            .then(data => {
                                document.getElementById('status').textContent = data.status || 'N/A';
                                document.getElementById('fps').textContent = data.fps !== undefined ? parseFloat(data.fps).toFixed(2) : 'N/A';
                                document.getElementById('resolution').textContent = data.resolution || 'N/A';
                                document.getElementById('ball_2d_px').textContent = data.ball_2d_px || 'N/A';
                                document.getElementById('ball_pixel_radius_px').textContent = data.ball_pixel_radius_px || 'N/A';
                                
                                const ball3dText = data.ball_3d_m || "N/A";
                                document.getElementById('ball_3d_m').textContent = ball3dText;
                                
                                document.getElementById('cam_intrinsics').textContent = data.cam_intrinsics || 'N/A';
                                document.getElementById('cam_extrinsics').textContent = data.cam_extrinsics || 'N/A';

                                if (typeof updateThreeJSScene === 'function' && data.ball_3d_m && data.ball_3d_m !== "N/A") {
                                    try {
                                        const parts = ball3dText.split(',').map(p => parseFloat(p.split(':')[1]));
                                        if (parts.length === 3 && !parts.some(isNaN)) {
                                            updateThreeJSScene({ x: parts[0], y: parts[1], z: parts[2] });
                                        }
                                    } catch (e) { console.error("Error parsing ball_3d_m for Three.js:", e); }
                                }
                                if (typeof initialBoxConfig === 'undefined' && data.box_dimensions && data.volleyball_radius && data.world_box_corners) {
                                     initThreeJS(data.box_dimensions, data.volleyball_radius, data.world_box_corners);
                                }
                            })
                            .catch(error => console.error('Error fetching status JSON:', error));
                    }
                    setInterval(updateDataFields, 750); 
                    document.addEventListener('DOMContentLoaded', () => {
                        updateDataFields(); 
                         if (typeof initThreeJS === 'function') {
                            const defaultBox = [{{ Config.BOX_WIDTH_M }}, {{ Config.BOX_DEPTH_M }}, {{ Config.BOX_HEIGHT_M }}];
                            const defaultRadius = {{ Config.VOLLEYBALL_RADIUS_M }};
                            const defaultCorners = JSON.parse(JSON.stringify({{ Config.WORLD_BOX_CORNERS_M.tolist() }})); // Pass as JS array
                            initThreeJS(defaultBox, defaultRadius, defaultCorners);
                        }
                    });

                    let scene, camera, renderer, boxMesh, volleyballMesh, controls; // Added controls
                    let initialBoxConfig; 

                    function initThreeJS(boxDims, ballRadius, worldBoxCorners) {
                        if (initialBoxConfig && scene) return; 
                        initialBoxConfig = {boxDims, ballRadius, worldBoxCorners};

                        const canvas = document.getElementById('threejs-canvas');
                        
                        scene = new THREE.Scene();
                        scene.background = new THREE.Color(0x34495e); // Darker slate blue

                        const panel = canvas.parentElement;
                        let panelWidth = panel.clientWidth - 30; 
                        let canvasHeight = 400; 
                        canvas.width = panelWidth > 300 ? panelWidth : 300; 
                        canvas.height = canvasHeight;

                        const aspectRatio = canvas.width / canvas.height;
                        camera = new THREE.PerspectiveCamera(50, aspectRatio, 0.1, 1000);
                        
                        // New Camera Position (rotated clockwise, higher angle)
                        // Looking at the center of the box base (boxDims[0]/2, boxDims[1]/2, 0)
                        // Position camera at (X, Y, Z) such that it's rotated clockwise from a top-left view
                        // Let's try a position that is more to the "right" and "front" of the box.
                        const camDist = Math.max(boxDims[0], boxDims[1]) * 2.0; // Distance from center
                        camera.position.set(
                            boxDims[0] / 2 + camDist * 1.707, // X offset (positive X)
                            boxDims[1] / 2 - camDist * 1.7, // Y offset (negative Y for clockwise rotation if Y is depth)
                            camDist * 0.8 // Z height (positive Z)
                        );
                        camera.lookAt(boxDims[0]/2, boxDims[1]/2, boxDims[2]/2); // Look at center of box volume

                        renderer = new THREE.WebGLRenderer({ canvas: canvas, antialias: true });
                        renderer.setSize(canvas.width, canvas.height); 

                        const ambientLight = new THREE.AmbientLight(0xffffff, 0.6); 
                        scene.add(ambientLight);
                        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8); 
                        directionalLight.position.set(boxDims[0], boxDims[1]*1.5, Math.max(boxDims[0],boxDims[1])*2); 
                        directionalLight.castShadow = false; 
                        scene.add(directionalLight);
                        const directionalLight2 = new THREE.DirectionalLight(0xffffff, 0.4); 
                        directionalLight2.position.set(-boxDims[0], -boxDims[1]*1.5, Math.max(boxDims[0],boxDims[1])); 
                        scene.add(directionalLight2);


                        // Box (Ground Plane volume)
                        const boxGeometry = new THREE.BoxGeometry(boxDims[0], boxDims[1], boxDims[2]);
                        const boxMaterial = new THREE.MeshStandardMaterial({ color: 0x666666, transparent: true, opacity:0.5 }); 
                        boxMesh = new THREE.Mesh(boxGeometry, boxMaterial);
                        boxMesh.position.set(boxDims[0] / 2, boxDims[1] / 2, boxDims[2] / 2); 
                        scene.add(boxMesh);

                        // Ground Grid Helper
                        const gridDivisions = Math.max(Math.round(boxDims[0]), Math.round(boxDims[1])); // e.g., 1 division per meter
                        const gridHelper = new THREE.GridHelper(Math.max(boxDims[0], boxDims[1]), gridDivisions, 0x888888, 0x555555);
                        gridHelper.position.set(boxDims[0] / 2, boxDims[1] / 2, 0); // Position at center of box base
                        gridHelper.rotation.x = Math.PI / 2; // Rotate to be on XY plane
                        scene.add(gridHelper);

                        // Corner Markers (at Z=0 for ground corners)
                        const cornerMarkerRadius = 0.03;
                        const cornerMarkerMaterial = new THREE.MeshStandardMaterial({ color: 0xff0000 }); // Red markers
                        if (worldBoxCorners && worldBoxCorners.length >= 4) {
                            worldBoxCorners.forEach(corner => {
                                const cornerSphere = new THREE.SphereGeometry(cornerMarkerRadius, 16, 16);
                                const cornerMesh = new THREE.Mesh(cornerSphere, cornerMarkerMaterial);
                                cornerMesh.position.set(corner[0], corner[1], corner[2] + cornerMarkerRadius); // Place slightly above ground for visibility
                                scene.add(cornerMesh);
                            });
                        }


                        const volleyballGeometry = new THREE.SphereGeometry(ballRadius, 32, 32);
                        const volleyballMaterial = new THREE.MeshStandardMaterial({ color: 0xffdd44, emissive: 0x111100 }); 
                        volleyballMesh = new THREE.Mesh(volleyballGeometry, volleyballMaterial);
                        // Initial position will be updated by fetched data, place it at center of box volume, raised by its radius
                        volleyballMesh.position.set(boxDims[0] / 2, boxDims[1] / 2, ballRadius + boxDims[2] / 2 ); 
                        scene.add(volleyballMesh);
                        
                        const axesHelper = new THREE.AxesHelper(Math.max(boxDims[0], boxDims[1]) * 0.5 ); 
                        axesHelper.position.set(0,0,0.005); 
                        scene.add(axesHelper);
                        
                        animate();

                        window.addEventListener('resize', onWindowResize, false);
                        function onWindowResize() {
                            let newPanelWidth = panel.clientWidth - 30;
                            let newCanvasWidth = newPanelWidth > 300 ? newPanelWidth : 300;
                            // Maintain aspect ratio based on original canvas.height or set a new one
                            let newCanvasHeight = (newCanvasWidth / aspectRatio_original) || canvasHeight; 
                            
                            // If you want fixed height and variable width:
                            // canvas.width = newCanvasWidth;
                            // canvas.height = canvasHeight; // Keep original height
                            
                            // Or if you want to scale proportionally to new width (might change height):
                            canvas.width = newCanvasWidth;
                            canvas.height = newCanvasWidth / aspectRatio;


                            camera.aspect = canvas.width / canvas.height;
                            camera.updateProjectionMatrix();
                            renderer.setSize(canvas.width, canvas.height);
                        }
                        const aspectRatio_original = canvas.width / canvas.height; // Store original for resize
                        // onWindowResize(); // Call once to set initial size (already sized by panel width)
                    }

                    function updateThreeJSScene(ballPosition) {
                        if (volleyballMesh && ballPosition) {
                            volleyballMesh.position.set(ballPosition.x, ballPosition.y, ballPosition.z);
                        }
                    }

                    function animate() {
                        requestAnimationFrame(animate);
                        // if (controls) controls.update(); 
                        if (renderer && scene && camera) {
                           renderer.render(scene, camera);
                        }
                    }
                    
                    const streamImg = document.getElementById('video_stream');
                    if (streamImg) {
                        streamImg.onerror = function() {
                            this.alt = 'Camera stream failed to load or is unavailable.';
                            this.src = ''; 
                            console.error('Error loading video stream.');
                        };
                    }
                </script>
            </body>
            </html>
            """
            with self.data_lock:
                return render_template_string(html_template, Config=Config)

        @self.app.route('/video_feed')
        def video_feed():
            return Response(self._generate_mjpeg_stream(),
                            mimetype='multipart/x-mixed-replace; boundary=frame')

        @self.app.route('/status_json')
        def status_json():
            with self.data_lock:
                # Ensure world_box_corners is part of the JSON response for initThreeJS
                response_data = self.display_variables.copy()
                if "world_box_corners" not in response_data: # Should be set in __init__
                    response_data["world_box_corners"] = Config.WORLD_BOX_CORNERS_M.tolist()
                return jsonify(response_data)

    def _generate_mjpeg_stream(self):
        logging.info("MJPEG stream: Client connected.")
        while not self.shutdown_event.is_set():
            frame_to_encode = None
            with self.data_lock:
                if self.latest_frame_for_stream is not None: 
                    frame_to_encode = self.latest_frame_for_stream.copy()

            if frame_to_encode is None:
                ph_height = Config.CAM_REQUESTED_HEIGHT // 2 if hasattr(Config, 'CAM_REQUESTED_HEIGHT') and Config.CAM_REQUESTED_HEIGHT else 240
                ph_width = Config.CAM_REQUESTED_WIDTH // 2 if hasattr(Config, 'CAM_REQUESTED_WIDTH') and Config.CAM_REQUESTED_WIDTH else 320
                frame_to_encode = np.zeros((ph_height, ph_width, 3), dtype=np.uint8)
                cv2.putText(frame_to_encode, "Waiting for annotated frame...", (10, ph_height // 2 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                cv2.putText(frame_to_encode, "(Display from main.py)", (10, ph_height // 2 + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                time.sleep(0.2)
            
            if frame_to_encode is not None: 
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
                    logging.info("MJPEG stream: Client disconnected (GeneratorExit).")
                    break
                except (ConnectionResetError, BrokenPipeError) as e:
                    logging.info(f"MJPEG stream: Client connection issue ({type(e).__name__}).")
                    break
                except Exception as e:
                    logging.error(f"MJPEG stream: Error: {e}", exc_info=True)
                    break
            
            max_fps_default = Config.CAM_REQUESTED_FPS if hasattr(Config, 'CAM_REQUESTED_FPS') and Config.CAM_REQUESTED_FPS > 0 else 15.0
            max_fps = Config.WEB_STREAM_MAX_FPS if hasattr(Config, 'WEB_STREAM_MAX_FPS') and Config.WEB_STREAM_MAX_FPS > 0 else max_fps_default
            sleep_duration = 1.0 / max_fps
            time.sleep(max(0.001, sleep_duration))
        logging.info("MJPEG stream: Generator stopped.")


    def update_data(self, frame_for_stream=None, 
                    ball_2d_coords=None, ball_pixel_radius=None, ball_3d_world_coords=None, 
                    box_corners_world=None, box_dimensions=None, 
                    camera_intrinsics=None, camera_extrinsics=None,
                    fps=None, resolution=None, status_message=None):
        with self.data_lock:
            if frame_for_stream is not None: 
                self.latest_frame_for_stream = frame_for_stream 
            
            if status_message is not None: self.display_variables["status"] = status_message
            if fps is not None: self.display_variables["fps"] = fps
            if resolution is not None: self.display_variables["resolution"] = resolution

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
            
            self.display_variables["box_dimensions"] = Config.BOX_WIDTH_M, Config.BOX_DEPTH_M, Config.BOX_HEIGHT_M
            self.display_variables["volleyball_radius"] = Config.VOLLEYBALL_RADIUS_M
            self.display_variables["world_box_corners"] = Config.WORLD_BOX_CORNERS_M.tolist()


    def run(self):
        if not self.app:
            logging.error("Flask app not initialized in WebUIManager.")
            return
        logging.info(f"Starting Flask web server on http://{self.host}:{self.port}")
        try:
            self.app.run(host=self.host, port=self.port, debug=False, use_reloader=False, threaded=True)
        except SystemExit:
            logging.info("Flask server SystemExit (likely controlled shutdown).")
        except Exception as e:
            logging.error(f"Failed to start Flask web server: {e}", exc_info=True)
        finally:
            logging.info("Flask web server has stopped.")
            self.stop()

    def stop(self):
        logging.info("WebUIManager stop sequence initiated.")
        if not self.shutdown_event.is_set():
            self.shutdown_event.set() 
        logging.info("WebUIManager stop sequence completed.")


if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG, format=Config.LOG_FORMAT, datefmt=Config.LOG_DATE_FORMAT)
    example_shutdown_event = threading.Event()
    
    if not hasattr(Config, 'VIS_CAMERA_FRONT'): Config.VIS_CAMERA_FRONT = [-0.7, -0.7, 1.5] 
    if not hasattr(Config, 'BOX_WIDTH_M'): Config.BOX_WIDTH_M = 1.0
    if not hasattr(Config, 'BOX_DEPTH_M'): Config.BOX_DEPTH_M = 1.0
    if not hasattr(Config, 'BOX_HEIGHT_M'): Config.BOX_HEIGHT_M = 0.05
    if not hasattr(Config, 'VOLLEYBALL_RADIUS_M'): Config.VOLLEYBALL_RADIUS_M = 0.105
    if not hasattr(Config, 'CAM_REQUESTED_FPS'): Config.CAM_REQUESTED_FPS = 30.0
    if not hasattr(Config, 'WEB_STREAM_MAX_FPS'): Config.WEB_STREAM_MAX_FPS = 15.0
    if not hasattr(Config, 'WORLD_BOX_CORNERS_M'): # Add dummy if missing for standalone test
        Config.WORLD_BOX_CORNERS_M = np.array([[0,0,0],[1,0,0],[1,1,0],[0,1,0]], dtype=float)


    ui_manager = WebUIManager(port=Config.WEB_PORT if hasattr(Config, 'WEB_PORT') else 8000,
                              shutdown_event_ref=example_shutdown_event)
    
    flask_thread = threading.Thread(target=ui_manager.run, name="TestFlaskThread", daemon=True)
    flask_thread.start()
    time.sleep(1.5)
    if not flask_thread.is_alive():
        logging.error("Failed to start Flask thread. Exiting.")
        exit()

    logging.info(f"Web UI example with Three.js started. Access web page at http://localhost:{ui_manager.port}")

    try:
        count = 0
        for _ in range(120): 
            if example_shutdown_event.is_set(): break
            time.sleep(1)
            count +=1

            sim_ball_x = Config.BOX_WIDTH_M / 2 + (Config.BOX_WIDTH_M/3) * np.sin(count * 0.1) 
            sim_ball_y = Config.BOX_DEPTH_M / 2 + (Config.BOX_DEPTH_M/3) * np.cos(count * 0.15) 
            sim_ball_z_vis = Config.VOLLEYBALL_RADIUS_M + Config.BOX_HEIGHT_M 

            annotated_frame_sim = np.zeros((240, 320, 3), dtype=np.uint8)
            cv2.putText(annotated_frame_sim, f"Annotated Frame {count}", (10,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
            cv2.circle(annotated_frame_sim, (int(160 + sim_ball_x*20), int(120 + sim_ball_y*20)), 10, (0,255,0), -1) 
            cv2.putText(annotated_frame_sim, f"X:{sim_ball_x:.1f} Y:{sim_ball_y:.1f}", (10, 220), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,0),1)

            ui_manager.update_data(
                frame_for_stream=annotated_frame_sim, 
                ball_3d_world_coords=[sim_ball_x, sim_ball_y, sim_ball_z_vis],
                status_message=f"Simulating frame {count}",
                fps= Config.CAM_REQUESTED_FPS - (count % 5), 
                resolution="320x240 (sim)",
                ball_2d_coords=(int(160+sim_ball_x*20), int(120+sim_ball_y*20)), 
                ball_pixel_radius= 10,
                camera_intrinsics=np.eye(3).tolist(),
                camera_extrinsics={'rvec': [0.0,0.0,0.0], 'tvec': [0.0,0.0,1.0]}
            )
            if count % 10 == 0: logging.debug(f"Example: Sent update {count}.")
        logging.info("Example: Simulated run finished.")
    except KeyboardInterrupt:
        logging.info("Keyboard interrupt. Shutting down.")
    finally:
        if not example_shutdown_event.is_set(): example_shutdown_event.set()
        if flask_thread.is_alive(): 
            logging.info("Waiting for Flask thread to complete...")
            flask_thread.join(timeout=2.0)
        logging.info("Web UI example completed.")