# Code/WebUi.py
# web_ui.py

import threading
import time
import logging
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
            "world_box_corners": Config.WORLD_BOX_CORNERS_M.tolist()
        }
        self.data_lock = threading.Lock()
        self._setup_routes()

    def _setup_routes(self):
        @self.app.route('/')
        def index():
            # MODIFIED: Added OrbitControls.js script and updated Three.js setup
            html_template = """
            <!DOCTYPE html>
            <html>
            <head>
                <title>3D Volleyball Tracker</title>
                <script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
                <!-- ADDED: Import OrbitControls for mouse interaction -->
                <script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js"></script>
                <style>
                    body { margin: 0; font-family: sans-serif; background-color: #f0f0f0; color: #333; display: flex; flex-direction: column; align-items: center; padding: 10px; box-sizing: border-box; }
                    h1, h2 { text-align: center; color: #333; margin-top: 10px; margin-bottom: 15px; }
                    .dashboard-container { width: 100%; max-width: 1400px; display: flex; flex-direction: column; gap: 20px; }
                    .visualizations-row { display: flex; flex-wrap: wrap; gap: 20px; width: 100%; }
                    .visualization-panel { flex: 1; min-width: 400px; background: #fff; padding: 15px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); display: flex; flex-direction: column; align-items: center; }
                    #threejs-canvas { display: block; width: 100%; max-width: 600px; height: 450px; background-color: #222; border: 1px solid #ccc; cursor: grab; }
                    img#video_stream { display: block; width: 100%; max-width: 640px; height: auto; background-color: #222; border: 1px solid #ccc; }
                    .data-section { width: 100%; background: #fff; padding: 20px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); display: flex; flex-wrap: wrap; gap: 20px; justify-content: space-around; box-sizing: border-box; }
                    .data-group { flex: 1; min-width: 280px; }
                    .data-group h2 { text-align: left; margin-top: 0; margin-bottom: 10px; font-size: 1.3em; border-bottom: 1px solid #eee; padding-bottom: 5px; }
                    table { width: 100%; border-collapse: collapse; }
                    th, td { text-align: left; padding: 8px; border-bottom: 1px solid #eee;}
                    td:first-child { font-weight: bold; color: #555; width: 40%;}
                    pre { font-size:0.9em; white-space:pre-wrap; background-color: #f8f8f8; padding:8px; border-radius:4px; border:1px solid #ddd; margin-top: 5px; }
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
                            <h2>3D Visualization (Click & Drag to Rotate)</h2>
                            <canvas id="threejs-canvas"></canvas>
                        </div>
                    </div>
                    <div class="data-section">
                        <!-- Data fields remain the same -->
                    </div>
                </div>

                <script>
                    function updateDataFields() {
                        fetch('/status_json')
                            .then(response => response.json())
                            .then(data => {
                                // This data update logic remains the same
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
                            })
                            .catch(error => console.error('Error fetching status JSON:', error));
                    }
                    setInterval(updateDataFields, 750);
                    
                    document.addEventListener('DOMContentLoaded', () => {
                        updateDataFields();
                         if (typeof initThreeJS === 'function') {
                            const defaultBox = [{{ Config.BOX_WIDTH_M }}, {{ Config.BOX_DEPTH_M }}, {{ Config.BOX_HEIGHT_M }}];
                            const defaultRadius = {{ Config.VOLLEYBALL_RADIUS_M }};
                            const defaultCorners = JSON.parse(JSON.stringify({{ Config.WORLD_BOX_CORNERS_M.tolist() }}));
                            const defaultCamPos = {{ Config.VIS_CAMERA_POSITION_THREEJS | tojson }};
                            const defaultCamLookAt = {{ Config.VIS_CAMERA_LOOKAT_THREEJS | tojson }};
                            initThreeJS(defaultBox, defaultRadius, defaultCorners, defaultCamPos, defaultCamLookAt);
                        }
                    });

                    let scene, camera, renderer, boxMesh, volleyballMesh, controls;
                    let initialBoxConfig;

                    function initThreeJS(boxDims, ballRadius, worldBoxCorners, cameraPosition, cameraLookAt) {
                        if (scene) return; // Prevent re-initialization
                        
                        const canvas = document.getElementById('threejs-canvas');
                        scene = new THREE.Scene();
                        scene.background = new THREE.Color(0x34495e); 

                        const panel = canvas.parentElement;
                        canvas.width = panel.clientWidth - 30;
                        canvas.height = 450;

                        camera = new THREE.PerspectiveCamera(50, canvas.width / canvas.height, 0.1, 1000);
                        
                        if (cameraPosition && cameraPosition.length === 3) {
                            camera.position.set(cameraPosition[0], cameraPosition[1], cameraPosition[2]);
                        } else {
                            camera.position.set(boxDims[0] / 2, -boxDims[1], boxDims[2] * 1.5);
                        }

                        renderer = new THREE.WebGLRenderer({ canvas: canvas, antialias: true });
                        renderer.setSize(canvas.width, canvas.height);

                        // --- ADDED: Initialize OrbitControls ---
                        controls = new THREE.OrbitControls(camera, renderer.domElement);
                        if (cameraLookAt && cameraLookAt.length === 3) {
                            controls.target.set(cameraLookAt[0], cameraLookAt[1], cameraLookAt[2]);
                        } else {
                            controls.target.set(boxDims[0] / 2, boxDims[1] / 2, boxDims[2] / 2);
                        }
                        controls.enableDamping = true; // an animation loop is required when either damping or auto-rotation are enabled
                        controls.dampingFactor = 0.05;
                        controls.screenSpacePanning = false;
                        controls.minDistance = 1;
                        controls.maxDistance = 50;
                        controls.maxPolarAngle = Math.PI; // Allow looking from below
                        // --- END: OrbitControls Initialization ---

                        const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
                        scene.add(ambientLight);
                        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
                        directionalLight.position.set(5, 5, 10);
                        scene.add(directionalLight);

                        const boxGeometry = new THREE.BoxGeometry(boxDims[0], boxDims[1], boxDims[2]);
                        const boxMaterial = new THREE.MeshStandardMaterial({ color: 0x666666, transparent: true, opacity:0.5 });
                        boxMesh = new THREE.Mesh(boxGeometry, boxMaterial);
                        boxMesh.position.set(boxDims[0] / 2, boxDims[1] / 2, boxDims[2] / 2);
                        scene.add(boxMesh);

                        const gridHelper = new THREE.GridHelper(Math.max(boxDims[0], boxDims[1]), 10);
                        gridHelper.position.set(boxDims[0] / 2, boxDims[1] / 2, 0); 
                        gridHelper.rotation.x = Math.PI / 2;
                        scene.add(gridHelper);

                        const cornerMarkerRadius = 0.03;
                        const cornerMarkerMaterial = new THREE.MeshStandardMaterial({ color: 0xff0000 });
                        if (worldBoxCorners) {
                            worldBoxCorners.forEach(corner => {
                                const cornerSphere = new THREE.SphereGeometry(cornerMarkerRadius, 16, 16);
                                const cornerMesh = new THREE.Mesh(cornerSphere, cornerMarkerMaterial);
                                cornerMesh.position.set(corner[0], corner[1], corner[2]);
                                scene.add(cornerMesh);
                            });
                        }

                        const volleyballGeometry = new THREE.SphereGeometry(ballRadius, 32, 32);
                        const volleyballMaterial = new THREE.MeshStandardMaterial({ color: 0xffdd44 });
                        volleyballMesh = new THREE.Mesh(volleyballGeometry, volleyballMaterial);
                        scene.add(volleyballMesh);
                        
                        const axesHelper = new THREE.AxesHelper(1);
                        axesHelper.position.set(0,0,0.005);
                        scene.add(axesHelper);
                        
                        animate();
                    }

                    function updateThreeJSScene(ballPosition) {
                        if (volleyballMesh && ballPosition) {
                            volleyballMesh.position.set(ballPosition.x, ballPosition.y, ballPosition.z);
                        }
                    }

                    function animate() {
                        requestAnimationFrame(animate);
                        // ADDED: Update controls in the animation loop
                        if (controls) controls.update();
                        if (renderer && scene && camera) {
                           renderer.render(scene, camera);
                        }
                    }
                </script>
            </body>
            </html>
            """
            with self.data_lock:
                return render_template_string(html_template, Config=Config)

        # Other routes remain the same
        @self.app.route('/video_feed')
        def video_feed():
            return Response(self._generate_mjpeg_stream(), mimetype='multipart/x-mixed-replace; boundary=frame')

        @self.app.route('/status_json')
        def status_json():
            with self.data_lock:
                return jsonify(self.display_variables)

    # The rest of the WebUi.py file (generate_mjpeg_stream, update_data, run, stop) remains unchanged.
    def _generate_mjpeg_stream(self):
        logging.info("MJPEG stream: Client connected.")
        while not self.shutdown_event.is_set():
            frame_to_encode = None
            with self.data_lock:
                if self.latest_frame_for_stream is not None:
                    frame_to_encode = self.latest_frame_for_stream.copy()

            if frame_to_encode is None:
                ph_height, ph_width = (480, 640)
                frame_to_encode = np.zeros((ph_height, ph_width, 3), dtype=np.uint8)
                cv2.putText(frame_to_encode, "Waiting for annotated frame...", (10, ph_height // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                time.sleep(0.2)
            
            if frame_to_encode is not None:
                try:
                    flag, encoded_image = cv2.imencode('.jpg', frame_to_encode, [cv2.IMWRITE_JPEG_QUALITY, Config.WEB_STREAM_JPEG_QUALITY])
                    if not flag: continue
                    yield (b'--frame\r\n'
                        b'Content-Type: image/jpeg\r\n\r\n' + bytearray(encoded_image) + b'\r\n')
                except Exception as e:
                    logging.info(f"MJPEG stream: Client connection issue ({type(e).__name__}).")
                    break
            time.sleep(1.0 / Config.WEB_STREAM_MAX_FPS)
        logging.info("MJPEG stream: Generator stopped.")

    def update_data(self, frame_for_stream=None,
                    ball_2d_coords=None, ball_pixel_radius=None, ball_3d_world_coords=None,
                    camera_intrinsics=None, camera_extrinsics=None,
                    fps=None, resolution=None, status_message=None):
        with self.data_lock:
            if frame_for_stream is not None: self.latest_frame_for_stream = frame_for_stream
            if status_message is not None: self.display_variables["status"] = status_message
            if fps is not None: self.display_variables["fps"] = fps
            if resolution is not None: self.display_variables["resolution"] = resolution
            self.display_variables["ball_2d_px"] = f"({ball_2d_coords[0]:.0f}, {ball_2d_coords[1]:.0f})" if ball_2d_coords else "N/A"
            self.display_variables["ball_pixel_radius_px"] = f"{ball_pixel_radius:.0f}px" if ball_pixel_radius else "N/A"
            self.display_variables["ball_3d_m"] = f"X:{ball_3d_world_coords[0]:.2f}, Y:{ball_3d_world_coords[1]:.2f}, Z:{ball_3d_world_coords[2]:.2f}" if ball_3d_world_coords else "N/A"
            if camera_intrinsics is not None: self.display_variables["cam_intrinsics"] = np.array2string(np.array(camera_intrinsics), precision=2, separator=', ')
            if camera_extrinsics is not None:
                rvec_str = np.array2string(np.array(camera_extrinsics.get('rvec')), precision=3)
                tvec_str = np.array2string(np.array(camera_extrinsics.get('tvec')), precision=3)
                self.display_variables["cam_extrinsics"] = f"rvec: {rvec_str}\\ntvec: {tvec_str}"

    def run(self):
        logging.info(f"Starting Flask web server on http://{self.host}:{self.port}")
        try:
            self.app.run(host=self.host, port=self.port, debug=False, use_reloader=False, threaded=True)
        except Exception as e:
            logging.error(f"Failed to start Flask web server: {e}", exc_info=True)
        finally:
            self.stop()

    def stop(self):
        logging.info("WebUIManager stop sequence initiated.")
        self.shutdown_event.set()
