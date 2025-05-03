# -*- coding: utf-8 -*-
"""
web_ui.py

Handles the Flask web server and user interface for the Pi Camera application.
Provides routes for streaming, status updates, and control commands.
Refactored for single-camera operation. Includes servo control UI.
Replaced AWB dropdown with ISO control.

**Modification:** Fixed AttributeError by using correct attribute name
                  `current_resolution_index0` and config constant `CAM0_RESOLUTIONS`.
**Modification 2 (User Request):** Added live FPS counter display.
**Modification 3 (User Request):** Added a second video feed frame for Computer Vision output.
**Modification 4 (User Request):** Removed references to audio errors.
"""

import logging
import time
import cv2
import threading # Import threading for lock
import numpy as np # Needed for dummy frame generation
from flask import Flask, Response, render_template_string, jsonify, request, url_for
from libcamera import controls # Needed for control validation

# Import configuration and managers (assuming they are in the same directory)
import config
# We don't directly import CameraManager/HardwareManager here,
# instances will be passed to the setup function.

# Attempt to import CV functions - will be created later (Req 5)
try:
    import cv_functions
    cv_available = True
except ImportError:
    logging.warning("web_ui: cv_functions.py not found. CV output stream will show placeholder.")
    cv_available = False

# Global Flask app instance
app = Flask(__name__)

# --- Global variables to hold manager instances and shared state ---
# These will be set by the setup_web_app function
_camera_manager = None
_hardware_manager = None
_app_state = {
    "digital_recording_active": False,
    "reconfigure_resolution_index": None, # Tracks requests from UI
    "reboot_requested": False, # Flag set by /power_down
    "shutdown_event": None # threading.Event passed from main
}
_ui_lock = threading.Lock() # Lock for accessing shared _app_state variables

# ===========================================================
# === HTML Template ===
# Added FPS counter span, second image tag for CV output
# ===========================================================
HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Pi Camera Stream & Record</title>
    {# Raw block to prevent Jinja processing CSS/JS #}
    {% raw %}
    <style>
        body { font-family: sans-serif; line-height: 1.4; margin: 1em; background-color: #f0f0f0;}
        .container { max-width: 960px; margin: auto; background: #fff; padding: 15px; border-radius: 8px; box-shadow: 0 2px 5px rgba(0,0,0,0.1); }
        h1 { text-align: center; color: #333; margin-bottom: 10px; }

        .grid-container { display: grid; grid-template-columns: repeat(auto-fit, minmax(280px, 1fr)); gap: 20px; margin-bottom: 15px; }

        /* Panel Styles */
        .status-panel, .controls-panel, .sliders-panel, .servo-panel { background-color: #eef; padding: 15px; border-radius: 5px; }
        .panel-title { font-weight: bold; margin-bottom: 10px; border-bottom: 1px solid #ccc; padding-bottom: 5px; }

        /* Status Grid */
        .status-grid { display: grid; grid-template-columns: auto 1fr; gap: 5px 10px; align-items: center; }
        .status-grid span:first-child { font-weight: bold; color: #555; text-align: right;}
        #status, #rec-status, #resolution, #battery-level, #measured-fps, /* Added measured-fps */
        #iso-mode-status, #ae-mode-status, #metering-mode-status, #nr-mode-status, /* Status IDs */
        #servo-angle-status
         { color: #0056b3; font-weight: normal;}
        #rec-status.active { color: #D83B01; font-weight: bold;}

        /* Main Controls */
        .main-controls { display: flex; justify-content: center; align-items: center; flex-wrap: wrap; gap: 10px; margin-bottom: 10px; }
        .main-controls button { padding: 10px 20px; margin: 5px; font-size: 1em; cursor: pointer; border-radius: 5px; border: 1px solid #ccc; background-color: #e9e9e9; transition: background-color 0.2s, border-color 0.2s; }
        .main-controls button:hover:not(:disabled) { background-color: #dcdcdc; border-color: #bbb; }

         /* Mode Select Controls */
        .mode-controls { display: grid; grid-template-columns: auto 1fr; gap: 8px 10px; align-items: center; }
        .mode-controls label { font-weight: normal; color: #444; text-align: right; font-size: 0.9em;}
        .mode-controls select { padding: 5px 8px; font-size: 0.9em; border-radius: 4px; border: 1px solid #ccc; width: 100%; box-sizing: border-box; }
        .mode-controls select:hover:not(:disabled) { border-color: #bbb; background-color: #f9f9f9; }

        /* Slider Controls (Image & Servo) */
        .slider-controls { display: grid; grid-template-columns: auto 1fr auto; gap: 5px 10px; align-items: center; margin-bottom: 8px; }
        .slider-controls label { font-weight: normal; color: #444; text-align: right; font-size: 0.9em;}
        .slider-controls input[type=range] { width: 100%; margin: 0; padding: 0; cursor: pointer; }
        .slider-controls span { font-size: 0.9em; color: #0056b3; min-width: 35px; text-align: right; } /* For slider value display */

        #error { color: red; margin-top: 15px; white-space: pre-wrap; font-weight: bold; min-height: 1.2em; text-align: center; background-color: #ffebeb; border: 1px solid red; padding: 8px; border-radius: 4px; display: none; /* Initially hidden */ }

        /* Video Stream Images */
        .video-container { display: flex; flex-direction: column; align-items: center; gap: 15px; margin-top: 15px; }
        img#stream, img#cv_stream { /* Style both streams */
            display: block;
            border: 1px solid black;
            max-width: 100%;
            height: auto;
            background-color: #ddd; /* Placeholder color */
        }

        /* Button Specific Styles */
        button#btn-record.recording-active { background-color: #ff4d4d; color: white; border-color: #ff1a1a; }
        button#btn-record.recording-active:hover:not(:disabled) { background-color: #e60000; }
        button#btn-record.recording-inactive { background-color: #4CAF50; color: white; border-color: #367c39;}
        button#btn-record.recording-inactive:hover:not(:disabled) { background-color: #45a049; }
        button#btn-powerdown { background-color: #f44336; color: white; border-color: #d32f2f;}
        button#btn-powerdown:hover:not(:disabled) { background-color: #c62828; }
        button:disabled, select:disabled, input[type=range]:disabled { background-color: #cccccc !important; cursor: not-allowed !important; border-color: #999 !important; color: #666 !important; opacity: 0.7; }
        input[type=range]:disabled::-webkit-slider-thumb { background: #999; }
        input[type=range]:disabled::-moz-range-thumb { background: #999; }
    </style>
    {% endraw %}
</head>
<body>
    <div class="container">
        <h1>Pi Camera Stream & Record</h1>

         <div class="main-controls">
             <button onclick="changeResolution('down')" id="btn-down" title="Decrease resolution">&laquo; Lower Res</button>
             <button onclick="toggleRecording()" id="btn-record" class="recording-inactive" title="Toggle recording via web interface">Start Rec (Web)</button>
             <button onclick="changeResolution('up')" id="btn-up" title="Increase resolution">Higher Res &raquo;</button>
             <button onclick="powerDown()" id="btn-powerdown" title="Gracefully stop service and reboot Pi">Power Down</button>
         </div>

         <div class="grid-container">
             <div class="status-panel">
                 <div class="panel-title">Current Status</div>
                 <div class="status-grid">
                     <span>Sys Status:</span> <span id="status">Initializing...</span>
                     <span>Recording:</span> <span id="rec-status">OFF</span>
                     <span>Resolution:</span> <span id="resolution">{{ resolution_text }}</span>
                     <span>Measured FPS:</span> <span id="measured-fps">--</span> {# Added FPS display #}
                     <span>Battery:</span> <span id="battery-level">{{ batt_text_initial }}%</span>
                     {% if servo_enabled %}
                     <span>Servo Angle:</span> <span id="servo-angle-status">{{ servo_angle_initial }}&deg;</span>
                     {% endif %}
                     <hr style="grid-column: 1 / -1; border-top: 1px dashed #bbb; border-bottom: none; margin: 5px 0;">
                     <span>ISO Level:</span> <span id="iso-mode-status">{{ current_iso_name_initial }}</span>
                     <span>AE Mode:</span> <span id="ae-mode-status">{{ current_ae_mode_name_initial }}</span>
                     <span>Metering:</span> <span id="metering-mode-status">{{ current_metering_mode_name_initial }}</span>
                     <span>Noise Red.:</span> <span id="nr-mode-status">{{ current_noise_reduction_mode_name_initial }}</span>
                 </div>
             </div>

             <div class="controls-panel">
                 <div class="panel-title">Mode Controls</div>
                 <div class="mode-controls">
                     <label for="iso-select">ISO Level:</label>
                     <select id="iso-select" onchange="changeCameraControl('ISOSetting', this.value)" title="Select ISO Level">
                         {{ iso_options_html | safe }} {# Use safe filter for HTML options #}
                     </select>
                     <label for="ae-select">Exposure Mode:</label>
                     <select id="ae-select" onchange="changeCameraControl('AeExposureMode', this.value)" title="Select Auto Exposure Mode">
                         {{ ae_options_html | safe }}
                     </select>
                     <label for="metering-select">Metering Mode:</label>
                     <select id="metering-select" onchange="changeCameraControl('AeMeteringMode', this.value)" title="Select AE Metering Mode">
                         {{ metering_options_html | safe }}
                     </select>
                     <label for="nr-select">Noise Reduction:</label>
                     <select id="nr-select" onchange="changeCameraControl('NoiseReductionMode', this.value)" title="Select Noise Reduction Mode">
                         {{ noise_reduction_options_html | safe }}
                     </select>
                 </div>
             </div>

             <div class="sliders-panel">
                 <div class="panel-title">Image Adjustments</div>
                 <div class="slider-controls">
                     <label for="brightness-slider">Brightness:</label>
                     <input type="range" id="brightness-slider" min="{{ MIN_BRIGHTNESS }}" max="{{ MAX_BRIGHTNESS }}" step="{{ STEP_BRIGHTNESS }}" value="{{ brightness_initial }}" oninput="updateSliderValue(this.id, this.value, 1)" onchange="changeCameraControl('Brightness', this.value)" title="Adjust Brightness">
                     <span id="brightness-slider-value">{{ "%.1f" | format(brightness_initial) }}</span>
                     <label for="contrast-slider">Contrast:</label>
                     <input type="range" id="contrast-slider" min="{{ MIN_CONTRAST }}" max="{{ MAX_CONTRAST }}" step="{{ STEP_CONTRAST }}" value="{{ contrast_initial }}" oninput="updateSliderValue(this.id, this.value, 1)" onchange="changeCameraControl('Contrast', this.value)" title="Adjust Contrast">
                     <span id="contrast-slider-value">{{ "%.1f" | format(contrast_initial) }}</span>
                     <label for="saturation-slider">Saturation:</label>
                     <input type="range" id="saturation-slider" min="{{ MIN_SATURATION }}" max="{{ MAX_SATURATION }}" step="{{ STEP_SATURATION }}" value="{{ saturation_initial }}" oninput="updateSliderValue(this.id, this.value, 1)" onchange="changeCameraControl('Saturation', this.value)" title="Adjust Saturation">
                     <span id="saturation-slider-value">{{ "%.1f" | format(saturation_initial) }}</span>
                     <label for="sharpness-slider">Sharpness:</label>
                     <input type="range" id="sharpness-slider" min="{{ MIN_SHARPNESS }}" max="{{ MAX_SHARPNESS }}" step="{{ STEP_SHARPNESS }}" value="{{ sharpness_initial }}" oninput="updateSliderValue(this.id, this.value, 1)" onchange="changeCameraControl('Sharpness', this.value)" title="Adjust Sharpness">
                     <span id="sharpness-slider-value">{{ "%.1f" | format(sharpness_initial) }}</span>
                 </div>
             </div>

             {% if servo_enabled %}
             <div class="servo-panel">
                 <div class="panel-title">Servo Control</div>
                 <div class="slider-controls">
                     <label for="servo-slider">Angle:</label>
                     <input type="range" id="servo-slider" min="{{ SERVO_MIN_ANGLE }}" max="{{ SERVO_MAX_ANGLE }}" step="1" value="{{ servo_angle_initial }}" oninput="updateSliderValue(this.id, this.value, 0)" onchange="setServoAngle(this.value)" title="Adjust Servo Angle">
                     <span id="servo-slider-value">{{ servo_angle_initial }}&deg;</span>
                 </div>
             </div>
             {% endif %}

         </div>

         <div id="error" {% if err_msg %}style="display: block;"{% endif %}>{{ err_msg }}</div>

         {# Container for the video streams #}
         <div class="video-container">
             {# Main Camera Stream #}
             <img id="stream" src="{{ video_feed_url }}" width="{{ current_w }}" height="{{ current_h }}" alt="Loading main stream..."
                  onerror="handleStreamError()" onload="handleStreamLoad()">

             {# Computer Vision Output Stream #}
             <img id="cv_stream" src="{{ cv_video_feed_url }}" width="{{ current_w }}" height="{{ current_h }}" alt="Loading CV stream..."
                  onerror="handleCvStreamError()" onload="handleCvStreamLoad()">
         </div>

    </div>

    <script>
        // Use Jinja var for initial state:
        let currentDigitalRecordState = {{ 'true' if digital_rec_state_initial else 'false' }};
        // Store base URLs generated by Python/Flask
        const videoFeedUrlBase = "{{ video_feed_url }}";
        const cvVideoFeedUrlBase = "{{ cv_video_feed_url }}";

        // Get Element References
        const statusElement = document.getElementById('status');
        const resolutionElement = document.getElementById('resolution');
        const measuredFpsElement = document.getElementById('measured-fps'); // Added FPS element
        const errorElement = document.getElementById('error');
        const streamImage = document.getElementById('stream');
        const cvStreamImage = document.getElementById('cv_stream'); // Added CV stream element
        const btnUp = document.getElementById('btn-up');
        const btnDown = document.getElementById('btn-down');
        const btnRecord = document.getElementById('btn-record');
        const btnPowerdown = document.getElementById('btn-powerdown');
        const recStatusElement = document.getElementById('rec-status');
        const batteryLevelElement = document.getElementById('battery-level');
        // Camera Control Elements
        const isoModeStatusElement = document.getElementById('iso-mode-status');
        const aeStatusElement = document.getElementById('ae-mode-status');
        const meteringStatusElement = document.getElementById('metering-mode-status');
        const nrStatusElement = document.getElementById('nr-mode-status');
        const isoSelectElement = document.getElementById('iso-select');
        const aeSelectElement = document.getElementById('ae-select');
        const meteringSelectElement = document.getElementById('metering-select');
        const nrSelectElement = document.getElementById('nr-select');
        // Slider Elements
        const brightnessSlider = document.getElementById('brightness-slider');
        const contrastSlider = document.getElementById('contrast-slider');
        const saturationSlider = document.getElementById('saturation-slider');
        const sharpnessSlider = document.getElementById('sharpness-slider');
        const brightnessValueSpan = document.getElementById('brightness-slider-value');
        const contrastValueSpan = document.getElementById('contrast-slider-value');
        const saturationValueSpan = document.getElementById('saturation-slider-value');
        const sharpnessValueSpan = document.getElementById('sharpness-slider-value');
        // Servo elements (check if they exist)
        const servoSlider = document.getElementById('servo-slider');
        const servoSliderValueSpan = document.getElementById('servo-slider-value');
        const servoAngleStatus = document.getElementById('servo-angle-status');


        // State Variables
        let isChangingResolution = false;
        let isTogglingRecording = false;
        let isChangingControl = false;
        let isSettingServo = false;
        let isPoweringDown = false;
        let statusUpdateInterval;
        let streamErrorTimeout = null;
        let cvStreamErrorTimeout = null; // Added for CV stream

        // --- UI Update Functions ---
        function updateRecordButtonState() {
            // ... (unchanged) ...
            if (currentDigitalRecordState) {
                btnRecord.textContent = "Stop Rec (Web)";
                btnRecord.classList.remove('recording-inactive');
                btnRecord.classList.add('recording-active');
            } else {
                btnRecord.textContent = "Start Rec (Web)";
                btnRecord.classList.add('recording-inactive');
                btnRecord.classList.remove('recording-active');
            }
        }

        function updateStatus() {
            // Prevent status updates during critical operations
            if (isChangingResolution || isTogglingRecording || isChangingControl || isSettingServo || isPoweringDown) return;

            fetch('/status')
                .then(response => { if (!response.ok) { throw new Error(`HTTP error! Status: ${response.status}`); } return response.json(); })
                .then(data => {
                    // Update general status
                    statusElement.textContent = data.status_text || 'Unknown';
                    recStatusElement.textContent = data.is_recording ? "ACTIVE" : "OFF";
                    recStatusElement.classList.toggle('active', data.is_recording);

                    // Update measured FPS (Requirement 3)
                    if (data.measured_fps !== null && data.measured_fps !== undefined) {
                        measuredFpsElement.textContent = data.measured_fps.toFixed(1);
                    } else {
                        measuredFpsElement.textContent = "--";
                    }

                    // Update resolution display and image dimensions if changed
                    if (data.resolution && resolutionElement.textContent !== data.resolution) {
                        resolutionElement.textContent = data.resolution;
                        const [w, h] = data.resolution.split('x');
                        // Update dimensions for both stream images
                        if (streamImage && (streamImage.getAttribute('width') != w || streamImage.getAttribute('height') != h)) {
                            console.log(`Updating main stream image dimensions to ${w}x${h}`);
                            streamImage.setAttribute('width', w);
                            streamImage.setAttribute('height', h);
                        }
                        if (cvStreamImage && (cvStreamImage.getAttribute('width') != w || cvStreamImage.getAttribute('height') != h)) {
                            console.log(`Updating CV stream image dimensions to ${w}x${h}`);
                            cvStreamImage.setAttribute('width', w);
                            cvStreamImage.setAttribute('height', h);
                        }
                    }

                    // Display/clear error messages
                    if (data.error) {
                        errorElement.textContent = data.error;
                        errorElement.style.display = 'block';
                    } else {
                        if (errorElement.style.display !== 'none') {
                            errorElement.textContent = '';
                            errorElement.style.display = 'none';
                        }
                    }

                    // Update digital recording button state
                    if (typeof data.digital_recording_active === 'boolean' && currentDigitalRecordState !== data.digital_recording_active) {
                        currentDigitalRecordState = data.digital_recording_active;
                        updateRecordButtonState();
                    }

                    // Update camera control UI elements (dropdowns and sliders)
                    updateControlUI('iso_mode', data.iso_mode, isoModeStatusElement, isoSelectElement);
                    updateControlUI('ae_mode', data.ae_mode, aeStatusElement, aeSelectElement);
                    updateControlUI('metering_mode', data.metering_mode, meteringStatusElement, meteringSelectElement);
                    updateControlUI('noise_reduction_mode', data.noise_reduction_mode, nrStatusElement, nrSelectElement);
                    updateControlUI('brightness', data.brightness, null, brightnessSlider, brightnessValueSpan, 1);
                    updateControlUI('contrast', data.contrast, null, contrastSlider, contrastValueSpan, 1);
                    updateControlUI('saturation', data.saturation, null, saturationSlider, saturationValueSpan, 1);
                    updateControlUI('sharpness', data.sharpness, null, sharpnessSlider, sharpnessValueSpan, 1);

                    // Update servo UI elements if they exist
                    if (servoSlider) {
                        updateControlUI('servo_angle', data.servo_angle, servoAngleStatus, servoSlider, servoSliderValueSpan, 0);
                    }

                    // Update battery level display
                    if (data.battery_percent !== null && data.battery_percent !== undefined) {
                         batteryLevelElement.textContent = data.battery_percent.toFixed(1) + '%';
                    } else {
                         batteryLevelElement.textContent = "--%";
                    }
                })
                .catch(err => {
                    // Handle errors during status fetch
                    console.error("Error fetching status:", err);
                    statusElement.textContent = "Error";
                    errorElement.textContent = `Status fetch failed: ${err.message}.`;
                    errorElement.style.display = 'block';
                    // Set related UI elements to error state
                    recStatusElement.textContent = "Err";
                    measuredFpsElement.textContent = "Err"; // Added
                    batteryLevelElement.textContent = "Err";
                    isoModeStatusElement.textContent = "Err";
                    aeStatusElement.textContent = "Err";
                    meteringStatusElement.textContent = "Err";
                    nrStatusElement.textContent = "Err";
                    if (servoAngleStatus) servoAngleStatus.textContent = "Err";
                });
        }

        // Helper function to update individual control UI elements
        function updateControlUI(controlKey, newValue, statusEl, controlEl, valueSpanEl = null, decimalPlaces = 1) {
            // ... (unchanged, handles suffix correctly now) ...
             if (newValue === undefined || newValue === null) return;

             let currentUIValue = controlEl ? controlEl.value : null;
             let formattedNewValue = newValue;
             let suffix = (controlKey === 'servo_angle') ? '°' : ''; // Add degree symbol for servo

             if (valueSpanEl) {
                 formattedNewValue = parseFloat(newValue).toFixed(decimalPlaces);
                 currentUIValue = controlEl ? parseFloat(controlEl.value).toFixed(decimalPlaces) : null; // Get current slider value
                 if (valueSpanEl.textContent !== formattedNewValue + suffix) {
                     valueSpanEl.textContent = formattedNewValue + suffix;
                 }
             } else if (statusEl) {
                 if (statusEl.textContent !== newValue.toString() + suffix) {
                     statusEl.textContent = newValue.toString() + suffix;
                 }
             }

             if (controlEl && currentUIValue !== formattedNewValue.toString() && !isChangingControl && !isChangingResolution && !isSettingServo) {
                 // console.log(`Status update forcing UI for ${controlKey}: UI='${currentUIValue}' -> Status='${formattedNewValue}'`);
                 controlEl.value = newValue; // Use the original newValue for setting element value
             }
        }


        // Function to disable all controls during operations
        function disableControls(poweringDown = false) {
            // ... (unchanged) ...
             const elementsToDisable = [
                btnUp, btnDown, btnRecord, btnPowerdown,
                isoSelectElement, aeSelectElement, meteringSelectElement, nrSelectElement,
                brightnessSlider, contrastSlider, saturationSlider, sharpnessSlider
            ];
            if (servoSlider) {
                elementsToDisable.push(servoSlider);
            }
            elementsToDisable.forEach(el => { if(el) el.disabled = true; });
            if(poweringDown) { document.body.style.opacity = '0.7'; }
        }

        // Function to re-enable controls
        function enableControls() {
            // ... (unchanged) ...
            if (!isPoweringDown) {
                 const elementsToEnable = [
                    btnUp, btnDown, btnRecord, btnPowerdown,
                    isoSelectElement, aeSelectElement, meteringSelectElement, nrSelectElement,
                    brightnessSlider, contrastSlider, saturationSlider, sharpnessSlider
                 ];
                 if (servoSlider) {
                     elementsToEnable.push(servoSlider);
                 }
                 elementsToEnable.forEach(el => { if(el) el.disabled = false; });
                 document.body.style.opacity = '1';
             }
        }

        // --- Action Functions ---
        function changeResolution(direction) {
            // ... (unchanged, now updates both stream image dimensions) ...
             if (isChangingResolution || isTogglingRecording || isChangingControl || isSettingServo || isPoweringDown) return;

            isChangingResolution = true;
            disableControls();
            statusElement.textContent = 'Changing resolution... Please wait.';
            errorElement.textContent = '';
            errorElement.style.display = 'none';

            const resolutionTimeoutId = setTimeout(() => {
                 console.warn("Resolution change timeout reached. Forcing UI cleanup.");
                 if (isChangingResolution) {
                     isChangingResolution = false;
                     enableControls();
                     updateStatus();
                 }
             }, 15000);

            fetch(`/set_resolution/${direction}`, { method: 'POST' })
                .then(response => response.json().then(data => ({ status: response.status, body: data })))
                .then(({ status, body }) => {
                    clearTimeout(resolutionTimeoutId);
                    if (status === 200 && body.success) {
                        statusElement.textContent = 'Resolution change initiated. Reloading streams...';
                        resolutionElement.textContent = body.new_resolution;
                        const [w, h] = body.new_resolution.split('x');
                        // Update dimensions for both images
                        if (streamImage) {
                            streamImage.setAttribute('width', w); streamImage.setAttribute('height', h);
                        }
                        if (cvStreamImage) {
                            cvStreamImage.setAttribute('width', w); cvStreamImage.setAttribute('height', h);
                        }
                        console.log("Resolution change successful, forcing stream reloads...");
                        setTimeout(() => {
                            // Reload both streams with cache busting
                            if (streamImage) streamImage.src = videoFeedUrlBase + "?" + Date.now();
                            if (cvStreamImage) cvStreamImage.src = cvVideoFeedUrlBase + "?" + Date.now();
                            isChangingResolution = false;
                            enableControls();
                            updateStatus();
                        }, 1500);

                    } else {
                        errorElement.textContent = `Error changing resolution: ${body.message || 'Unknown error.'}`;
                        errorElement.style.display = 'block';
                        statusElement.textContent = 'Resolution change failed.';
                        isChangingResolution = false;
                        enableControls();
                        updateStatus();
                    }
                })
                .catch(err => {
                     clearTimeout(resolutionTimeoutId);
                     console.error("Network error changing resolution:", err);
                     errorElement.textContent = `Network error: ${err.message}`;
                     errorElement.style.display = 'block';
                     statusElement.textContent = 'Resolution change failed (Network).';
                     isChangingResolution = false;
                     enableControls();
                     updateStatus();
                 });
        }

        function toggleRecording() {
            // ... (unchanged) ...
             if (isChangingResolution || isTogglingRecording || isChangingControl || isSettingServo || isPoweringDown) return;

            isTogglingRecording = true;
            disableControls();
            statusElement.textContent = 'Sending record command...';
            errorElement.textContent = '';
            errorElement.style.display = 'none';

             fetch('/toggle_recording', { method: 'POST' })
                 .then(response => { if (!response.ok) { throw new Error(`HTTP error! Status: ${response.status}`); } return response.json(); })
                 .then(data => {
                     if (data.success) {
                         currentDigitalRecordState = data.digital_recording_active;
                         updateRecordButtonState();
                         statusElement.textContent = `Digital recording ${currentDigitalRecordState ? 'enabled' : 'disabled'}. State updating...`;
                         setTimeout(updateStatus, 1500);
                     } else {
                         errorElement.textContent = `Error toggling recording: ${data.message || 'Unknown error.'}`;
                         errorElement.style.display = 'block';
                         statusElement.textContent = 'Record command failed.';
                         setTimeout(updateStatus, 1000);
                     }
                 })
                 .catch(err => {
                     console.error("Error toggling recording:", err);
                     errorElement.textContent = `Network error toggling recording: ${err.message}`;
                     errorElement.style.display = 'block';
                     statusElement.textContent = 'Command failed (Network).';
                     setTimeout(updateStatus, 1000);
                 })
                 .finally(() => {
                     isTogglingRecording = false;
                     enableControls();
                 });
        }

        function changeCameraControl(controlName, controlValue) {
            // ... (unchanged) ...
             if (isChangingResolution || isTogglingRecording || isChangingControl || isSettingServo || isPoweringDown) return;

             console.log(`Requesting control change: ${controlName} = ${controlValue}`);
             isChangingControl = true;
             disableControls();
             statusElement.textContent = `Setting ${controlName}...`;
             errorElement.textContent = '';
             errorElement.style.display = 'none';

             fetch('/set_camera_control', {
                 method: 'POST',
                 headers: { 'Content-Type': 'application/json' },
                 body: JSON.stringify({ control: controlName, value: controlValue })
             })
             .then(response => response.json().then(data => ({ status: response.status, body: data })))
             .then(({ status, body }) => {
                 if (status === 200 && body.success) {
                     statusElement.textContent = `${controlName} set.`;
                     setTimeout(updateStatus, 750);
                 } else {
                     errorElement.textContent = `Error setting ${controlName}: ${body.message || 'Unknown error.'}`;
                     errorElement.style.display = 'block';
                     statusElement.textContent = `${controlName} change failed.`;
                     setTimeout(updateStatus, 500);
                 }
             })
             .catch(err => {
                 console.error(`Network error setting ${controlName}:`, err);
                 errorElement.textContent = `Network error: ${err.message}`;
                 errorElement.style.display = 'block';
                 statusElement.textContent = `${controlName} change failed (Network).`;
                 setTimeout(updateStatus, 500);
             })
             .finally(() => {
                 isChangingControl = false;
                 enableControls();
             });
        }

        // Function to update the displayed value next to a slider
        function updateSliderValue(sliderId, value, decimalPlaces = 1) {
            // ... (unchanged) ...
            const spanId = sliderId + '-value';
            const spanElement = document.getElementById(spanId);
            if (spanElement) {
                let suffix = (sliderId === 'servo-slider') ? '°' : '';
                spanElement.textContent = parseFloat(value).toFixed(decimalPlaces) + suffix;
            }
        }

        // Function to send servo angle command
        function setServoAngle(angle) {
            // ... (unchanged) ...
             if (isChangingResolution || isTogglingRecording || isChangingControl || isSettingServo || isPoweringDown) return;

            console.log(`Requesting servo angle change: ${angle}`);
            isSettingServo = true;
            disableControls();
            statusElement.textContent = `Setting servo angle to ${angle}°...`;
            errorElement.textContent = '';
            errorElement.style.display = 'none';

            const servoTimeoutId = setTimeout(() => {
                 console.warn("Servo set timeout reached. Forcing UI cleanup.");
                 if (isSettingServo) {
                     isSettingServo = false;
                     enableControls();
                     updateStatus();
                 }
             }, 5000);


            fetch('/set_servo_angle', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ angle: angle })
            })
            .then(response => response.json().then(data => ({ status: response.status, body: data })))
            .then(({ status, body }) => {
                 clearTimeout(servoTimeoutId);
                 if (status === 200 && body.success) {
                     statusElement.textContent = `Servo angle set to ${angle}°.`;
                     if (servoAngleStatus) servoAngleStatus.textContent = `${angle}°`;
                     setTimeout(updateStatus, 750);
                 } else {
                     errorElement.textContent = `Error setting servo angle: ${body.message || 'Unknown error.'}`;
                     errorElement.style.display = 'block';
                     statusElement.textContent = `Servo angle change failed.`;
                     setTimeout(updateStatus, 500);
                 }
             })
             .catch(err => {
                 clearTimeout(servoTimeoutId);
                 console.error(`Network error setting servo angle:`, err);
                 errorElement.textContent = `Network error: ${err.message}`;
                 errorElement.style.display = 'block';
                 statusElement.textContent = `Servo angle change failed (Network).`;
                 setTimeout(updateStatus, 500);
             })
             .finally(() => {
                 isSettingServo = false;
                 enableControls();
             });
        }


        function powerDown() {
             // ... (unchanged) ...
             if (isChangingResolution || isTogglingRecording || isChangingControl || isSettingServo || isPoweringDown) return;
             if (!confirm("Are you sure you want to power down the Raspberry Pi?")) { return; }

             isPoweringDown = true;
             disableControls(true);
             statusElement.textContent = 'Powering down...';
             errorElement.textContent = '';
             errorElement.style.display = 'none';
             if (statusUpdateInterval) clearInterval(statusUpdateInterval);
             if (streamImage) { streamImage.onerror = null; streamImage.src = ""; }
             if (cvStreamImage) { cvStreamImage.onerror = null; cvStreamImage.src = ""; } // Clear CV stream too


             fetch('/power_down', { method: 'POST' })
                 .then(response => {
                     if (!response.ok) {
                         return response.json().then(data => { throw new Error(data.message || `HTTP error! Status: ${response.status}`); })
                                           .catch(() => { throw new Error(`HTTP error! Status: ${response.status}`); });
                     }
                     return response.json();
                 })
                 .then(data => {
                     if (data.success) {
                         statusElement.textContent = 'Shutdown initiated. System will reboot shortly.';
                         document.body.innerHTML = "<h1>Shutting Down... Please Wait.</h1>";
                     } else {
                         errorElement.textContent = `Shutdown request failed: ${data.message || 'Unknown error.'}`;
                         errorElement.style.display = 'block';
                         statusElement.textContent = 'Shutdown failed.';
                         isPoweringDown = false;
                         enableControls();
                         statusUpdateInterval = setInterval(updateStatus, 5000);
                     }
                 })
                 .catch(err => {
                     console.error("Error sending power down command:", err);
                     errorElement.textContent = `Error initiating shutdown: ${err.message}.`;
                     errorElement.style.display = 'block';
                     statusElement.textContent = 'Shutdown error.';
                     isPoweringDown = false;
                     enableControls();
                     statusUpdateInterval = setInterval(updateStatus, 5000);
                 });
        }

        // --- Stream Handling ---
        function handleStreamError() {
            if (!streamImage) return; // Check if element exists
            console.warn("Main stream image 'onerror' event triggered.");
            if (streamErrorTimeout || isPoweringDown || isChangingResolution) return;
            statusElement.textContent = 'Main stream interrupted. Attempting reload...';
            streamImage.src = "";
            streamErrorTimeout = setTimeout(() => {
                 console.log("Attempting main stream reload...");
                 streamImage.src = videoFeedUrlBase + "?" + Date.now();
                 streamErrorTimeout = null;
             }, 3000);
         }

        function handleStreamLoad() {
            if (!streamImage) return; // Check if element exists
             console.log("Main stream image 'onload' event fired.");
             if (streamErrorTimeout) { clearTimeout(streamErrorTimeout); streamErrorTimeout = null; }
             if (isChangingResolution) {
                 console.log("Main stream loaded after resolution change.");
                 // Only fully re-enable controls once both streams potentially load after res change
                 // This load might complete before the CV stream load
             }
         }

        // --- CV Stream Handling (Mirrors main stream) ---
        function handleCvStreamError() {
            if (!cvStreamImage) return; // Check if element exists
            console.warn("CV stream image 'onerror' event triggered.");
            if (cvStreamErrorTimeout || isPoweringDown || isChangingResolution) return;
            // Optionally update status or just log
            // statusElement.textContent = 'CV stream interrupted. Attempting reload...';
            cvStreamImage.src = "";
            cvStreamErrorTimeout = setTimeout(() => {
                 console.log("Attempting CV stream reload...");
                 cvStreamImage.src = cvVideoFeedUrlBase + "?" + Date.now();
                 cvStreamErrorTimeout = null;
             }, 3000);
         }

        function handleCvStreamLoad() {
            if (!cvStreamImage) return; // Check if element exists
             console.log("CV stream image 'onload' event fired.");
             if (cvStreamErrorTimeout) { clearTimeout(cvStreamErrorTimeout); cvStreamErrorTimeout = null; }
             if (isChangingResolution) {
                 console.log("CV stream loaded after resolution change, enabling controls.");
                 // Assume resolution change is complete if CV stream loads
                 isChangingResolution = false;
                 enableControls();
                 updateStatus();
             }
         }


        // --- Initialization ---
        document.addEventListener('DOMContentLoaded', () => {
            updateRecordButtonState();
            updateStatus();
            statusUpdateInterval = setInterval(updateStatus, 5000);
        });

        window.addEventListener('beforeunload', () => {
            if (statusUpdateInterval) clearInterval(statusUpdateInterval);
        });
    </script>
</body>
</html>
"""

# ===========================================================
# === Helper Functions ===
# ===========================================================

def generate_stream_frames():
    """Generates JPEG frames for the main MJPEG stream from the camera manager."""
    global _camera_manager
    frame_counter = 0
    last_frame_time = time.monotonic()
    logging.info("MJPEG main stream client connected. Starting frame generation.")

    if not _camera_manager:
        logging.error("Main Stream: CameraManager not initialized!")
        # Yield a placeholder error image if manager is missing
        error_img = create_placeholder_frame(640, 480, "Camera Error")
        (flag, encodedImage) = cv2.imencode(".jpg", error_img)
        if flag:
            yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' +
                   bytearray(encodedImage) + b'\r\n')
        return # Stop generation

    while True:
        # Check shutdown status frequently
        shutdown_event = _app_state.get("shutdown_event")
        if shutdown_event and shutdown_event.is_set():
            logging.info("Main stream generator: Shutdown detected, stopping.")
            break

        # Get the latest processed/combined frame from the camera manager
        frame_to_encode = _camera_manager.get_latest_frame()

        if frame_to_encode is None:
            # If no frame, maybe camera is initializing or error occurred
            time.sleep(0.05) # Wait briefly
            # Consider yielding a placeholder? For now, just wait.
            continue

        try:
            # Encode the frame as JPEG
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 85] # Quality 85 for main stream
            (flag, encodedImage) = cv2.imencode(".jpg", frame_to_encode, encode_param)
            if not flag:
                logging.warning("Main stream generator: Could not encode frame to JPEG.")
                time.sleep(0.1)
                continue

            # Yield the frame in MJPEG format
            yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' +
                   bytearray(encodedImage) + b'\r\n')

            frame_counter += 1

            # --- Rate limiting (Simple) ---
            # Limit stream FPS slightly below target capture FPS to reduce browser load
            current_time = time.monotonic()
            elapsed = current_time - last_frame_time
            try:
                 # Use target FPS from config for rate limiting
                 _, _, target_fps = _camera_manager.get_cam0_resolution_config()
                 if target_fps <= 0: target_fps = 30.0
                 stream_target_fps = min(target_fps, 30.0) # Cap stream FPS e.g. at 30
            except:
                 stream_target_fps = 25.0 # Fallback stream FPS

            target_delay = 1.0 / stream_target_fps
            sleep_time = max(0, target_delay - elapsed)
            time.sleep(sleep_time)
            last_frame_time = time.monotonic()


        except GeneratorExit:
            logging.info(f"Main streaming client disconnected after {frame_counter} frames.")
            break
        except Exception as e:
            logging.exception(f"!!! Error in main MJPEG streaming generator: {e}")
            time.sleep(0.5) # Pause after error

    logging.info("Main stream generator thread exiting.")

def generate_cv_stream_frames():
    """Generates JPEG frames for the CV MJPEG stream."""
    global _camera_manager, cv_available # Access manager and CV availability flag
    frame_counter = 0
    last_frame_time = time.monotonic()
    logging.info("MJPEG CV stream client connected. Starting frame generation.")

    placeholder_frame = None # To store placeholder if needed

    if not _camera_manager:
        logging.error("CV Stream: CameraManager not initialized!")
        placeholder_frame = create_placeholder_frame(640, 480, "CV Error: No Camera")
        cv_available = False # Treat as unavailable if camera manager missing

    while True:
        shutdown_event = _app_state.get("shutdown_event")
        if shutdown_event and shutdown_event.is_set():
            logging.info("CV stream generator: Shutdown detected, stopping.")
            break

        processed_frame = None
        error_msg = None

        # Only attempt CV processing if enabled and possible
        if config.CV_PROCESSING_ENABLED and cv_available:
            raw_frame = _camera_manager.get_latest_raw_frame0()

            if raw_frame is not None:
                try:
                    # Call the process_frame function from cv_functions module
                    # This function is expected to return the frame with overlays/filters
                    processed_frame = cv_functions.process_frame(raw_frame)
                    if processed_frame is None:
                        error_msg = "CV Processing Failed"
                except AttributeError:
                    error_msg = "CV Func Error" # cv_functions or process_frame missing
                    cv_available = False # Assume CV unavailable if function missing
                except Exception as cv_err:
                    error_msg = f"CV Error: {type(cv_err).__name__}"
                    logging.error(f"CV processing error: {cv_err}", exc_info=False) # Log less verbosely maybe
                    processed_frame = None # Ensure frame is None on error
            else:
                # No raw frame available from camera manager
                error_msg = "No Raw Frame"
                time.sleep(0.1) # Wait longer if no raw frame

        elif not config.CV_PROCESSING_ENABLED:
            error_msg = "CV Disabled"
        else: # CV not available (import failed)
             error_msg = "CV Module Error"


        # Use processed frame or create/use placeholder on error/disable
        frame_to_encode = processed_frame
        if frame_to_encode is None:
            if placeholder_frame is None or placeholder_frame.shape[1] != config.CAM0_RESOLUTIONS[_camera_manager.current_resolution_index0][0]:
                 # Create placeholder with current resolution if needed
                 w, h, _ = _camera_manager.get_cam0_resolution_config()
                 placeholder_frame = create_placeholder_frame(w, h, error_msg or "CV Unavailable")
            frame_to_encode = placeholder_frame


        # Encode and yield the frame
        try:
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 70] # Lower quality for CV stream
            (flag, encodedImage) = cv2.imencode(".jpg", frame_to_encode, encode_param)
            if not flag:
                logging.warning("CV stream generator: Could not encode frame to JPEG.")
                time.sleep(0.1)
                continue

            yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' +
                   bytearray(encodedImage) + b'\r\n')

            frame_counter += 1

            # --- Rate limiting (Simple) ---
            # Limit CV stream FPS, e.g., to 10-15 FPS
            current_time = time.monotonic()
            elapsed = current_time - last_frame_time
            cv_stream_target_fps = 15.0 # Target FPS for CV stream
            target_delay = 1.0 / cv_stream_target_fps
            sleep_time = max(0, target_delay - elapsed)
            time.sleep(sleep_time)
            last_frame_time = time.monotonic()

        except GeneratorExit:
            logging.info(f"CV streaming client disconnected after {frame_counter} frames.")
            break
        except Exception as e:
            logging.exception(f"!!! Error in CV MJPEG streaming generator: {e}")
            time.sleep(0.5)

    logging.info("CV stream generator thread exiting.")


def create_placeholder_frame(width, height, text):
    """Creates a simple black frame with text."""
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    text = text or "Error"
    # Put text in the center
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 1.0
    thickness = 2
    text_size = cv2.getTextSize(text, font, font_scale, thickness)[0]
    text_x = (width - text_size[0]) // 2
    text_y = (height + text_size[1]) // 2
    cv2.putText(frame, text, (text_x, text_y), font, font_scale, (0, 255, 255), thickness, cv2.LINE_AA)
    return frame

def build_options_html(available_options, current_selection):
    """Helper to build <option> tags for HTML select dropdowns."""
    # ... (unchanged) ...
    options_html = ""
    options_list = available_options if isinstance(available_options, list) else available_options.keys()
    for option_name in options_list:
        selected_attr = ' selected' if option_name == current_selection else ''
        options_html += f'<option value="{option_name}"{selected_attr}>{option_name}</option>'
    return options_html

# ===========================================================
# === Flask Routes ===
# ===========================================================

@app.route("/")
def index():
    """Renders the main HTML page."""
    global _camera_manager, _hardware_manager, _app_state

    if not _camera_manager or not _hardware_manager:
         logging.error("Web UI: Managers not initialized when rendering index.")
         return "Error: Application not fully initialized.", 500

    # Get initial state
    cam_state = _camera_manager.get_camera_state()
    hw_state = {
        'battery_percent': _hardware_manager.battery_percentage,
        'last_error': _hardware_manager.last_error,
        'current_servo_duty_ns': getattr(_hardware_manager, 'current_servo_duty_ns', None)
    }
    with _ui_lock:
        app_state_copy = _app_state.copy()

    # Combine hardware and camera errors, ignore audio errors
    err_msg = cam_state.get('last_error') or hw_state.get('last_error') or ""

    # Get resolution for image tags
    current_w, current_h = cam_state.get('output_frame_wh', (640, 480)) # Use output frame size
    resolution_text = f"{cam_state.get('resolution_wh', (0,0))[0]}x{cam_state.get('resolution_wh', (0,0))[1]}"

    batt_perc_initial = hw_state.get('battery_percent')
    batt_text_initial = f"{batt_perc_initial:.1f}" if batt_perc_initial is not None else "--"

    # Calculate initial servo angle (unchanged)
    servo_angle_initial = config.SERVO_CENTER_ANGLE
    # ... (servo calculation logic unchanged) ...
    if config.SERVO_ENABLED and hw_state['current_servo_duty_ns'] is not None:
        try:
            duty_range = config.SERVO_MAX_DUTY_NS - config.SERVO_MIN_DUTY_NS
            angle_range = config.SERVO_MAX_ANGLE - config.SERVO_MIN_ANGLE
            if duty_range > 0 and angle_range > 0:
                 proportion = (hw_state['current_servo_duty_ns'] - config.SERVO_MIN_DUTY_NS) / duty_range
                 servo_angle_initial = int(round(config.SERVO_MIN_ANGLE + proportion * angle_range))
                 servo_angle_initial = max(config.SERVO_MIN_ANGLE, min(config.SERVO_MAX_ANGLE, servo_angle_initial))
            else: servo_angle_initial = config.SERVO_CENTER_ANGLE
        except Exception as e:
            logging.warning(f"Could not calculate initial servo angle from duty cycle {hw_state['current_servo_duty_ns']}: {e}")
            servo_angle_initial = config.SERVO_CENTER_ANGLE


    # Build HTML options (unchanged)
    current_iso_name_initial = cam_state.get('iso_mode', config.DEFAULT_ISO_NAME)
    iso_options_html = build_options_html(config.AVAILABLE_ISO_SETTINGS, current_iso_name_initial)
    ae_options_html = build_options_html(config.AVAILABLE_AE_MODES, cam_state.get('ae_mode', config.DEFAULT_AE_MODE_NAME))
    metering_options_html = build_options_html(config.AVAILABLE_METERING_MODES, cam_state.get('metering_mode', config.DEFAULT_METERING_MODE_NAME))
    noise_reduction_options_html = build_options_html(config.AVAILABLE_NOISE_REDUCTION_MODES, cam_state.get('noise_reduction_mode', config.DEFAULT_NOISE_REDUCTION_MODE_NAME))

    # Render the template string
    return render_template_string(HTML_TEMPLATE,
        resolution_text=resolution_text,
        current_w=current_w, current_h=current_h, # Use output dimensions for placeholders
        err_msg=err_msg,
        digital_rec_state_initial=app_state_copy.get('digital_recording_active', False),
        batt_text_initial=batt_text_initial,
        # Camera Controls
        current_iso_name_initial=current_iso_name_initial, iso_options_html=iso_options_html,
        current_ae_mode_name_initial=cam_state.get('ae_mode', config.DEFAULT_AE_MODE_NAME), ae_options_html=ae_options_html,
        current_metering_mode_name_initial=cam_state.get('metering_mode', config.DEFAULT_METERING_MODE_NAME), metering_options_html=metering_options_html,
        current_noise_reduction_mode_name_initial=cam_state.get('noise_reduction_mode', config.DEFAULT_NOISE_REDUCTION_MODE_NAME), noise_reduction_options_html=noise_reduction_options_html,
        brightness_initial=cam_state.get('brightness', config.DEFAULT_BRIGHTNESS), MIN_BRIGHTNESS=config.MIN_BRIGHTNESS, MAX_BRIGHTNESS=config.MAX_BRIGHTNESS, STEP_BRIGHTNESS=config.STEP_BRIGHTNESS,
        contrast_initial=cam_state.get('contrast', config.DEFAULT_CONTRAST), MIN_CONTRAST=config.MIN_CONTRAST, MAX_CONTRAST=config.MAX_CONTRAST, STEP_CONTRAST=config.STEP_CONTRAST,
        saturation_initial=cam_state.get('saturation', config.DEFAULT_SATURATION), MIN_SATURATION=config.MIN_SATURATION, MAX_SATURATION=config.MAX_SATURATION, STEP_SATURATION=config.STEP_SATURATION,
        sharpness_initial=cam_state.get('sharpness', config.DEFAULT_SHARPNESS), MIN_SHARPNESS=config.MIN_SHARPNESS, MAX_SHARPNESS=config.MAX_SHARPNESS, STEP_SHARPNESS=config.STEP_SHARPNESS,
        # Servo Controls
        servo_enabled=config.SERVO_ENABLED, servo_angle_initial=servo_angle_initial, SERVO_MIN_ANGLE=config.SERVO_MIN_ANGLE, SERVO_MAX_ANGLE=config.SERVO_MAX_ANGLE,
        # Stream URLs
        video_feed_url=url_for('video_feed'),
        cv_video_feed_url=url_for('cv_video_feed') # Added CV stream URL
    )


@app.route("/video_feed")
def video_feed():
    """Route for the main MJPEG video stream."""
    return Response(generate_stream_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route("/cv_video_feed")
def cv_video_feed():
    """Route for the Computer Vision MJPEG video stream."""
    return Response(generate_cv_stream_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route("/status")
def status():
    """Returns the current status of the application as JSON."""
    global _camera_manager, _hardware_manager, _app_state

    if not _camera_manager or not _hardware_manager:
        return jsonify({'error': 'Application not fully initialized.'}), 500

    cam_state = _camera_manager.get_camera_state()
    batt_perc = _hardware_manager.battery_percentage
    current_servo_duty = getattr(_hardware_manager, 'current_servo_duty_ns', None)

    # Combine hardware and camera errors (audio error check removed)
    err_msg = cam_state.get('last_error') or _hardware_manager.last_error or ""

    # Auto-clear non-persistent errors if things seem okay now
    latest_frame = _camera_manager.get_latest_frame() # Check main stream frame
    if latest_frame is not None:
        if err_msg and ("capture" in err_msg.lower() or "unavailable" in err_msg.lower()):
             # Clear capture/init errors if we are getting frames
             if _camera_manager and _camera_manager.last_error == err_msg: _camera_manager.last_error = None
             if _hardware_manager and _hardware_manager.last_error == err_msg: _hardware_manager.last_error = None
             err_msg = "" # Clear the combined message
    if batt_perc is not None:
         if err_msg and ("battery" in err_msg.lower() or "ina219" in err_msg.lower() or "i2c" in err_msg.lower()):
             # Clear battery errors if we get a reading
             if _hardware_manager and _hardware_manager.last_error == err_msg: _hardware_manager.last_error = None
             if _camera_manager and _camera_manager.last_error == err_msg: _camera_manager.last_error = None
             err_msg = "" # Clear the combined message


    status_text = "Streaming"
    if cam_state.get('is_recording'):
        rec_count = len(cam_state.get('recording_paths', []))
        status_text += f" (Recording to {rec_count} USB(s))" if rec_count > 0 else " (ERROR: Recording active but no paths!)"
        if rec_count == 0 and not err_msg: err_msg = "Inconsistent State: Recording active but no paths."

    with _ui_lock: digital_rec_active = _app_state.get("digital_recording_active", False)

    # Calculate current servo angle (unchanged)
    current_servo_angle = None
    # ... (servo calculation logic unchanged) ...
    if config.SERVO_ENABLED and current_servo_duty is not None:
         try:
             duty_range = config.SERVO_MAX_DUTY_NS - config.SERVO_MIN_DUTY_NS
             angle_range = config.SERVO_MAX_ANGLE - config.SERVO_MIN_ANGLE
             if duty_range > 0 and angle_range > 0:
                  proportion = (current_servo_duty - config.SERVO_MIN_DUTY_NS) / duty_range
                  current_servo_angle = int(round(config.SERVO_MIN_ANGLE + proportion * angle_range))
                  current_servo_angle = max(config.SERVO_MIN_ANGLE, min(config.SERVO_MAX_ANGLE, current_servo_angle))
         except Exception: pass


    status_data = {
        'is_recording': cam_state.get('is_recording', False),
        'digital_recording_active': digital_rec_active,
        'resolution': f"{cam_state.get('resolution_wh', ['?','?'])[0]}x{cam_state.get('resolution_wh', ['?','?'])[1]}",
        'measured_fps': cam_state.get('measured_cam0_fps'), # Added FPS
        'status_text': status_text, 'error': err_msg,
        'active_recordings': cam_state.get('recording_paths', []),
        'battery_percent': batt_perc,
        'iso_mode': cam_state.get('iso_mode'), 'ae_mode': cam_state.get('ae_mode'),
        'metering_mode': cam_state.get('metering_mode'), 'noise_reduction_mode': cam_state.get('noise_reduction_mode'),
        'brightness': cam_state.get('brightness'), 'contrast': cam_state.get('contrast'),
        'saturation': cam_state.get('saturation'), 'sharpness': cam_state.get('sharpness'),
        'servo_angle': current_servo_angle,
    }
    return jsonify(status_data)


@app.route("/set_resolution/<direction>", methods=['POST'])
def set_resolution(direction):
    """Handles requests to change the camera resolution."""
    # ... (unchanged logic, already uses correct attributes/constants) ...
    global _camera_manager, _app_state

    if not _camera_manager: return jsonify({'success': False, 'message': 'Camera manager not ready.'}), 500

    with _ui_lock:
        if _app_state.get("reconfigure_resolution_index") is not None:
            return jsonify({'success': False, 'message': 'Reconfiguration already in progress.'}), 429

        current_index = _camera_manager.current_resolution_index0
        original_index = current_index
        new_index = current_index

        if direction == 'up': new_index += 1
        elif direction == 'down': new_index -= 1
        else: return jsonify({'success': False, 'message': 'Invalid direction specified.'}), 400

        num_resolutions = len(config.CAM0_RESOLUTIONS)
        new_index = max(0, min(num_resolutions - 1, new_index))

        if new_index == original_index:
            msg = 'Already at highest resolution.' if direction == 'up' else 'Already at lowest resolution.'
            return jsonify({'success': False, 'message': msg}), 400

        _app_state["reconfigure_resolution_index"] = new_index
        new_w, new_h, _ = config.CAM0_RESOLUTIONS[new_index]
        logging.info(f"Web request: Queue resolution change index {original_index} -> {new_index} ({new_w}x{new_h})")
        if _camera_manager: _camera_manager.last_error = None # Clear error on success

        # Return the resolution text the UI expects
        return jsonify({'success': True, 'message': 'Resolution change requested.', 'new_resolution': f"{new_w}x{new_h}"})


@app.route('/toggle_recording', methods=['POST'])
def toggle_recording():
    """Toggles the digital recording trigger state."""
    # ... (unchanged logic, audio error clearing removed implicitly) ...
    global _app_state, _camera_manager, _hardware_manager
    new_state = False
    with _ui_lock:
        current_state = _app_state.get("digital_recording_active", False)
        _app_state["digital_recording_active"] = not current_state
        new_state = _app_state["digital_recording_active"]
        logging.info(f"Digital recording trigger toggled via web UI to: {'ON' if new_state else 'OFF'}")
        # Clear potential previous recording errors when user interacts
        if _camera_manager:
            if _camera_manager.last_error and ("Recording" in _camera_manager.last_error or "writer" in _camera_manager.last_error or "USB" in _camera_manager.last_error or "sync" in _camera_manager.last_error):
                logging.info(f"Clearing previous video error via toggle: '{_camera_manager.last_error}'")
                _camera_manager.last_error = None
        if _hardware_manager and _hardware_manager.last_error and ("Recording" in _hardware_manager.last_error):
             _hardware_manager.last_error = None # Should not happen but check anyway

    return jsonify({'success': True, 'digital_recording_active': new_state})


@app.route('/set_camera_control', methods=['POST'])
def set_camera_control():
    """Sets a specific camera control value."""
    # ... (unchanged logic, already uses correct attributes/constants) ...
    global _camera_manager

    if not _camera_manager: return jsonify({'success': False, 'message': 'Camera manager not ready.'}), 500
    if not _camera_manager.is_initialized0: return jsonify({'success': False, 'message': 'Camera 0 not initialized.'}), 503

    control_name = None
    try:
        data = request.get_json()
        if not data or 'control' not in data or 'value' not in data:
            return jsonify({'success': False, 'message': 'Missing control or value in request.'}), 400

        control_name = data['control']
        control_value = data['value']
        logging.info(f"Web request: Set control '{control_name}' to '{control_value}'")

        control_dict_to_set = {}
        parsed_value = None
        validation_error = None
        iso_name = None

        # --- Validate and Parse Value ---
        if control_name == 'ISOSetting':
            iso_name = control_value
            analogue_gain = config.AVAILABLE_ISO_SETTINGS.get(iso_name)
            if analogue_gain is not None:
                parsed_value = analogue_gain; control_dict_to_set["AnalogueGain"] = parsed_value
                if analogue_gain == 0.0: control_dict_to_set["AeEnable"] = True
                control_name = 'AnalogueGain' # Use internal name for apply function
            else: validation_error = f"Invalid ISOSetting value: {iso_name}"
        elif control_name == 'AeExposureMode':
            enum_val = controls.AeExposureModeEnum.__members__.get(control_value)
            if enum_val is not None: parsed_value = enum_val; control_dict_to_set["AeEnable"] = True
            else: validation_error = f"Invalid AeExposureMode value: {control_value}"
        elif control_name == 'AeMeteringMode':
            enum_val = controls.AeMeteringModeEnum.__members__.get(control_value)
            if enum_val is not None: parsed_value = enum_val; control_dict_to_set["AeEnable"] = True
            else: validation_error = f"Invalid AeMeteringMode value: {control_value}"
        elif control_name == 'NoiseReductionMode':
            try:
                enum_val = controls.draft.NoiseReductionModeEnum.__members__.get(control_value)
                if enum_val is not None: parsed_value = enum_val
                else: validation_error = f"Invalid NoiseReductionMode value: {control_value}"
            except AttributeError: validation_error = "NoiseReductionMode control not available."
        elif control_name == 'Brightness':
            try: val = float(control_value); parsed_value = max(config.MIN_BRIGHTNESS, min(config.MAX_BRIGHTNESS, val))
            except ValueError: validation_error = "Invalid numeric value for Brightness"
        elif control_name == 'Contrast':
            try: val = float(control_value); parsed_value = max(config.MIN_CONTRAST, min(config.MAX_CONTRAST, val))
            except ValueError: validation_error = "Invalid numeric value for Contrast"
        elif control_name == 'Saturation':
            try: val = float(control_value); parsed_value = max(config.MIN_SATURATION, min(config.MAX_SATURATION, val))
            except ValueError: validation_error = "Invalid numeric value for Saturation"
        elif control_name == 'Sharpness':
            try: val = float(control_value); parsed_value = max(config.MIN_SHARPNESS, min(config.MAX_SHARPNESS, val))
            except ValueError: validation_error = "Invalid numeric value for Sharpness"
        else:
            if 'AnalogueGain' not in control_dict_to_set: validation_error = f"Unknown control name: {control_name}"

        if validation_error:
            logging.error(f"Control validation failed for '{control_name}'='{control_value}': {validation_error}")
            return jsonify({'success': False, 'message': validation_error}), 400

        if parsed_value is not None and control_name not in control_dict_to_set:
             control_dict_to_set[control_name] = parsed_value

        if not control_dict_to_set:
             return jsonify({'success': False, 'message': 'Internal processing error.'}), 500

        if _camera_manager.apply_camera_controls(control_dict_to_set):
             log_key = 'ISOSetting' if iso_name else control_name
             log_val = iso_name if iso_name else control_value
             logging.info(f"Successfully applied control request: {log_key}={log_val}")
             return jsonify({'success': True, 'message': f'{log_key} set.'})
        else:
             error_msg = _camera_manager.last_error or f"Failed to apply {control_name}"
             return jsonify({'success': False, 'message': error_msg}), 500

    except Exception as e:
        logging.error(f"Error processing set_camera_control '{control_name}': {e}", exc_info=True)
        return jsonify({'success': False, 'message': f'Unexpected server error: {e}'}), 500


@app.route('/set_servo_angle', methods=['POST'])
def set_servo_angle_route():
    """Sets the servo angle via the hardware manager."""
    # ... (unchanged logic) ...
    global _hardware_manager

    if not config.SERVO_ENABLED: return jsonify({'success': False, 'message': 'Servo control is disabled in config.'}), 403
    if not _hardware_manager: return jsonify({'success': False, 'message': 'Hardware manager not ready.'}), 500

    target_angle = None
    try:
        data = request.get_json()
        if not data or 'angle' not in data:
            return jsonify({'success': False, 'message': 'Missing angle in request.'}), 400

        target_angle = data['angle']
        logging.info(f"Web request: Set servo angle to '{target_angle}'")

        try: angle_float = float(target_angle)
        except ValueError: return jsonify({'success': False, 'message': 'Invalid angle value.'}), 400

        _hardware_manager.set_servo(angle_float)
        logging.info(f"Servo angle set to {angle_float} degrees.")
        return jsonify({'success': True, 'message': 'Servo angle set.'})

    except Exception as e:
        logging.error(f"Error processing set_servo_angle '{target_angle}': {e}", exc_info=True)
        return jsonify({'success': False, 'message': f'Unexpected server error: {e}'}), 500


@app.route('/power_down', methods=['POST'])
def power_down():
    """Signals the main application to initiate shutdown and reboot."""
    # ... (unchanged logic) ...
    global _app_state, _camera_manager, _hardware_manager
    logging.warning("Received request for power down via web UI. Setting flags.")

    with _ui_lock:
        _app_state["reboot_requested"] = True
        shutdown_event = _app_state.get("shutdown_event")
        if shutdown_event: shutdown_event.set()
        else:
            return jsonify({'success': False, 'message': 'Internal server error: Shutdown event not configured.'}), 500

    if _camera_manager: _camera_manager.last_error = "Shutdown initiated via web UI..."
    if _hardware_manager: _hardware_manager.last_error = "Shutdown initiated via web UI..."

    return jsonify({'success': True, 'message': 'Shutdown initiated. System will reboot after cleanup.'})


# ===========================================================
# === Web Server Setup Function ===
# ===========================================================

def setup_web_app(camera_manager, hardware_manager, shutdown_event_ref):
    """
    Sets up the Flask application context with manager instances and shared event.
    """
    global _camera_manager, _hardware_manager, _app_state
    _camera_manager = camera_manager
    _hardware_manager = hardware_manager
    with _ui_lock:
        _app_state["shutdown_event"] = shutdown_event_ref
        _app_state["digital_recording_active"] = False
        _app_state["reconfigure_resolution_index"] = None
        _app_state["reboot_requested"] = False
    logging.info("Web application context configured with manager instances.")


def run_web_server():
    """Runs the Flask web server."""
    logging.info(f"Starting Flask web server on 0.0.0.0:{config.WEB_PORT}...")
    try:
        # Use werkzeug server directly for potentially better control if needed,
        # but app.run is usually fine for development/simple deployment.
        app.run(host='0.0.0.0', port=config.WEB_PORT, debug=False, use_reloader=False, threaded=True)
    except Exception as e:
        logging.exception(f"!!! Flask web server failed: {e}")
        # Signal main thread to shut down if Flask fails catastrophically
        with _ui_lock:
            shutdown_event = _app_state.get("shutdown_event")
            if shutdown_event and not shutdown_event.is_set():
                logging.error("Signaling main thread shutdown due to Flask error.")
                shutdown_event.set()

# Example usage block removed