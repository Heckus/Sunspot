# -*- coding: utf-8 -*-
"""
web_ui.py

Handles the Flask web server and user interface for the Pi Camera application.
Provides routes for streaming, status updates, and control commands.
Refactored for single-camera operation.
"""

import logging
import time
import cv2
import threading # Import threading for lock
from flask import Flask, Response, render_template_string, jsonify, request, url_for
from libcamera import controls # Needed for control validation

# Import configuration and managers (assuming they are in the same directory)
import config
# We don't directly import CameraManager/HardwareManager here,
# instances will be passed to the setup function.

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
# ===========================================================
# Keep the template as a large string for simplicity in this single file UI setup
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

        .status-panel, .controls-panel, .sliders-panel { background-color: #eef; padding: 15px; border-radius: 5px; }
        .panel-title { font-weight: bold; margin-bottom: 10px; border-bottom: 1px solid #ccc; padding-bottom: 5px; }

        /* Status Grid */
        .status-grid { display: grid; grid-template-columns: auto 1fr; gap: 5px 10px; align-items: center; }
        .status-grid span:first-child { font-weight: bold; color: #555; text-align: right;}
        #status, #rec-status, #resolution, #battery-level,
        #awb-mode-status, #ae-mode-status, #metering-mode-status, #nr-mode-status /* Status IDs */
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

        /* Slider Controls */
        .slider-controls { display: grid; grid-template-columns: auto 1fr auto; gap: 5px 10px; align-items: center; margin-bottom: 8px; }
        .slider-controls label { font-weight: normal; color: #444; text-align: right; font-size: 0.9em;}
        .slider-controls input[type=range] { width: 100%; margin: 0; padding: 0; cursor: pointer; }
        .slider-controls span { font-size: 0.9em; color: #0056b3; min-width: 35px; text-align: right; } /* For slider value display */

        #error { color: red; margin-top: 15px; white-space: pre-wrap; font-weight: bold; min-height: 1.2em; text-align: center; background-color: #ffebeb; border: 1px solid red; padding: 8px; border-radius: 4px; display: none; /* Initially hidden */ }
        img#stream { display: block; margin: 15px auto; border: 1px solid black; max-width: 100%; height: auto; background-color: #ddd; } /* Placeholder color */

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
                     <span>Battery:</span> <span id="battery-level">{{ batt_text_initial }}%</span>
                     <hr style="grid-column: 1 / -1; border-top: 1px dashed #bbb; border-bottom: none; margin: 5px 0;">
                     <span>AWB Mode:</span> <span id="awb-mode-status">{{ current_awb_mode_name_initial }}</span>
                     <span>AE Mode:</span> <span id="ae-mode-status">{{ current_ae_mode_name_initial }}</span>
                     <span>Metering:</span> <span id="metering-mode-status">{{ current_metering_mode_name_initial }}</span>
                     <span>Noise Red.:</span> <span id="nr-mode-status">{{ current_noise_reduction_mode_name_initial }}</span>
                 </div>
             </div>

             <div class="controls-panel">
                 <div class="panel-title">Mode Controls</div>
                 <div class="mode-controls">
                     <label for="awb-select">AWB Mode:</label>
                     <select id="awb-select" onchange="changeCameraControl('AwbMode', this.value)" title="Select Auto White Balance Mode">
                         {{ awb_options_html | safe }} {# Use safe filter for HTML options #}
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
                     <input type="range" id="brightness-slider" min="{{ MIN_BRIGHTNESS }}" max="{{ MAX_BRIGHTNESS }}" step="{{ STEP_BRIGHTNESS }}" value="{{ brightness_initial }}" oninput="updateSliderValue(this.id, this.value)" onchange="changeCameraControl('Brightness', this.value)" title="Adjust Brightness">
                     <span id="brightness-slider-value">{{ "%.1f" | format(brightness_initial) }}</span> {# Format initial value #}

                     <label for="contrast-slider">Contrast:</label>
                     <input type="range" id="contrast-slider" min="{{ MIN_CONTRAST }}" max="{{ MAX_CONTRAST }}" step="{{ STEP_CONTRAST }}" value="{{ contrast_initial }}" oninput="updateSliderValue(this.id, this.value)" onchange="changeCameraControl('Contrast', this.value)" title="Adjust Contrast">
                     <span id="contrast-slider-value">{{ "%.1f" | format(contrast_initial) }}</span>

                     <label for="saturation-slider">Saturation:</label>
                     <input type="range" id="saturation-slider" min="{{ MIN_SATURATION }}" max="{{ MAX_SATURATION }}" step="{{ STEP_SATURATION }}" value="{{ saturation_initial }}" oninput="updateSliderValue(this.id, this.value)" onchange="changeCameraControl('Saturation', this.value)" title="Adjust Saturation">
                     <span id="saturation-slider-value">{{ "%.1f" | format(saturation_initial) }}</span>

                     <label for="sharpness-slider">Sharpness:</label>
                     <input type="range" id="sharpness-slider" min="{{ MIN_SHARPNESS }}" max="{{ MAX_SHARPNESS }}" step="{{ STEP_SHARPNESS }}" value="{{ sharpness_initial }}" oninput="updateSliderValue(this.id, this.value)" onchange="changeCameraControl('Sharpness', this.value)" title="Adjust Sharpness">
                     <span id="sharpness-slider-value">{{ "%.1f" | format(sharpness_initial) }}</span>
                 </div>
             </div>

         </div>
         <div id="error" {% if err_msg %}style="display: block;"{% endif %}>{{ err_msg }}</div>
         <img id="stream" src="{{ video_feed_url }}" width="{{ current_w }}" height="{{ current_h }}" alt="Loading stream..."
              onerror="handleStreamError()" onload="handleStreamLoad()">
    </div>

    <script>
        // Use Jinja var for initial state:
        let currentDigitalRecordState = {{ 'true' if digital_rec_state_initial else 'false' }};
        // Store base URL generated by Python/Flask
        const videoFeedUrlBase = "{{ video_feed_url }}";

        // Get Element References
        const statusElement = document.getElementById('status');
        const resolutionElement = document.getElementById('resolution');
        const errorElement = document.getElementById('error');
        const streamImage = document.getElementById('stream');
        const btnUp = document.getElementById('btn-up');
        const btnDown = document.getElementById('btn-down');
        const btnRecord = document.getElementById('btn-record');
        const btnPowerdown = document.getElementById('btn-powerdown');
        const recStatusElement = document.getElementById('rec-status');
        const batteryLevelElement = document.getElementById('battery-level');
        const awbStatusElement = document.getElementById('awb-mode-status');
        const aeStatusElement = document.getElementById('ae-mode-status');
        const meteringStatusElement = document.getElementById('metering-mode-status');
        const nrStatusElement = document.getElementById('nr-mode-status');
        const awbSelectElement = document.getElementById('awb-select');
        const aeSelectElement = document.getElementById('ae-select');
        const meteringSelectElement = document.getElementById('metering-select');
        const nrSelectElement = document.getElementById('nr-select');
        const brightnessSlider = document.getElementById('brightness-slider');
        const contrastSlider = document.getElementById('contrast-slider');
        const saturationSlider = document.getElementById('saturation-slider');
        const sharpnessSlider = document.getElementById('sharpness-slider');
        const brightnessValueSpan = document.getElementById('brightness-slider-value');
        const contrastValueSpan = document.getElementById('contrast-slider-value');
        const saturationValueSpan = document.getElementById('saturation-slider-value');
        const sharpnessValueSpan = document.getElementById('sharpness-slider-value');

        // State Variables
        let isChangingResolution = false;
        let isTogglingRecording = false;
        let isChangingControl = false;
        let isPoweringDown = false;
        let statusUpdateInterval;
        let streamErrorTimeout = null;

        // --- UI Update Functions ---
        function updateRecordButtonState() {
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
            if (isChangingResolution || isTogglingRecording || isChangingControl || isPoweringDown) return;

            fetch('/status')
                .then(response => { if (!response.ok) { throw new Error(`HTTP error! Status: ${response.status}`); } return response.json(); })
                .then(data => {
                    // Update general status
                    statusElement.textContent = data.status_text || 'Unknown';
                    recStatusElement.textContent = data.is_recording ? "ACTIVE" : "OFF";
                    recStatusElement.classList.toggle('active', data.is_recording);

                    // Update resolution display and image dimensions if changed
                    if (data.resolution && resolutionElement.textContent !== data.resolution) {
                        resolutionElement.textContent = data.resolution;
                        const [w, h] = data.resolution.split('x');
                        if (streamImage.getAttribute('width') != w || streamImage.getAttribute('height') != h) {
                            console.log(`Updating stream image dimensions to ${w}x${h}`);
                            streamImage.setAttribute('width', w);
                            streamImage.setAttribute('height', h);
                        }
                    }

                    // Display/clear error messages
                    if (data.error) {
                        errorElement.textContent = data.error;
                        errorElement.style.display = 'block';
                    } else {
                        // Clear error only if it was previously visible
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
                    // Use the keys from the refactored camera_manager state
                    updateControlUI('awb_mode', data.awb_mode, awbStatusElement, awbSelectElement);
                    updateControlUI('ae_mode', data.ae_mode, aeStatusElement, aeSelectElement);
                    updateControlUI('metering_mode', data.metering_mode, meteringStatusElement, meteringSelectElement);
                    updateControlUI('noise_reduction_mode', data.noise_reduction_mode, nrStatusElement, nrSelectElement);
                    updateControlUI('brightness', data.brightness, null, brightnessSlider, brightnessValueSpan);
                    updateControlUI('contrast', data.contrast, null, contrastSlider, contrastValueSpan);
                    updateControlUI('saturation', data.saturation, null, saturationSlider, saturationValueSpan);
                    updateControlUI('sharpness', data.sharpness, null, sharpnessSlider, sharpnessValueSpan);

                    // Update battery level display
                    if (data.battery_percent !== null && data.battery_percent !== undefined) {
                         batteryLevelElement.textContent = data.battery_percent.toFixed(1) + '%'; // Add % sign
                    } else {
                         batteryLevelElement.textContent = "--%"; // Placeholder if no reading
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
                    batteryLevelElement.textContent = "Err";
                    awbStatusElement.textContent = "Err";
                    aeStatusElement.textContent = "Err";
                    meteringStatusElement.textContent = "Err";
                    nrStatusElement.textContent = "Err";
                });
        }

        // Helper function to update individual control UI elements
        function updateControlUI(controlKey, newValue, statusEl, controlEl, valueSpanEl = null) {
             // Only update if new value is provided
             if (newValue === undefined || newValue === null) return;

             let currentUIValue = controlEl ? controlEl.value : null;
             let formattedNewValue = newValue;

             // Handle sliders specifically (formatting and value span)
             if (valueSpanEl) {
                 formattedNewValue = parseFloat(newValue).toFixed(1);
                 currentUIValue = controlEl ? parseFloat(controlEl.value).toFixed(1) : null; // Get current slider value
                 // Update the text span next to the slider
                 if (valueSpanEl.textContent !== formattedNewValue) {
                     valueSpanEl.textContent = formattedNewValue;
                 }
             } else if (statusEl) { // Handle dropdown status text display
                 if (statusEl.textContent !== newValue.toString()) {
                     statusEl.textContent = newValue.toString();
                 }
             }

             // Update the control element itself (select or range slider)
             // Only update if the value differs and the user isn't actively changing it
             if (controlEl && currentUIValue !== formattedNewValue.toString() && !isChangingControl && !isChangingResolution) {
                 console.log(`Status update forcing UI for ${controlKey}: UI='${currentUIValue}' -> Status='${formattedNewValue}'`);
                 controlEl.value = newValue; // Use the original newValue for setting element value
             }
         }


        // Function to disable all controls during operations
        function disableControls(poweringDown = false) {
            [btnUp, btnDown, btnRecord, btnPowerdown,
             awbSelectElement, aeSelectElement, meteringSelectElement, nrSelectElement,
             brightnessSlider, contrastSlider, saturationSlider, sharpnessSlider
            ].forEach(el => el.disabled = true);
            // Dim the page slightly during power down
            if(poweringDown) { document.body.style.opacity = '0.7'; }
        }

        // Function to re-enable controls
        function enableControls() {
             // Only enable if not in the process of powering down
             if (!isPoweringDown) {
                 [btnUp, btnDown, btnRecord, btnPowerdown,
                  awbSelectElement, aeSelectElement, meteringSelectElement, nrSelectElement,
                  brightnessSlider, contrastSlider, saturationSlider, sharpnessSlider
                 ].forEach(el => el.disabled = false);
                 document.body.style.opacity = '1'; // Restore full opacity
             }
         }

        // --- Action Functions ---
        function changeResolution(direction) {
            // Prevent action if another operation is in progress
            if (isChangingResolution || isTogglingRecording || isChangingControl || isPoweringDown) return;

            isChangingResolution = true;
            disableControls();
            statusElement.textContent = 'Changing resolution... Please wait.';
            errorElement.textContent = ''; // Clear previous errors
            errorElement.style.display = 'none';

            // Timeout to re-enable controls if the backend hangs
            const resolutionTimeoutId = setTimeout(() => {
                 console.warn("Resolution change timeout reached. Forcing UI cleanup.");
                 if (isChangingResolution) { // Check flag again
                     isChangingResolution = false;
                     enableControls();
                     updateStatus(); // Refresh status
                 }
             }, 10000); // 10 second timeout

            // Send request to backend
            fetch(`/set_resolution/${direction}`, { method: 'POST' })
                .then(response => response.json().then(data => ({ status: response.status, body: data })))
                .then(({ status, body }) => {
                    clearTimeout(resolutionTimeoutId); // Clear timeout on response
                    if (status === 200 && body.success) {
                        statusElement.textContent = 'Resolution change initiated. Reloading stream...';
                        resolutionElement.textContent = body.new_resolution; // Update display immediately
                        const [w, h] = body.new_resolution.split('x');
                        streamImage.setAttribute('width', w); // Update image tag dimensions
                        streamImage.setAttribute('height', h);
                        console.log("Resolution change request successful, forcing stream reload...");
                        // Force stream reload after a delay to allow backend reconfiguration
                        setTimeout(() => {
                            // Add cache-busting query parameter
                            streamImage.src = videoFeedUrlBase + "?" + Date.now();
                            // Re-enable controls AFTER stream starts reloading
                            isChangingResolution = false;
                            enableControls();
                            updateStatus(); // Refresh status
                        }, 1500); // Delay stream reload slightly

                    } else {
                        // Handle failure response from backend
                        errorElement.textContent = `Error changing resolution: ${body.message || 'Unknown error.'}`;
                        errorElement.style.display = 'block';
                        statusElement.textContent = 'Resolution change failed.';
                        isChangingResolution = false; // Re-enable controls on failure
                        enableControls();
                        updateStatus(); // Refresh status
                    }
                })
                .catch(err => {
                     // Handle network errors
                     clearTimeout(resolutionTimeoutId);
                     console.error("Network error sending resolution change:", err);
                     errorElement.textContent = `Network error changing resolution: ${err.message}`;
                     errorElement.style.display = 'block';
                     statusElement.textContent = 'Resolution change failed (Network).';
                     isChangingResolution = false; // Re-enable controls on failure
                     enableControls();
                     updateStatus(); // Refresh status
                 });
        }

        function toggleRecording() {
            // Prevent action if another operation is in progress
            if (isChangingResolution || isTogglingRecording || isChangingControl || isPoweringDown) return;

            isTogglingRecording = true;
            disableControls();
            statusElement.textContent = 'Sending record command...';
            errorElement.textContent = ''; // Clear previous errors
            errorElement.style.display = 'none';

             fetch('/toggle_recording', { method: 'POST' })
                 .then(response => { if (!response.ok) { throw new Error(`HTTP error! Status: ${response.status}`); } return response.json(); })
                 .then(data => {
                     if (data.success) {
                         // Update state and UI based on response
                         currentDigitalRecordState = data.digital_recording_active;
                         updateRecordButtonState();
                         statusElement.textContent = `Digital recording ${currentDigitalRecordState ? 'enabled' : 'disabled'}. State updating...`;
                         // Fetch status again shortly after to confirm backend state
                         setTimeout(updateStatus, 1500);
                     } else {
                         // Handle failure response
                         errorElement.textContent = `Error toggling recording: ${data.message || 'Unknown error.'}`;
                         errorElement.style.display = 'block';
                         statusElement.textContent = 'Record command failed.';
                         setTimeout(updateStatus, 1000); // Update status to reflect failure
                     }
                 })
                 .catch(err => {
                     // Handle network errors
                     console.error("Error toggling recording:", err);
                     errorElement.textContent = `Network error toggling recording: ${err.message}`;
                     errorElement.style.display = 'block';
                     statusElement.textContent = 'Command failed (Network).';
                     setTimeout(updateStatus, 1000); // Update status
                 })
                 .finally(() => {
                     // Re-enable controls once operation is complete (success or fail)
                     isTogglingRecording = false;
                     enableControls();
                 });
         }

        function changeCameraControl(controlName, controlValue) {
             // Prevent action if another operation is in progress
             if (isChangingResolution || isTogglingRecording || isChangingControl || isPoweringDown) return;

             console.log(`Requesting control change: ${controlName} = ${controlValue}`);
             isChangingControl = true;
             disableControls();
             statusElement.textContent = `Setting ${controlName}...`;
             errorElement.textContent = ''; // Clear previous errors
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
                     // Optionally update UI immediately based on success, then verify with status update
                     // updateControlUI(controlName.toLowerCase(), controlValue, ...); // Be careful with this
                     setTimeout(updateStatus, 750); // Fetch status slightly later to confirm
                 } else {
                     // Handle failure response
                     errorElement.textContent = `Error setting ${controlName}: ${body.message || 'Unknown error.'}`;
                     errorElement.style.display = 'block';
                     statusElement.textContent = `${controlName} change failed.`;
                     setTimeout(updateStatus, 500); // Fetch status to revert UI if needed
                 }
             })
             .catch(err => {
                 // Handle network errors
                 console.error(`Network error setting ${controlName}:`, err);
                 errorElement.textContent = `Network error: ${err.message}`;
                 errorElement.style.display = 'block';
                 statusElement.textContent = `${controlName} change failed (Network).`;
                 setTimeout(updateStatus, 500); // Fetch status to revert UI if needed
             })
             .finally(() => {
                 // Re-enable controls once operation is complete
                 isChangingControl = false;
                 enableControls();
             });
         }

        // Function to update the displayed value next to a slider
        function updateSliderValue(sliderId, value) {
            const spanId = sliderId + '-value';
            const spanElement = document.getElementById(spanId);
            if (spanElement) {
                // Format to one decimal place
                spanElement.textContent = parseFloat(value).toFixed(1);
            }
        }

        function powerDown() {
             // Prevent action if another operation is in progress
             if (isChangingResolution || isTogglingRecording || isChangingControl || isPoweringDown) return;
             // Confirm with user
             if (!confirm("Are you sure you want to power down the Raspberry Pi?")) { return; }

             isPoweringDown = true;
             disableControls(true); // Disable controls and dim page
             statusElement.textContent = 'Powering down...';
             errorElement.textContent = ''; // Clear previous errors
             errorElement.style.display = 'none';
             // Stop status updates and stream handling during shutdown
             if (statusUpdateInterval) clearInterval(statusUpdateInterval);
             streamImage.onerror = null;
             streamImage.src = ""; // Clear stream image

             fetch('/power_down', { method: 'POST' })
                 .then(response => {
                     // Try to parse JSON error message even on non-200 responses
                     if (!response.ok) {
                         return response.json().then(data => {
                             throw new Error(data.message || `HTTP error! Status: ${response.status}`);
                         }).catch(() => { // Fallback if JSON parsing fails
                             throw new Error(`HTTP error! Status: ${response.status}`);
                         });
                     }
                     return response.json();
                 })
                 .then(data => {
                     if (data.success) {
                         statusElement.textContent = 'Shutdown initiated. System will reboot shortly.';
                         // Replace page content to indicate shutdown
                         document.body.innerHTML = "<h1>Shutting Down... Please Wait.</h1>";
                     } else {
                         // Handle failure response from backend
                         errorElement.textContent = `Shutdown request failed: ${data.message || 'Unknown error.'}`;
                         errorElement.style.display = 'block';
                         statusElement.textContent = 'Shutdown failed.';
                         isPoweringDown = false; // Re-allow actions
                         enableControls();
                         // Restart status updates on failure
                         statusUpdateInterval = setInterval(updateStatus, 5000);
                     }
                 })
                 .catch(err => {
                     // Handle network errors
                     console.error("Error sending power down command:", err);
                     errorElement.textContent = `Error initiating shutdown: ${err.message}.`;
                     errorElement.style.display = 'block';
                     statusElement.textContent = 'Shutdown error.';
                     isPoweringDown = false; // Re-allow actions
                     enableControls();
                     // Restart status updates on failure
                     statusUpdateInterval = setInterval(updateStatus, 5000);
                 });
         }

        // --- Stream Handling ---
        function handleStreamError() {
            console.warn("Stream image 'onerror' event triggered.");
            // Avoid retrying if already retrying, shutting down, or changing resolution
            if (streamErrorTimeout || isPoweringDown || isChangingResolution) return;
            statusElement.textContent = 'Stream interrupted. Attempting reload...';
            streamImage.src = ""; // Clear broken image placeholder
            // Set timeout to retry loading the stream
            streamErrorTimeout = setTimeout(() => {
                 console.log("Attempting stream reload...");
                 // Add cache-busting query parameter
                 streamImage.src = videoFeedUrlBase + "?" + Date.now();
                 streamErrorTimeout = null; // Clear timeout ID
                 // Don't update status immediately, wait for onload or next error
             }, 3000); // Wait 3 seconds before retrying
         }

        function handleStreamLoad() {
             console.log("Stream image 'onload' event fired.");
             // If we were in an error state, clear the timeout
             if (streamErrorTimeout) {
                 clearTimeout(streamErrorTimeout);
                 streamErrorTimeout = null;
             }
             // This confirms the image element itself loaded (or reloaded)
             // If a resolution change was in progress, mark it as complete and enable controls
             if (isChangingResolution) {
                 console.log("Stream loaded after resolution change, enabling controls.");
                 isChangingResolution = false;
                 enableControls();
                 updateStatus(); // Update status now that stream is loaded
             }
         }

        // --- Initialization ---
        document.addEventListener('DOMContentLoaded', () => {
            updateRecordButtonState(); // Set initial button text based on state
            updateStatus(); // Initial status fetch
            // Start periodic status updates
            statusUpdateInterval = setInterval(updateStatus, 5000); // Update every 5 seconds
        });

        // Clean up interval when the page is unloaded
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
    """Generates JPEG frames for the MJPEG stream from the single camera."""
    global _camera_manager # Access the manager instance
    frame_counter = 0
    last_frame_time = time.monotonic()
    logging.info("MJPEG stream client connected. Starting frame generation.")

    if not _camera_manager:
        logging.error("Stream: CameraManager not initialized!")
        # Yield a placeholder or error frame? For now, just exit.
        return

    while True:
        # Check shutdown status frequently
        shutdown_event = _app_state.get("shutdown_event")
        if shutdown_event and shutdown_event.is_set():
            logging.info("Stream generator: Shutdown detected, stopping.")
            break

        # Get the latest frame from the camera manager (already handles single camera)
        frame_to_encode = _camera_manager.get_latest_frame()

        if frame_to_encode is None:
            # logging.debug("Stream generator: No frame available yet.")
            time.sleep(0.05) # Wait briefly if no frame
            continue

        try:
            # Encode the frame as JPEG
            # Adjust JPEG quality (lower for less bandwidth, higher for better quality)
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 75]
            (flag, encodedImage) = cv2.imencode(".jpg", frame_to_encode, encode_param)
            if not flag:
                logging.warning("Stream generator: Could not encode frame to JPEG.")
                time.sleep(0.1) # Wait a bit longer after encoding failure
                continue

            # Yield the frame in MJPEG format
            yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' +
                   bytearray(encodedImage) + b'\r\n')

            frame_counter += 1

            # --- Rate limiting (Simple) ---
            # Aim slightly faster than camera FPS to avoid falling behind,
            # but also cap to prevent overwhelming client/network.
            # This is approximate and depends on capture/encode speed.
            current_time = time.monotonic()
            elapsed = current_time - last_frame_time
            # Get target FPS from config as a baseline, default to 30 if unavailable
            try:
                 _, _, target_fps = _camera_manager.get_resolution_config()
                 if target_fps <= 0: target_fps = 30.0
            except:
                 target_fps = 30.0

            # Calculate delay, ensuring it's not excessively small or large
            target_delay = 1.0 / (target_fps + 5) # Aim slightly faster than target
            target_delay = max(0.01, min(target_delay, 0.1)) # Cap delay between 10ms and 100ms (10-100 FPS limit)

            sleep_time = max(0, target_delay - elapsed) # Ensure non-negative sleep
            time.sleep(sleep_time)
            last_frame_time = time.monotonic()


        except GeneratorExit:
            # Client disconnected
            logging.info(f"Streaming client disconnected after {frame_counter} frames.")
            break
        except Exception as e:
            # Log other errors (e.g., encoding issues)
            logging.exception(f"!!! Error in MJPEG streaming generator: {e}")
            time.sleep(0.5) # Pause briefly after an error

    logging.info("Stream generator thread exiting.")


def build_options_html(available_modes, current_mode_name):
    """Helper to build <option> tags for HTML select dropdowns."""
    options_html = ""
    for mode_name in available_modes:
        selected_attr = ' selected' if mode_name == current_mode_name else ''
        options_html += f'<option value="{mode_name}"{selected_attr}>{mode_name}</option>'
    return options_html

# ===========================================================
# === Flask Routes ===
# ===========================================================

@app.route("/")
def index():
    """Renders the main HTML page."""
    global _camera_manager, _hardware_manager, _app_state

    if not _camera_manager or not _hardware_manager:
         # Handle case where managers aren't initialized yet
         logging.error("Web UI: Managers not initialized when rendering index.")
         return "Error: Application not fully initialized.", 500

    # Get initial state from managers and app state
    cam_state = _camera_manager.get_camera_state()
    hw_state = { # Get relevant hardware state
        'battery_percent': _hardware_manager.battery_percentage,
        'last_error': _hardware_manager.last_error
    }
    with _ui_lock:
        app_state_copy = _app_state.copy() # Get UI specific state

    # Combine error messages (prioritize camera errors?)
    err_msg = cam_state.get('last_error') or hw_state.get('last_error') or ""

    # Get resolution from camera state (using refactored keys)
    current_w, current_h = cam_state.get('resolution_wh', (640, 480)) # Default fallback
    resolution_text = f"{current_w}x{current_h}"
    batt_perc_initial = hw_state.get('battery_percent')
    batt_text_initial = f"{batt_perc_initial:.1f}" if batt_perc_initial is not None else "--"

    # Build HTML option strings using config lists and current state from CameraManager
    # Use refactored state keys
    awb_options_html = build_options_html(config.AVAILABLE_AWB_MODES, cam_state.get('awb_mode', config.DEFAULT_AWB_MODE_NAME))
    ae_options_html = build_options_html(config.AVAILABLE_AE_MODES, cam_state.get('ae_mode', config.DEFAULT_AE_MODE_NAME))
    metering_options_html = build_options_html(config.AVAILABLE_METERING_MODES, cam_state.get('metering_mode', config.DEFAULT_METERING_MODE_NAME))
    noise_reduction_options_html = build_options_html(config.AVAILABLE_NOISE_REDUCTION_MODES, cam_state.get('noise_reduction_mode', config.DEFAULT_NOISE_REDUCTION_MODE_NAME))

    # Render the template string
    return render_template_string(HTML_TEMPLATE,
        resolution_text=resolution_text,
        current_w=current_w, # Pass dimensions for the img tag
        current_h=current_h,
        err_msg=err_msg,
        digital_rec_state_initial=app_state_copy.get('digital_recording_active', False),
        batt_text_initial=batt_text_initial,
        # Pass current control values from CameraManager state (using refactored keys)
        current_awb_mode_name_initial=cam_state.get('awb_mode', config.DEFAULT_AWB_MODE_NAME),
        awb_options_html=awb_options_html,
        current_ae_mode_name_initial=cam_state.get('ae_mode', config.DEFAULT_AE_MODE_NAME),
        ae_options_html=ae_options_html,
        current_metering_mode_name_initial=cam_state.get('metering_mode', config.DEFAULT_METERING_MODE_NAME),
        metering_options_html=metering_options_html,
        current_noise_reduction_mode_name_initial=cam_state.get('noise_reduction_mode', config.DEFAULT_NOISE_REDUCTION_MODE_NAME),
        noise_reduction_options_html=noise_reduction_options_html,
        brightness_initial=cam_state.get('brightness', config.DEFAULT_BRIGHTNESS),
        MIN_BRIGHTNESS=config.MIN_BRIGHTNESS, MAX_BRIGHTNESS=config.MAX_BRIGHTNESS, STEP_BRIGHTNESS=config.STEP_BRIGHTNESS,
        contrast_initial=cam_state.get('contrast', config.DEFAULT_CONTRAST),
        MIN_CONTRAST=config.MIN_CONTRAST, MAX_CONTRAST=config.MAX_CONTRAST, STEP_CONTRAST=config.STEP_CONTRAST,
        saturation_initial=cam_state.get('saturation', config.DEFAULT_SATURATION),
        MIN_SATURATION=config.MIN_SATURATION, MAX_SATURATION=config.MAX_SATURATION, STEP_SATURATION=config.STEP_SATURATION,
        sharpness_initial=cam_state.get('sharpness', config.DEFAULT_SHARPNESS),
        MIN_SHARPNESS=config.MIN_SHARPNESS, MAX_SHARPNESS=config.MAX_SHARPNESS, STEP_SHARPNESS=config.STEP_SHARPNESS,
        video_feed_url=url_for('video_feed') # Generate URL dynamically
    )


@app.route("/video_feed")
def video_feed():
    """Route for the MJPEG video stream."""
    # generate_stream_frames now handles the single camera stream
    return Response(generate_stream_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route("/status")
def status():
    """Returns the current status of the application as JSON."""
    global _camera_manager, _hardware_manager, _app_state

    if not _camera_manager or not _hardware_manager:
        return jsonify({'error': 'Application not fully initialized.'}), 500

    # Get current state from managers
    cam_state = _camera_manager.get_camera_state()
    # Get latest battery reading (don't trigger a new read here)
    batt_perc = _hardware_manager.battery_percentage

    # Combine error messages
    err_msg = cam_state.get('last_error') or _hardware_manager.last_error or ""

    # Auto-clear certain errors if conditions are met (e.g., frames are flowing)
    latest_frame = _camera_manager.get_latest_frame() # Check if frames are coming
    if latest_frame is not None and err_msg and ("Init Error" in err_msg or "unavailable" in err_msg or "capture" in err_msg or "Capture Error" in err_msg):
         logging.info("Status: Auto-clearing previous camera/capture error as frames are being received.")
         if _camera_manager.last_error == err_msg: _camera_manager.last_error = None
         if _hardware_manager.last_error == err_msg: _hardware_manager.last_error = None
         err_msg = "" # Clear for current response
    if batt_perc is not None and err_msg and ("Battery Monitor" in err_msg or "INA219" in err_msg or "I2C Error" in err_msg):
         logging.info("Status: Auto-clearing previous battery monitor error as a reading was successful.")
         if _hardware_manager.last_error == err_msg: _hardware_manager.last_error = None
         # Camera manager shouldn't have battery errors, but check just in case
         if _camera_manager.last_error == err_msg: _camera_manager.last_error = None
         err_msg = ""

    # Build status text
    status_text = "Streaming"
    if cam_state.get('is_recording'):
        rec_count = len(cam_state.get('recording_paths', []))
        if rec_count > 0:
            status_text += f" (Recording to {rec_count} USB(s))"
        else:
             # This indicates an inconsistent state
             status_text += " (ERROR: Recording active but no paths!)"
             if not err_msg: err_msg = "Inconsistent State: Recording active but no paths."

    # Get UI-specific state under lock
    with _ui_lock:
        digital_rec_active = _app_state.get("digital_recording_active", False)

    # Return combined status using refactored state keys
    status_data = {
        'is_recording': cam_state.get('is_recording', False),
        'digital_recording_active': digital_rec_active,
        # Use resolution_wh tuple directly
        'resolution': f"{cam_state.get('resolution_wh', ['?','?'])[0]}x{cam_state.get('resolution_wh', ['?','?'])[1]}",
        'status_text': status_text,
        'error': err_msg,
        'active_recordings': cam_state.get('recording_paths', []),
        'battery_percent': batt_perc,
        # Include current control values from CameraManager state (using refactored keys)
        'awb_mode': cam_state.get('awb_mode'),
        'ae_mode': cam_state.get('ae_mode'),
        'metering_mode': cam_state.get('metering_mode'),
        'noise_reduction_mode': cam_state.get('noise_reduction_mode'),
        'brightness': cam_state.get('brightness'),
        'contrast': cam_state.get('contrast'),
        'saturation': cam_state.get('saturation'),
        'sharpness': cam_state.get('sharpness'),
    }
    return jsonify(status_data)


@app.route("/set_resolution/<direction>", methods=['POST'])
def set_resolution(direction):
    """Handles requests to change the camera resolution."""
    global _camera_manager, _app_state

    if not _camera_manager: return jsonify({'success': False, 'message': 'Camera manager not ready.'}), 500

    with _ui_lock: # Lock access to the request index
        if _app_state.get("reconfigure_resolution_index") is not None:
            # Prevent concurrent requests
            return jsonify({'success': False, 'message': 'Reconfiguration already in progress.'}), 429

        # Use refactored attribute name
        current_index = _camera_manager.current_resolution_index
        original_index = current_index
        new_index = current_index

        if direction == 'up': new_index += 1
        elif direction == 'down': new_index -= 1
        else:
            return jsonify({'success': False, 'message': 'Invalid direction specified.'}), 400

        # Clamp index within valid range using refactored config constant
        new_index = max(0, min(len(config.CAMERA_RESOLUTIONS) - 1, new_index))

        if new_index == original_index:
            # Already at the limit
            msg = 'Already at highest resolution.' if direction == 'up' else 'Already at lowest resolution.'
            return jsonify({'success': False, 'message': msg}), 400

        # Store the requested index for the main loop to pick up
        _app_state["reconfigure_resolution_index"] = new_index
        # Get new resolution details using refactored config constant
        new_w, new_h, _ = config.CAMERA_RESOLUTIONS[new_index]
        logging.info(f"Web request: Queue resolution change index {original_index} -> {new_index} ({new_w}x{new_h})")
        # Clear camera manager's last error on successful request queuing
        _camera_manager.last_error = None

        # Return success and the new resolution string for the UI
        return jsonify({'success': True, 'message': 'Resolution change requested.', 'new_resolution': f"{new_w}x{new_h}"})


@app.route('/toggle_recording', methods=['POST'])
def toggle_recording():
    """Toggles the digital recording trigger state."""
    global _app_state
    new_state = False
    with _ui_lock:
        current_state = _app_state.get("digital_recording_active", False)
        _app_state["digital_recording_active"] = not current_state
        new_state = _app_state["digital_recording_active"]
        logging.info(f"Digital recording trigger toggled via web UI to: {'ON' if new_state else 'OFF'}")
        # Clear potential previous recording errors when user interacts
        if _camera_manager and _camera_manager.last_error and ("Recording" in _camera_manager.last_error or "writers" in _camera_manager.last_error or "USB" in _camera_manager.last_error or "sync" in _camera_manager.last_error or "Mux" in _camera_manager.last_error):
            logging.info(f"Clearing previous recording/muxing error via toggle: '{_camera_manager.last_error}'")
            _camera_manager.last_error = None
        if _camera_manager and _camera_manager.audio_last_error: # Clear audio errors too
             logging.info(f"Clearing previous audio error via toggle: '{_camera_manager.audio_last_error}'")
             _camera_manager.audio_last_error = None
        if _hardware_manager and _hardware_manager.last_error and ("Recording" in _hardware_manager.last_error): # Less likely
             _hardware_manager.last_error = None

    return jsonify({'success': True, 'digital_recording_active': new_state})


@app.route('/set_camera_control', methods=['POST'])
def set_camera_control():
    """Sets a specific camera control value."""
    global _camera_manager

    if not _camera_manager: return jsonify({'success': False, 'message': 'Camera manager not ready.'}), 500
    # Use refactored attribute
    if not _camera_manager.is_initialized: return jsonify({'success': False, 'message': 'Camera not initialized.'}), 503

    control_name = None # Initialize for error logging
    try:
        data = request.get_json()
        if not data or 'control' not in data or 'value' not in data:
            logging.warning("/set_camera_control called without 'control' or 'value'.")
            return jsonify({'success': False, 'message': 'Missing control or value in request.'}), 400

        control_name = data['control']
        control_value = data['value']
        logging.info(f"Web request: Set control '{control_name}' to '{control_value}'")

        control_dict_to_set = {}
        parsed_value = None
        validation_error = None

        # --- Validate and Parse Value ---
        # Mode Controls (convert string name to enum member)
        if control_name == 'AwbMode':
            enum_val = controls.AwbModeEnum.__members__.get(control_value)
            if enum_val is not None: parsed_value = enum_val; control_dict_to_set["AwbEnable"] = True # Ensure enabled
            else: validation_error = f"Invalid AwbMode value: {control_value}"
        elif control_name == 'AeExposureMode':
            enum_val = controls.AeExposureModeEnum.__members__.get(control_value)
            if enum_val is not None: parsed_value = enum_val; control_dict_to_set["AeEnable"] = True
            else: validation_error = f"Invalid AeExposureMode value: {control_value}"
        elif control_name == 'AeMeteringMode':
            enum_val = controls.AeMeteringModeEnum.__members__.get(control_value)
            if enum_val is not None: parsed_value = enum_val; control_dict_to_set["AeEnable"] = True
            else: validation_error = f"Invalid AeMeteringMode value: {control_value}"
        elif control_name == 'NoiseReductionMode':
            try: # Handle potential absence of draft controls
                enum_val = controls.draft.NoiseReductionModeEnum.__members__.get(control_value)
                if enum_val is not None: parsed_value = enum_val
                else: validation_error = f"Invalid NoiseReductionMode value: {control_value}"
            except AttributeError:
                validation_error = "NoiseReductionMode control not available."

        # Numeric Controls (convert to float and clamp)
        elif control_name == 'Brightness':
            try:
                val = float(control_value)
                parsed_value = max(config.MIN_BRIGHTNESS, min(config.MAX_BRIGHTNESS, val))
            except ValueError: validation_error = "Invalid numeric value for Brightness"
        elif control_name == 'Contrast':
            try:
                val = float(control_value)
                parsed_value = max(config.MIN_CONTRAST, min(config.MAX_CONTRAST, val))
            except ValueError: validation_error = "Invalid numeric value for Contrast"
        elif control_name == 'Saturation':
            try:
                val = float(control_value)
                parsed_value = max(config.MIN_SATURATION, min(config.MAX_SATURATION, val))
            except ValueError: validation_error = "Invalid numeric value for Saturation"
        elif control_name == 'Sharpness':
            try:
                val = float(control_value)
                parsed_value = max(config.MIN_SHARPNESS, min(config.MAX_SHARPNESS, val))
            except ValueError: validation_error = "Invalid numeric value for Sharpness"
        else:
            validation_error = f"Unknown control name: {control_name}"

        # --- Handle Validation Result ---
        if validation_error:
            logging.error(f"Control validation failed for '{control_name}': {validation_error}")
            return jsonify({'success': False, 'message': validation_error}), 400

        # --- Apply Control ---
        if parsed_value is not None:
            control_dict_to_set[control_name] = parsed_value
            # Call refactored method in camera manager
            if _camera_manager.apply_camera_controls(control_dict_to_set):
                 # Value applied successfully by camera manager
                 logged_value = f"{parsed_value:.2f}" if isinstance(parsed_value, float) else parsed_value.name if hasattr(parsed_value,'name') else str(parsed_value)
                 logging.info(f"Successfully set {control_name} to: {logged_value}")
                 return jsonify({'success': True, 'message': f'{control_name} set.'})
            else:
                 # apply_camera_controls failed, error should be in manager's last_error
                 error_msg = _camera_manager.last_error or f"Failed to apply {control_name}"
                 return jsonify({'success': False, 'message': error_msg}), 500
        else:
             # Should have been caught by validation_error check, but as a fallback
             logging.error(f"Internal error: Parsed value is None for control '{control_name}' after validation.")
             return jsonify({'success': False, 'message': 'Internal processing error.'}), 500

    except Exception as e:
        # Catch-all for unexpected errors during request processing
        logging.error(f"Error processing set_camera_control '{control_name}': {e}", exc_info=True)
        return jsonify({'success': False, 'message': f'Unexpected server error: {e}'}), 500


@app.route('/power_down', methods=['POST'])
def power_down():
    """Signals the main application to initiate shutdown and reboot."""
    global _app_state
    logging.warning("Received request for power down via web UI. Setting flags.")

    with _ui_lock:
        _app_state["reboot_requested"] = True
        # Set the shutdown event if it exists
        shutdown_event = _app_state.get("shutdown_event")
        if shutdown_event:
            shutdown_event.set() # Signal main loop to stop
        else:
            logging.error("Power down requested, but shutdown_event not set in app_state!")
            return jsonify({'success': False, 'message': 'Internal server error: Shutdown event not configured.'}), 500

    # Update manager errors to reflect shutdown trigger
    if _camera_manager: _camera_manager.last_error = "Shutdown initiated via web UI..."
    if _hardware_manager: _hardware_manager.last_error = "Shutdown initiated via web UI..."

    return jsonify({'success': True, 'message': 'Shutdown initiated. System will reboot after cleanup.'})

# ===========================================================
# === Web Server Setup Function ===
# ===========================================================

def setup_web_app(camera_manager, hardware_manager, shutdown_event_ref):
    """
    Sets up the Flask application context with manager instances and shared event.

    Args:
        camera_manager (CameraManager): Instance of the camera manager.
        hardware_manager (HardwareManager): Instance of the hardware manager.
        shutdown_event_ref (threading.Event): Reference to the main shutdown event.
    """
    global _camera_manager, _hardware_manager, _app_state
    _camera_manager = camera_manager
    _hardware_manager = hardware_manager
    with _ui_lock:
        _app_state["shutdown_event"] = shutdown_event_ref
        # Reset other UI state flags on setup
        _app_state["digital_recording_active"] = False
        _app_state["reconfigure_resolution_index"] = None
        _app_state["reboot_requested"] = False

    logging.info("Web application context configured with manager instances.")


def run_web_server():
    """Runs the Flask web server."""
    logging.info(f"Starting Flask web server on 0.0.0.0:{config.WEB_PORT}...")
    # Use 'threaded=True' for handling multiple clients (stream + status requests)
    # 'debug=False' and 'use_reloader=False' are important for stability when run as a service
    try:
        # Use waitress or gunicorn in production for better performance/stability
        # For simplicity here, using Flask's built-in server
        app.run(host='0.0.0.0', port=config.WEB_PORT, debug=False, use_reloader=False, threaded=True)
    except Exception as e:
        logging.exception(f"!!! Flask web server failed: {e}")
        # Signal shutdown if Flask fails catastrophically
        with _ui_lock:
            shutdown_event = _app_state.get("shutdown_event")
            if shutdown_event and not shutdown_event.is_set():
                logging.error("Signaling main thread shutdown due to Flask error.")
                shutdown_event.set()

# Example Usage (for testing purposes - requires dummy managers)
if __name__ == "__main__":
    # --- Create Dummy Managers for testing ---
    class DummyManager:
        def __init__(self):
            self.last_error = None
            self.audio_last_error = None
            self.current_resolution_index = config.DEFAULT_RESOLUTION_INDEX
            self.is_initialized = True
        def get_latest_frame(self):
            # Create a simple placeholder frame
            img = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(img,'No Frame',(50,240),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),2)
            return img
        def get_resolution_config(self):
             return config.CAMERA_RESOLUTIONS[self.current_resolution_index]
        def get_camera_state(self):
             w, h, fps = self.get_resolution_config()
             return {
                 'is_initialized': self.is_initialized,
                 'resolution_index': self.current_resolution_index,
                 'resolution_wh': (w, h),
                 'output_frame_wh': (w, h),
                 'target_fps': fps,
                 'actual_fps': fps,
                 'measured_fps': fps,
                 'is_recording': False,
                 'recording_paths': [],
                 'last_error': self.last_error,
                 'audio_last_error': self.audio_last_error,
                 'awb_mode': config.DEFAULT_AWB_MODE_NAME,
                 'ae_mode': config.DEFAULT_AE_MODE_NAME,
                 'metering_mode': config.DEFAULT_METERING_MODE_NAME,
                 'noise_reduction_mode': config.DEFAULT_NOISE_REDUCTION_MODE_NAME,
                 'brightness': config.DEFAULT_BRIGHTNESS,
                 'contrast': config.DEFAULT_CONTRAST,
                 'saturation': config.DEFAULT_SATURATION,
                 'sharpness': config.DEFAULT_SHARPNESS,
                 'analogue_gain': config.DEFAULT_ANALOGUE_GAIN,
                 'iso_mode': config.DEFAULT_ISO_NAME
             }
        def apply_camera_controls(self, d): print(f"DummyCam: Apply controls {d}"); return True
        def initialize_camera(self, idx=None):
             print(f"DummyCam: Init camera {idx if idx is not None else self.current_resolution_index}")
             if idx is not None: self.current_resolution_index = idx
             return True

    class DummyHWManager:
         battery_percentage = 75.3
         last_error = None

    # --- Setup Logging ---
    logging.basicConfig(level=config.LOG_LEVEL, format=config.LOG_FORMAT, datefmt=config.LOG_DATE_FORMAT)
    logging.getLogger("werkzeug").setLevel(logging.WARNING) # Quieten Flask's default logging

    # --- Setup and Run ---
    print("--- Web UI Test (Single Cam Refactor) ---")
    print(f"--- Running on http://0.0.0.0:{config.WEB_PORT} ---")
    dummy_shutdown = threading.Event()
    setup_web_app(DummyManager(), DummyHWManager(), dummy_shutdown)
    run_web_server() # This will block until Ctrl+C
    print("--- Web UI Test Complete ---")

