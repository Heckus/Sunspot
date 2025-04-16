# -*- coding: utf-8 -*-
"""
web_ui.py

Handles the Flask web server and user interface for the Pi Camera application.
Provides routes for streaming (combined view), status updates, and control commands
(ISO, AE, Metering, Sliders, Servo).
"""

import logging
import time
import cv2
import threading # Added for Lock
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
    "reconfigure_resolution_index": None, # Tracks requests from UI for Cam0
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
    <title>Pi Multi-Camera Stream & Record</title>
    {# Raw block to prevent Jinja processing CSS/JS #}
    {% raw %}
    <style>
        body { font-family: sans-serif; line-height: 1.4; margin: 1em; background-color: #f0f0f0;}
        .container { max-width: 960px; margin: auto; background: #fff; padding: 15px; border-radius: 8px; box-shadow: 0 2px 5px rgba(0,0,0,0.1); }
        h1 { text-align: center; color: #333; margin-bottom: 10px; }

        .grid-container { display: grid; grid-template-columns: repeat(auto-fit, minmax(280px, 1fr)); gap: 20px; margin-bottom: 15px; }

        .status-panel, .controls-panel, .sliders-panel, .servo-panel { background-color: #eef; padding: 15px; border-radius: 5px; }
        .panel-title { font-weight: bold; margin-bottom: 10px; border-bottom: 1px solid #ccc; padding-bottom: 5px; }

        /* Status Grid */
        .status-grid { display: grid; grid-template-columns: auto 1fr; gap: 5px 10px; align-items: center; }
        .status-grid span:first-child { font-weight: bold; color: #555; text-align: right;}
        #status, #rec-status, #resolution, #battery-level,
        #iso-mode-status, #ae-mode-status, #metering-mode-status, #nr-mode-status, /* Status IDs */
        #servo-angle-status /* Servo status */
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
        /* Remove explicit width/height, let CSS handle max-width */
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
        <h1>Pi Multi-Camera Stream & Record</h1>

         <div class="main-controls">
             <button onclick="changeResolution('down')" id="btn-down" title="Decrease Cam0 resolution">&laquo; Lower Res</button>
             <button onclick="toggleRecording()" id="btn-record" class="recording-inactive" title="Toggle Cam0 recording via web interface">Start Rec (Web)</button>
             <button onclick="changeResolution('up')" id="btn-up" title="Increase Cam0 resolution">Higher Res &raquo;</button>
             <button onclick="powerDown()" id="btn-powerdown" title="Gracefully stop service and reboot Pi">Power Down</button>
         </div>

         <div class="grid-container">
             <div class="status-panel">
                 <div class="panel-title">Current Status</div>
                 <div class="status-grid">
                     <span>Sys Status:</span> <span id="status">Initializing...</span>
                     <span>Recording:</span> <span id="rec-status">OFF</span>
                     <span>Cam0 Res:</span> <span id="resolution">{{ resolution_text }}</span>
                     <span>Battery:</span> <span id="battery-level">{{ batt_text_initial }}%</span>
                     <hr style="grid-column: 1 / -1; border-top: 1px dashed #bbb; border-bottom: none; margin: 5px 0;">
                     <span>ISO Mode:</span> <span id="iso-mode-status">{{ current_iso_mode_name_initial }}</span>
                     <span>AE Mode:</span> <span id="ae-mode-status">{{ current_ae_mode_name_initial }}</span>
                     <span>Metering:</span> <span id="metering-mode-status">{{ current_metering_mode_name_initial }}</span>
                     <span>Noise Red.:</span> <span id="nr-mode-status">{{ current_noise_reduction_mode_name_initial }}</span>
                     <span>Servo Angle:</span> <span id="servo-angle-status">{{ servo_angle_initial }}&deg;</span>
                 </div>
             </div>

             <div class="controls-panel">
                 <div class="panel-title">Mode Controls</div>
                 <div class="mode-controls">
                     <label for="iso-select">ISO Mode:</label>
                     <select id="iso-select" onchange="changeCameraControl('AnalogueGain', this.value)" title="Select ISO Sensitivity (Analogue Gain)">
                         {{ iso_options_html | safe }} {# ISO options #}
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

             <div class="servo-panel"> {# New panel for Servo #}
                 <div class="panel-title">Servo Control</div>
                 <div class="slider-controls">
                     <label for="servo-slider">Angle:</label>
                     <input type="range" id="servo-slider" min="{{ SERVO_MIN_ANGLE }}" max="{{ SERVO_MAX_ANGLE }}" step="1" value="{{ servo_angle_initial }}" oninput="updateSliderValue(this.id, this.value)" onchange="changeServoAngle(this.value)" title="Adjust Servo Angle">
                     <span id="servo-slider-value">{{ servo_angle_initial }}&deg;</span> {# Display servo angle #}
                 </div>
             </div>

         </div>
         <div id="error" {% if err_msg %}style="display: block;"{% endif %}>{{ err_msg }}</div>
         {# Removed explicit width/height #}
         <img id="stream" src="{{ video_feed_url }}" alt="Loading stream..."
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
        // AWB removed
        const isoStatusElement = document.getElementById('iso-mode-status'); // New
        const aeStatusElement = document.getElementById('ae-mode-status');
        const meteringStatusElement = document.getElementById('metering-mode-status');
        const nrStatusElement = document.getElementById('nr-mode-status');
        // AWB removed
        const isoSelectElement = document.getElementById('iso-select'); // New
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
        // Servo elements
        const servoSlider = document.getElementById('servo-slider');
        const servoValueSpan = document.getElementById('servo-slider-value');
        const servoAngleStatus = document.getElementById('servo-angle-status'); // Status display

        // State Variables
        let isChangingResolution = false;
        let isTogglingRecording = false;
        let isChangingControl = false; // Used for camera controls AND servo
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
            if (isChangingResolution || isTogglingRecording || isChangingControl || isPoweringDown) return;
            fetch('/status')
                .then(response => { if (!response.ok) { throw new Error(`HTTP error! Status: ${response.status}`); } return response.json(); })
                .then(data => {
                    statusElement.textContent = data.status_text || 'Unknown';
                    recStatusElement.textContent = data.is_recording ? "ACTIVE" : "OFF";
                    recStatusElement.classList.toggle('active', data.is_recording);
                    if (data.resolution && resolutionElement.textContent !== data.resolution) {
                        resolutionElement.textContent = data.resolution;
                        // No need to update img width/height here anymore
                    }
                    // Combine errors
                    let combinedError = data.error || "";
                    if (data.audio_last_error) {
                         combinedError += (combinedError ? " | " : "") + "Audio: " + data.audio_last_error;
                    }
                    if (combinedError) { errorElement.textContent = combinedError; errorElement.style.display = 'block'; }
                    else { if (errorElement.style.display !== 'none') { errorElement.textContent = ''; errorElement.style.display = 'none'; } }

                    if (typeof data.digital_recording_active === 'boolean' && currentDigitalRecordState !== data.digital_recording_active) {
                        currentDigitalRecordState = data.digital_recording_active;
                        updateRecordButtonState();
                    }

                    // Update Camera Controls
                    updateControlUI('iso_mode', data.iso_mode, isoStatusElement, isoSelectElement); // Use iso_mode (name)
                    updateControlUI('ae_mode', data.ae_mode, aeStatusElement, aeSelectElement);
                    updateControlUI('metering_mode', data.metering_mode, meteringStatusElement, meteringSelectElement);
                    updateControlUI('noise_reduction_mode', data.noise_reduction_mode, nrStatusElement, nrSelectElement);
                    updateControlUI('brightness', data.brightness, null, brightnessSlider, brightnessValueSpan);
                    updateControlUI('contrast', data.contrast, null, contrastSlider, contrastValueSpan);
                    updateControlUI('saturation', data.saturation, null, saturationSlider, saturationValueSpan);
                    updateControlUI('sharpness', data.sharpness, null, sharpnessSlider, sharpnessValueSpan);

                    // Update Servo Control
                    updateControlUI('servo_angle', data.servo_angle, servoAngleStatus, servoSlider, servoValueSpan);

                    // Update Battery
                    if (data.battery_percent !== null && data.battery_percent !== undefined) {
                         batteryLevelElement.textContent = data.battery_percent.toFixed(1) + '%'; // Add % sign
                    } else {
                         batteryLevelElement.textContent = "--%";
                    }
                })
                .catch(err => {
                    console.error("Error fetching status:", err); statusElement.textContent = "Error"; errorElement.textContent = `Status fetch failed: ${err.message}.`; errorElement.style.display = 'block'; recStatusElement.textContent = "Err"; batteryLevelElement.textContent = "Err";
                    isoStatusElement.textContent = "Err"; aeStatusElement.textContent = "Err"; meteringStatusElement.textContent = "Err"; nrStatusElement.textContent = "Err"; servoAngleStatus.textContent = "Err";
                });
        }

        function updateControlUI(controlKey, newValue, statusEl, controlEl, valueSpanEl = null) {
             if (newValue === undefined || newValue === null) return;

             let currentUIValue = controlEl ? controlEl.value : null;
             let formattedNewValue = newValue; // Use as string by default

             if (controlKey === 'brightness' || controlKey === 'contrast' || controlKey === 'saturation' || controlKey === 'sharpness') {
                 // Handle float sliders
                 formattedNewValue = parseFloat(newValue).toFixed(1);
                 currentUIValue = controlEl ? parseFloat(controlEl.value).toFixed(1) : null;
                 if (valueSpanEl && valueSpanEl.textContent !== formattedNewValue) {
                     valueSpanEl.textContent = formattedNewValue;
                 }
             } else if (controlKey === 'servo_angle') {
                 // Handle integer servo slider
                 formattedNewValue = parseInt(newValue).toString(); // Ensure it's a string for comparison
                 currentUIValue = controlEl ? parseInt(controlEl.value).toString() : null;
                 if (valueSpanEl && valueSpanEl.textContent !== formattedNewValue + '°') {
                     valueSpanEl.textContent = formattedNewValue + '°';
                 }
                 if (statusEl && statusEl.textContent !== formattedNewValue + '°') { // Update status text too
                     statusEl.textContent = formattedNewValue + '°';
                 }
             } else { // Handle dropdowns (ISO, AE, Metering, NR)
                 formattedNewValue = newValue.toString(); // Already a string name
                 currentUIValue = controlEl ? controlEl.value : null;
                 if (statusEl && statusEl.textContent !== formattedNewValue) {
                     statusEl.textContent = formattedNewValue;
                 }
             }

             // Update control element (select or range) if not currently being changed by user
             // Check the 'isChangingControl' flag to prevent overwriting user input
             if (controlEl && currentUIValue !== formattedNewValue && !isChangingControl) {
                 console.log(`Status update forcing UI for ${controlKey}: UI='${currentUIValue}' -> Status='${formattedNewValue}'`);
                 controlEl.value = newValue; // Use the original newValue for setting element value
             }
         }


        function disableControls(poweringDown = false) {
            [btnUp, btnDown, btnRecord, btnPowerdown,
             isoSelectElement, aeSelectElement, meteringSelectElement, nrSelectElement, // ISO instead of AWB
             brightnessSlider, contrastSlider, saturationSlider, sharpnessSlider,
             servoSlider // Add servo slider
            ].forEach(el => { if(el) el.disabled = true; }); // Check if element exists
            if(poweringDown) { document.body.style.opacity = '0.7'; }
        }

        function enableControls() {
             if (!isPoweringDown) {
                 [btnUp, btnDown, btnRecord, btnPowerdown,
                  isoSelectElement, aeSelectElement, meteringSelectElement, nrSelectElement, // ISO instead of AWB
                  brightnessSlider, contrastSlider, saturationSlider, sharpnessSlider,
                  servoSlider // Add servo slider
                 ].forEach(el => { if(el) el.disabled = false; }); // Check if element exists
                 document.body.style.opacity = '1';
             }
         }

        // --- Action Functions ---
        function changeResolution(direction) {
            if (isChangingResolution || isTogglingRecording || isChangingControl || isPoweringDown) return;

            isChangingResolution = true;
            disableControls();
            statusElement.textContent = 'Changing Cam0 resolution... Please wait.';
            errorElement.textContent = '';
            errorElement.style.display = 'none';

            const resolutionTimeoutId = setTimeout(() => {
                 console.warn("Resolution change timeout reached. Forcing UI cleanup.");
                 if (isChangingResolution) {
                     isChangingResolution = false; enableControls(); updateStatus();
                 }
             }, 15000); // Increased timeout further for dual cam init

            fetch(`/set_resolution/${direction}`, { method: 'POST' })
                .then(response => response.json().then(data => ({ status: response.status, body: data })))
                .then(({ status, body }) => {
                    clearTimeout(resolutionTimeoutId);
                    if (status === 200 && body.success) {
                        statusElement.textContent = 'Resolution change initiated. Reloading stream...';
                        resolutionElement.textContent = body.new_resolution;
                        // No need to set img width/height here
                        console.log("Resolution change request successful, forcing stream reload...");
                        setTimeout(() => {
                            streamImage.src = videoFeedUrlBase + "?" + Date.now();
                            isChangingResolution = false;
                            enableControls();
                            updateStatus();
                        }, 2000); // Slightly longer delay for dual cam

                    } else {
                        errorElement.textContent = `Error changing resolution: ${body.message || 'Unknown error.'}`;
                        errorElement.style.display = 'block'; statusElement.textContent = 'Resolution change failed.';
                        isChangingResolution = false; enableControls(); updateStatus();
                    }
                })
                .catch(err => {
                     clearTimeout(resolutionTimeoutId);
                     console.error("Network error sending resolution change:", err);
                     errorElement.textContent = `Network error changing resolution: ${err.message}`;
                     errorElement.style.display = 'block'; statusElement.textContent = 'Resolution change failed (Network).';
                     isChangingResolution = false; enableControls(); updateStatus();
                 });
        }

        function toggleRecording() {
            if (isChangingResolution || isTogglingRecording || isChangingControl || isPoweringDown) return;
            isTogglingRecording = true; disableControls(); statusElement.textContent = 'Sending record command...'; errorElement.textContent = ''; errorElement.style.display = 'none';
             fetch('/toggle_recording', { method: 'POST' })
                 .then(response => { if (!response.ok) { throw new Error(`HTTP error! Status: ${response.status}`); } return response.json(); })
                 .then(data => {
                     if (data.success) {
                         currentDigitalRecordState = data.digital_recording_active; updateRecordButtonState(); statusElement.textContent = `Digital recording ${currentDigitalRecordState ? 'enabled' : 'disabled'}. State updating...`; setTimeout(updateStatus, 1500);
                     } else {
                         errorElement.textContent = `Error toggling recording: ${data.message || 'Unknown error.'}`; errorElement.style.display = 'block'; statusElement.textContent = 'Record command failed.'; setTimeout(updateStatus, 1000);
                     }
                 })
                 .catch(err => { console.error("Error toggling recording:", err); errorElement.textContent = `Network error toggling recording: ${err.message}`; errorElement.style.display = 'block'; statusElement.textContent = 'Command failed (Network).'; setTimeout(updateStatus, 1000); })
                 .finally(() => { isTogglingRecording = false; enableControls(); });
         }

        // Handles Camera controls (ISO, AE, Metering, Sliders)
        function changeCameraControl(controlName, controlValue) {
             if (isChangingResolution || isTogglingRecording || isChangingControl || isPoweringDown) return;
             // Note: For ISO, controlName='AnalogueGain', controlValue=ISO Name (e.g., "100")
             console.log(`Requesting camera control change: ${controlName} = ${controlValue}`);
             isChangingControl = true; disableControls();
             // Display user-friendly name if changing ISO
             let settingName = (controlName === 'AnalogueGain') ? `ISO to ${controlValue}` : controlName;
             statusElement.textContent = `Setting ${settingName}...`;
             errorElement.textContent = ''; errorElement.style.display = 'none';

             fetch('/set_camera_control', {
                 method: 'POST',
                 headers: { 'Content-Type': 'application/json' },
                 body: JSON.stringify({ control: controlName, value: controlValue }) // Send ISO name or slider value
             })
             .then(response => response.json().then(data => ({ status: response.status, body: data })))
             .then(({ status, body }) => {
                 if (status === 200 && body.success) {
                     statusElement.textContent = `${settingName} set.`;
                     // Update UI immediately based on success, then verify with status update
                     // For ISO, update using the name ('iso_mode')
                     let uiKey = (controlName === 'AnalogueGain') ? 'iso_mode' : controlName.toLowerCase();
                     let statusEl = document.getElementById(uiKey.replace('_','-') + '-status');
                     let controlEl = document.getElementById(uiKey.replace('_','-') + (uiKey.includes('mode') ? '-select' : '-slider'));
                     let valueSpanEl = document.getElementById(uiKey.replace('_','-') + '-slider-value');
                     updateControlUI(uiKey, controlValue, statusEl, controlEl, valueSpanEl);
                     setTimeout(updateStatus, 750); // Fetch status slightly later to confirm
                 } else {
                     errorElement.textContent = `Error setting ${settingName}: ${body.message || 'Unknown error.'}`; errorElement.style.display = 'block'; statusElement.textContent = `${settingName} change failed.`;
                     setTimeout(updateStatus, 500); // Fetch status to revert UI if needed
                 }
             })
             .catch(err => {
                 console.error(`Network error setting ${settingName}:`, err); errorElement.textContent = `Network error: ${err.message}`; errorElement.style.display = 'block'; statusElement.textContent = `${settingName} change failed (Network).`;
                 setTimeout(updateStatus, 500); // Fetch status to revert UI if needed
             })
             .finally(() => {
                 isChangingControl = false;
                 enableControls();
             });
         }

        // Handles Servo control
        function changeServoAngle(angle) {
             if (isChangingResolution || isTogglingRecording || isChangingControl || isPoweringDown) return;
             console.log(`Requesting servo angle change: ${angle}`);
             isChangingControl = true; disableControls(); // Use same flag for simplicity
             statusElement.textContent = `Setting Servo Angle to ${angle}°...`;
             errorElement.textContent = ''; errorElement.style.display = 'none';

             fetch('/set_servo_angle', { // New endpoint
                 method: 'POST',
                 headers: { 'Content-Type': 'application/json' },
                 body: JSON.stringify({ angle: parseInt(angle) }) // Send integer angle
             })
             .then(response => response.json().then(data => ({ status: response.status, body: data })))
             .then(({ status, body }) => {
                 if (status === 200 && body.success) {
                     statusElement.textContent = `Servo angle set to ${angle}°.`;
                     // Update UI immediately
                     updateControlUI('servo_angle', angle, servoAngleStatus, servoSlider, servoValueSpan);
                     setTimeout(updateStatus, 750); // Verify with status update
                 } else {
                     errorElement.textContent = `Error setting servo angle: ${body.message || 'Unknown error.'}`; errorElement.style.display = 'block'; statusElement.textContent = `Servo angle change failed.`;
                     setTimeout(updateStatus, 500); // Revert UI if needed
                 }
             })
             .catch(err => {
                 console.error(`Network error setting servo angle:`, err); errorElement.textContent = `Network error: ${err.message}`; errorElement.style.display = 'block'; statusElement.textContent = `Servo angle change failed (Network).`;
                 setTimeout(updateStatus, 500); // Revert UI if needed
             })
             .finally(() => {
                 isChangingControl = false;
                 enableControls();
             });
         }

        function updateSliderValue(sliderId, value) {
            const spanId = sliderId + '-value';
            const spanElement = document.getElementById(spanId);
            if (spanElement) {
                if (sliderId === 'servo-slider') {
                    spanElement.textContent = parseInt(value).toString() + '°'; // Integer for servo
                } else {
                    spanElement.textContent = parseFloat(value).toFixed(1); // Float for others
                }
            }
        }

        function powerDown() {
             if (isChangingResolution || isTogglingRecording || isChangingControl || isPoweringDown) return;
             if (!confirm("Are you sure you want to power down the Raspberry Pi?")) { return; }
             isPoweringDown = true; disableControls(true); statusElement.textContent = 'Powering down...'; errorElement.textContent = ''; errorElement.style.display = 'none';
             if (statusUpdateInterval) clearInterval(statusUpdateInterval);
             streamImage.onerror = null; streamImage.src = "";
             fetch('/power_down', { method: 'POST' })
                 .then(response => { if (!response.ok) { return response.json().then(data => { throw new Error(data.message || `HTTP error! Status: ${response.status}`); }).catch(() => { throw new Error(`HTTP error! Status: ${response.status}`); }); } return response.json(); })
                 .then(data => { if (data.success) { statusElement.textContent = 'Shutdown initiated. System will reboot shortly.'; document.body.innerHTML = "<h1>Shutting Down... Please Wait.</h1>"; } else { errorElement.textContent = `Shutdown request failed: ${data.message || 'Unknown error.'}`; errorElement.style.display = 'block'; statusElement.textContent = 'Shutdown failed.'; isPoweringDown = false; enableControls(); statusUpdateInterval = setInterval(updateStatus, 5000); } })
                 .catch(err => { console.error("Error sending power down command:", err); errorElement.textContent = `Error initiating shutdown: ${err.message}.`; errorElement.style.display = 'block'; statusElement.textContent = 'Shutdown error.'; isPoweringDown = false; enableControls(); statusUpdateInterval = setInterval(updateStatus, 5000); });
         }

        // --- Stream Handling ---
        function handleStreamError() {
            console.warn("Stream image 'onerror' event triggered.");
            if (streamErrorTimeout || isPoweringDown || isChangingResolution) return;
            statusElement.textContent = 'Stream interrupted. Attempting reload...';
            streamImage.src = "";
            streamErrorTimeout = setTimeout(() => {
                 console.log("Attempting stream reload...");
                 streamImage.src = videoFeedUrlBase + "?" + Date.now();
                 streamErrorTimeout = null;
             }, 3000);
         }

        function handleStreamLoad() {
             console.log("Stream image 'onload' event fired.");
             if (streamErrorTimeout) { clearTimeout(streamErrorTimeout); streamErrorTimeout = null; }
             if (isChangingResolution) {
                 console.log("Stream loaded after resolution change, enabling controls.");
                 isChangingResolution = false; enableControls(); updateStatus();
             }
         }

        // --- Initialization ---
        document.addEventListener('DOMContentLoaded', () => {
            updateRecordButtonState(); updateStatus();
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
    """Generates JPEG frames for the MJPEG stream from the combined camera view."""
    global _camera_manager # Access the manager instance
    frame_counter = 0
    last_frame_time = time.monotonic()
    logging.info("MJPEG stream client connected. Starting combined frame generation.")

    if not _camera_manager:
        logging.error("Stream: CameraManager not initialized!")
        # Yield a placeholder image or error message?
        # For now, just return, client will see broken image.
        return

    while True:
        if _app_state.get("shutdown_event") and _app_state["shutdown_event"].is_set():
            logging.info("Stream generator: Shutdown detected, stopping.")
            break

        # Get the latest COMBINED frame
        frame_to_encode = _camera_manager.get_latest_combined_frame()

        if frame_to_encode is None:
            time.sleep(0.05)
            continue

        try:
            (flag, encodedImage) = cv2.imencode(".jpg", frame_to_encode, [cv2.IMWRITE_JPEG_QUALITY, 80]) # Slightly higher quality
            if not flag:
                logging.warning("Stream generator: Could not encode frame to JPEG.")
                time.sleep(0.1)
                continue

            yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' +
                   bytearray(encodedImage) + b'\r\n')

            frame_counter += 1
            # --- Rate limiting (Approximate) ---
            current_time = time.monotonic()
            elapsed = current_time - last_frame_time
            # Base target delay on primary camera's FPS? Or fixed rate?
            # Let's aim for a fixed reasonable stream rate like 25-30fps
            target_delay = 1.0 / 30.0 # Aim for 30fps stream
            sleep_time = max(0.01, target_delay - elapsed)
            time.sleep(sleep_time)
            last_frame_time = time.monotonic()

        except GeneratorExit:
            logging.info(f"Streaming client disconnected after {frame_counter} frames.")
            break
        except Exception as e:
            logging.exception(f"!!! Error in MJPEG streaming generator: {e}")
            time.sleep(0.5)

    logging.info("Stream generator thread exiting.")


def build_options_html(options_source, current_value_key):
    """
    Helper to build <option> tags for HTML select dropdowns.
    Can handle lists (for modes) or dictionaries (for ISO).
    """
    options_html = ""
    if isinstance(options_source, dict): # For ISO { "Name": value }
        for key_name in options_source.keys():
            selected_attr = ' selected' if key_name == current_value_key else ''
            options_html += f'<option value="{key_name}"{selected_attr}>{key_name}</option>'
    elif isinstance(options_source, list): # For simple mode lists ["Mode1", "Mode2"]
        for mode_name in options_source:
            selected_attr = ' selected' if mode_name == current_value_key else ''
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
         return "Error: Application not fully initialized.", 500

    cam_state = _camera_manager.get_camera_state()
    # Assume get_current_servo_angle() exists in hardware_manager
    try:
        current_servo_angle = _hardware_manager.get_current_servo_angle()
        servo_angle_initial = int(current_servo_angle) if current_servo_angle is not None else config.SERVO_CENTER_ANGLE
    except AttributeError:
         logging.error("HardwareManager does not have get_current_servo_angle() method yet!")
         servo_angle_initial = config.SERVO_CENTER_ANGLE # Fallback
         # Set error message?
         cam_state['last_error'] = cam_state.get('last_error', '') + " Servo angle unavailable."


    hw_state = {
        'battery_percent': _hardware_manager.battery_percentage,
        'last_error': _hardware_manager.last_error,
        'servo_angle': servo_angle_initial
    }
    with _ui_lock:
        app_state_copy = _app_state.copy()

    # Combine error messages
    err_msg = cam_state.get('last_error') or hw_state.get('last_error') or ""
    if cam_state.get('audio_last_error'): # Add audio error if present
         err_msg += (err_msg.strip() and " | " or "") + "Audio: " + cam_state['audio_last_error']


    # Cam0 resolution text
    current_w, current_h = cam_state.get('resolution_wh', ('?', '?'))
    resolution_text = f"{current_w}x{current_h}"
    batt_perc_initial = hw_state.get('battery_percent')
    batt_text_initial = f"{batt_perc_initial:.1f}" if batt_perc_initial is not None else "--"

    # Build HTML option strings
    # Use config.AVAILABLE_ISO_SETTINGS dict for ISO
    iso_options_html = build_options_html(config.AVAILABLE_ISO_SETTINGS, cam_state.get('iso_mode', config.DEFAULT_ISO_NAME))
    ae_options_html = build_options_html(config.AVAILABLE_AE_MODES, cam_state.get('ae_mode', config.DEFAULT_AE_MODE_NAME))
    metering_options_html = build_options_html(config.AVAILABLE_METERING_MODES, cam_state.get('metering_mode', config.DEFAULT_METERING_MODE_NAME))
    noise_reduction_options_html = build_options_html(config.AVAILABLE_NOISE_REDUCTION_MODES, cam_state.get('noise_reduction_mode', config.DEFAULT_NOISE_REDUCTION_MODE_NAME))

    # Render the template string
    return render_template_string(HTML_TEMPLATE,
        resolution_text=resolution_text,
        # current_w=current_w, # Removed from template img tag
        # current_h=current_h, # Removed from template img tag
        err_msg=err_msg,
        digital_rec_state_initial=app_state_copy.get('digital_recording_active', False),
        batt_text_initial=batt_text_initial,
        # Pass current control values from CameraManager state
        current_iso_mode_name_initial=cam_state.get('iso_mode', config.DEFAULT_ISO_NAME), # ISO name
        iso_options_html=iso_options_html,
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
        # Servo initial values
        servo_angle_initial=servo_angle_initial,
        SERVO_MIN_ANGLE=config.SERVO_MIN_ANGLE,
        SERVO_MAX_ANGLE=config.SERVO_MAX_ANGLE,
        video_feed_url=url_for('video_feed')
    )


@app.route("/video_feed")
def video_feed():
    """Route for the MJPEG video stream (combined view)."""
    return Response(generate_stream_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route("/status")
def status():
    """Returns the current status of the application as JSON."""
    global _camera_manager, _hardware_manager, _app_state

    if not _camera_manager or not _hardware_manager:
        return jsonify({'error': 'Application not fully initialized.'}), 500

    cam_state = _camera_manager.get_camera_state()
    batt_perc = _hardware_manager.battery_percentage
    # Assume get_current_servo_angle() exists
    try:
        current_servo_angle = _hardware_manager.get_current_servo_angle()
        servo_angle = int(current_servo_angle) if current_servo_angle is not None else None
    except AttributeError:
         servo_angle = None # Indicate error or unavailability


    # Combine error messages
    err_msg = cam_state.get('last_error') or _hardware_manager.last_error or ""
    if cam_state.get('audio_last_error'):
         err_msg += (err_msg.strip() and " | " or "") + "Audio: " + cam_state['audio_last_error']

    # Auto-clear certain errors (simplified logic)
    latest_frame = _camera_manager.get_latest_combined_frame()
    if latest_frame is not None and err_msg and ("unavailable" in err_msg or "capture" in err_msg):
         logging.info("Status: Auto-clearing previous camera/capture error as frames are being received.")
         if _camera_manager.last_error == err_msg: _camera_manager.last_error = None
         if _hardware_manager.last_error == err_msg: _hardware_manager.last_error = None
         err_msg = ""
    if batt_perc is not None and err_msg and ("Battery Monitor" in err_msg or "INA219" in err_msg or "I2C Error" in err_msg):
         logging.info("Status: Auto-clearing previous battery monitor error as a reading was successful.")
         if _hardware_manager.last_error == err_msg: _hardware_manager.last_error = None
         err_msg = ""
    if servo_angle is not None and err_msg and ("Servo" in err_msg):
         logging.info("Status: Auto-clearing previous servo error as angle was retrieved.")
         if _hardware_manager.last_error == err_msg: _hardware_manager.last_error = None
         err_msg = ""

    # Build status text
    status_text = "Streaming"
    if not cam_state.get('is_initialized0', False): status_text += " (Cam0 ERR)"
    if not cam_state.get('is_initialized1', False): status_text += " (Cam1 ERR)"
    if cam_state.get('is_recording'):
        rec_count = len(cam_state.get('recording_paths', []))
        status_text += f" (Recording Cam0 to {rec_count} USB(s))"
        if config.AUDIO_ENABLED and cam_state.get('audio_last_error'):
             status_text += " (Audio ERR)"


    with _ui_lock:
        digital_rec_active = _app_state.get("digital_recording_active", False)

    status_data = {
        'is_recording': cam_state.get('is_recording', False),
        'digital_recording_active': digital_rec_active,
        'resolution': f"{cam_state.get('resolution_wh', ['?','?'])[0]}x{cam_state.get('resolution_wh', ['?','?'])[1]}", # Cam0 Res
        'status_text': status_text,
        'error': err_msg,
        'audio_last_error': cam_state.get('audio_last_error'), # Separate audio error
        'active_recordings': cam_state.get('recording_paths', []),
        'battery_percent': batt_perc,
        'servo_angle': servo_angle, # Add servo angle
        # Camera Controls
        'iso_mode': cam_state.get('iso_mode'), # ISO Name
        'analogue_gain': cam_state.get('analogue_gain'), # ISO Float value
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
    """Handles requests to change the Cam0 resolution."""
    global _camera_manager, _app_state

    if not _camera_manager: return jsonify({'success': False, 'message': 'Camera manager not ready.'}), 500

    with _ui_lock:
        if _app_state.get("reconfigure_resolution_index") is not None:
            return jsonify({'success': False, 'message': 'Reconfiguration already in progress.'}), 429

        current_index = _camera_manager.current_resolution_index0 # Get Cam0 index
        original_index = current_index
        new_index = current_index

        if direction == 'up': new_index += 1
        elif direction == 'down': new_index -= 1
        else: return jsonify({'success': False, 'message': 'Invalid direction specified.'}), 400

        new_index = max(0, min(len(config.CAM0_RESOLUTIONS) - 1, new_index))

        if new_index == original_index:
            msg = 'Already at highest resolution.' if direction == 'up' else 'Already at lowest resolution.'
            return jsonify({'success': False, 'message': msg}), 400

        _app_state["reconfigure_resolution_index"] = new_index
        new_w, new_h, new_fps = config.CAM0_RESOLUTIONS[new_index]
        logging.info(f"Web request: Queue Cam0 resolution change index {original_index} -> {new_index} ({new_w}x{new_h}@{new_fps}fps)")
        _camera_manager.last_error = None # Clear error on queuing

        return jsonify({'success': True, 'message': 'Resolution change requested.', 'new_resolution': f"{new_w}x{new_h}"})


@app.route('/toggle_recording', methods=['POST'])
def toggle_recording():
    """Toggles the digital recording trigger state."""
    global _app_state, _camera_manager, _hardware_manager # Allow clearing errors
    new_state = False
    with _ui_lock:
        current_state = _app_state.get("digital_recording_active", False)
        _app_state["digital_recording_active"] = not current_state
        new_state = _app_state["digital_recording_active"]
        logging.info(f"Digital recording trigger toggled via web UI to: {'ON' if new_state else 'OFF'}")
        # Clear potential previous recording/audio errors when user interacts
        if _camera_manager:
            if _camera_manager.last_error and ("Recording" in _camera_manager.last_error or "writers" in _camera_manager.last_error or "USB" in _camera_manager.last_error or "sync" in _camera_manager.last_error or "Mux" in _camera_manager.last_error):
                logging.info(f"Clearing previous recording error via toggle: '{_camera_manager.last_error}'")
                _camera_manager.last_error = None
            if _camera_manager.audio_last_error:
                 logging.info(f"Clearing previous audio error via toggle: '{_camera_manager.audio_last_error}'")
                 _camera_manager.audio_last_error = None
        if _hardware_manager and _hardware_manager.last_error and ("Recording" in _hardware_manager.last_error):
             _hardware_manager.last_error = None

    return jsonify({'success': True, 'digital_recording_active': new_state})


@app.route('/set_camera_control', methods=['POST'])
def set_camera_control():
    """Sets a specific common camera control value for both cameras."""
    global _camera_manager

    if not _camera_manager: return jsonify({'success': False, 'message': 'Camera manager not ready.'}), 500
    # Check if *any* camera is initialized? Or require both? Let's allow if at least one is ready.
    if not _camera_manager.is_initialized0 and not _camera_manager.is_initialized1:
         return jsonify({'success': False, 'message': 'Cameras not initialized.'}), 503

    control_name_req = None # Requested control name/key
    control_value_req = None # Requested control value (might be name for ISO)
    try:
        data = request.get_json()
        if not data or 'control' not in data or 'value' not in data:
            logging.warning("/set_camera_control called without 'control' or 'value'.")
            return jsonify({'success': False, 'message': 'Missing control or value in request.'}), 400

        control_name_req = data['control']
        control_value_req = data['value']
        logging.info(f"Web request: Set control '{control_name_req}' to '{control_value_req}'")

        control_dict_to_apply = {} # Controls to pass to camera_manager
        validation_error = None

        # --- Validate and Parse Value ---
        if control_name_req == 'AnalogueGain': # ISO control uses name in request
            iso_name = str(control_value_req)
            gain_value = config.AVAILABLE_ISO_SETTINGS.get(iso_name)
            if gain_value is not None:
                control_dict_to_apply["AnalogueGain"] = gain_value
                if gain_value == 0.0: # Auto ISO
                     control_dict_to_apply["AeEnable"] = True # Ensure AE is on for Auto ISO
            else:
                validation_error = f"Invalid ISO Mode name: {iso_name}"
        elif control_name_req == 'AeExposureMode':
            enum_val = controls.AeExposureModeEnum.__members__.get(control_value_req)
            if enum_val is not None: control_dict_to_apply["AeExposureMode"] = enum_val; control_dict_to_apply["AeEnable"] = True
            else: validation_error = f"Invalid AeExposureMode value: {control_value_req}"
        elif control_name_req == 'AeMeteringMode':
            enum_val = controls.AeMeteringModeEnum.__members__.get(control_value_req)
            if enum_val is not None: control_dict_to_apply["AeMeteringMode"] = enum_val; control_dict_to_apply["AeEnable"] = True
            else: validation_error = f"Invalid AeMeteringMode value: {control_value_req}"
        elif control_name_req == 'NoiseReductionMode':
            try:
                enum_val = controls.draft.NoiseReductionModeEnum.__members__.get(control_value_req)
                if enum_val is not None: control_dict_to_apply["NoiseReductionMode"] = enum_val
                else: validation_error = f"Invalid NoiseReductionMode value: {control_value_req}"
            except AttributeError: validation_error = "NoiseReductionMode control not available."
        elif control_name_req == 'Brightness':
            try: val = float(control_value_req); control_dict_to_apply["Brightness"] = max(config.MIN_BRIGHTNESS, min(config.MAX_BRIGHTNESS, val))
            except ValueError: validation_error = "Invalid numeric value for Brightness"
        elif control_name_req == 'Contrast':
            try: val = float(control_value_req); control_dict_to_apply["Contrast"] = max(config.MIN_CONTRAST, min(config.MAX_CONTRAST, val))
            except ValueError: validation_error = "Invalid numeric value for Contrast"
        elif control_name_req == 'Saturation':
            try: val = float(control_value_req); control_dict_to_apply["Saturation"] = max(config.MIN_SATURATION, min(config.MAX_SATURATION, val))
            except ValueError: validation_error = "Invalid numeric value for Saturation"
        elif control_name_req == 'Sharpness':
            try: val = float(control_value_req); control_dict_to_apply["Sharpness"] = max(config.MIN_SHARPNESS, min(config.MAX_SHARPNESS, val))
            except ValueError: validation_error = "Invalid numeric value for Sharpness"
        else:
            validation_error = f"Unknown control name: {control_name_req}"

        # --- Handle Validation Result ---
        if validation_error:
            logging.error(f"Control validation failed for '{control_name_req}': {validation_error}")
            return jsonify({'success': False, 'message': validation_error}), 400

        # --- Apply Control ---
        if not control_dict_to_apply:
             logging.error(f"Internal error: No controls parsed for request '{control_name_req}'.")
             return jsonify({'success': False, 'message': 'Internal processing error.'}), 500

        if _camera_manager.apply_camera_controls(control_dict_to_apply):
             # Value applied successfully by camera manager
             applied_val_str = str(list(control_dict_to_apply.values())[0]) # Get first value applied
             logging.info(f"Successfully applied controls: {control_dict_to_apply}")
             return jsonify({'success': True, 'message': f'{control_name_req} set.'})
        else:
             # apply_camera_controls failed, error should be in manager's last_error
             error_msg = _camera_manager.last_error or f"Failed to apply {control_name_req}"
             return jsonify({'success': False, 'message': error_msg}), 500

    except Exception as e:
        logging.error(f"Error processing set_camera_control '{control_name_req}': {e}", exc_info=True)
        return jsonify({'success': False, 'message': f'Unexpected server error: {e}'}), 500


@app.route('/set_servo_angle', methods=['POST'])
def set_servo_angle():
    """Sets the servo angle."""
    global _hardware_manager

    if not _hardware_manager: return jsonify({'success': False, 'message': 'Hardware manager not ready.'}), 500
    if not config.SERVO_ENABLED: return jsonify({'success': False, 'message': 'Servo control is disabled in config.'}), 403

    angle = None # Initialize for error logging
    try:
        data = request.get_json()
        if not data or 'angle' not in data:
            logging.warning("/set_servo_angle called without 'angle'.")
            return jsonify({'success': False, 'message': 'Missing angle in request.'}), 400

        angle_req = data['angle']
        logging.info(f"Web request: Set servo angle to '{angle_req}'")

        # Validate angle
        try:
            angle = int(angle_req)
            if not (config.SERVO_MIN_ANGLE <= angle <= config.SERVO_MAX_ANGLE):
                 raise ValueError(f"Angle {angle} out of range ({config.SERVO_MIN_ANGLE}-{config.SERVO_MAX_ANGLE})")
        except (ValueError, TypeError) as e:
            logging.error(f"Invalid servo angle value: {angle_req}. Error: {e}")
            return jsonify({'success': False, 'message': f"Invalid angle: {angle_req}. Must be integer between {config.SERVO_MIN_ANGLE} and {config.SERVO_MAX_ANGLE}."}), 400

        # Apply angle using HardwareManager method
        # Assume set_servo updates internal state and handles errors
        _hardware_manager.set_servo(angle)

        # Check for errors set by hardware_manager during set_servo
        if _hardware_manager.last_error and "Servo" in _hardware_manager.last_error:
            error_msg = _hardware_manager.last_error
            # Clear the error after reporting it? Or let status poll clear it? Let status poll clear.
            return jsonify({'success': False, 'message': error_msg}), 500
        else:
            logging.info(f"Successfully set servo angle to: {angle}")
            return jsonify({'success': True, 'message': f'Servo angle set to {angle}°.'})

    except Exception as e:
        logging.error(f"Error processing set_servo_angle '{angle}': {e}", exc_info=True)
        return jsonify({'success': False, 'message': f'Unexpected server error: {e}'}), 500


@app.route('/power_down', methods=['POST'])
def power_down():
    """Signals the main application to initiate shutdown and reboot."""
    global _app_state, _camera_manager, _hardware_manager
    logging.warning("Received request for power down via web UI. Setting flags.")

    with _ui_lock:
        _app_state["reboot_requested"] = True
        shutdown_event = _app_state.get("shutdown_event")
        if shutdown_event:
            shutdown_event.set()
        else:
            logging.error("Power down requested, but shutdown_event not set in app_state!")
            return jsonify({'success': False, 'message': 'Internal server error: Shutdown event not configured.'}), 500

    # Update manager errors
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
        app.run(host='0.0.0.0', port=config.WEB_PORT, debug=False, use_reloader=False, threaded=True)
    except Exception as e:
        logging.exception(f"!!! Flask web server failed: {e}")
        with _ui_lock:
            shutdown_event = _app_state.get("shutdown_event")
            if shutdown_event and not shutdown_event.is_set():
                logging.error("Signaling main thread shutdown due to Flask error.")
                shutdown_event.set()

# Example Usage (for testing purposes - requires dummy managers)
if __name__ == "__main__":
    # --- Dummy Managers ---
    class DummyCamManager:
        is_initialized0 = True; is_initialized1 = True; is_recording = False; last_error = None; audio_last_error = None
        current_resolution_index0 = config.CAM0_DEFAULT_RESOLUTION_INDEX
        current_iso_name = config.DEFAULT_ISO_NAME; current_analogue_gain = config.DEFAULT_ANALOGUE_GAIN
        current_ae_mode = config.DEFAULT_AE_MODE_NAME; current_metering_mode = config.DEFAULT_METERING_MODE_NAME
        current_noise_reduction_mode = config.DEFAULT_NOISE_REDUCTION_MODE_NAME
        current_brightness = config.DEFAULT_BRIGHTNESS; current_contrast = config.DEFAULT_CONTRAST
        current_saturation = config.DEFAULT_SATURATION; current_sharpness = config.DEFAULT_SHARPNESS
        recording_paths = []
        def get_latest_combined_frame(self):
            img = np.zeros((720, 1280, 3), dtype=np.uint8); cv2.putText(img,'Combined Stream',(50,360),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),2)
            return img
        def get_camera_state(self):
             w,h,f = config.CAM0_RESOLUTIONS[self.current_resolution_index0]
             return {'is_initialized': True, 'is_initialized0': True, 'is_initialized1': True, 'resolution_index': self.current_resolution_index0, 'resolution_wh': (w,h), 'resolution_fps': f, 'is_recording': self.is_recording, 'recording_paths': self.recording_paths, 'last_error': self.last_error, 'audio_last_error': self.audio_last_error, 'iso_mode': self.current_iso_name, 'analogue_gain': self.current_analogue_gain, 'ae_mode': self.current_ae_mode, 'metering_mode': self.current_metering_mode, 'noise_reduction_mode': self.current_noise_reduction_mode, 'brightness': self.current_brightness, 'contrast': self.current_contrast, 'saturation': self.current_saturation, 'sharpness': self.current_sharpness}
        def apply_camera_controls(self, d): print(f"DummyCam: Apply controls {d}"); return True
        def initialize_cameras(self, idx): print(f"DummyCam: Init cameras (idx={idx})"); return True
        def toggle_recording(self): self.is_recording = not self.is_recording; return True
        def shutdown(self): print("DummyCam Shutdown")

    class DummyHWManager:
         battery_percentage = 75.3
         last_error = None
         current_servo_angle = config.SERVO_CENTER_ANGLE
         def get_current_servo_angle(self): return self.current_servo_angle
         def set_servo(self, angle): print(f"DummyHW: Set servo to {angle}"); self.current_servo_angle = angle; return True
         def cleanup(self): print("DummyHW Cleanup")

    # --- Setup Logging ---
    logging.basicConfig(level=config.LOG_LEVEL, format=config.LOG_FORMAT, datefmt=config.LOG_DATE_FORMAT)
    logging.getLogger("werkzeug").setLevel(logging.WARNING)

    # --- Setup and Run ---
    print("--- Web UI Test (Dual Cam / Servo) ---")
    print(f"--- Running on http://0.0.0.0:{config.WEB_PORT} ---")
    dummy_shutdown = threading.Event()
    setup_web_app(DummyCamManager(), DummyHWManager(), dummy_shutdown)
    run_web_server()
    print("--- Web UI Test Complete ---")

