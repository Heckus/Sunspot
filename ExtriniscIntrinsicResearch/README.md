# 3D Volleyball Tracking System

This project uses a single camera to detect a volleyball and track its position in a 3D-calibrated space. It leverages computer vision techniques to identify the ball in a 2D video feed and then uses intrinsic and extrinsic camera parameters to estimate its real-world 3D coordinates. The system includes a web-based UI for visualizing the camera feed and the ball's calculated 3D position in real-time.

## System Components

- **`main.py`**: The main application entry point. It orchestrates the camera, CV functions, and web UI to run the tracking loop.
- **`Config.py`**: A centralized configuration file for all system parameters, including camera settings, world definitions, and CV thresholds.
- **`CameraManager.py`**: Handles all interactions with the camera hardware (Picamera2), including initialization and frame capture.
- **`CvFunctions.py`**: Contains all computer vision logic, including ball detection and the 2D-to-3D position estimation algorithms.
- **`WebUi.py`**: A Flask-based web server that provides a real-time dashboard with an annotated video stream and a 3D visualization of the tracked space.
- **`camera_calibration.py`**: A standalone script for performing the initial intrinsic camera calibration using a checkerboard pattern.

## Hardware Requirements

1.  **Raspberry Pi**: A Raspberry Pi 4 or 5 is recommended for adequate processing power.
2.  **Camera**: A Raspberry Pi Camera Module (v2 or v3).
3.  **Reference Object for Calibration**:
    * **Intrinsic**: A printed checkerboard pattern (e.g., 10x7 squares). The exact size must be known.
    * **Extrinsic**: A physical 1x1x1 meter cube with clearly identifiable corners. This is used to define the world coordinate system.

## Software Setup

This project is written in Python 3. It is highly recommended to use a virtual environment to manage dependencies.

1.  **Clone the Repository**:
    ```bash
    git clone <your-repo-url>
    cd <your-repo-directory>
    ```

2.  **Create and Activate a Virtual Environment**:
    ```bash
    python3 -m venv venv
    source venv/bin/activate
    ```

3.  **Install Dependencies**: Install all required Python packages.
    ```bash
    pip install opencv-contrib-python numpy "picamera2[gui]" flask
    ```

## System Operation: A Step-by-Step Guide

The system requires a two-step calibration process before it can be run. **This process only needs to be done once**, unless the camera is moved or its lens is refocused.

### Step 1: Intrinsic Camera Calibration

This step teaches the system about the camera's internal properties, like focal length and lens distortion.

1.  **Configure**: Open `Config.py` and ensure the `CHECKERBOARD_DIMS` and `SQUARE_SIZE_MM` variables match your physical checkerboard pattern.
2.  **Run the Calibration Script**:
    ```bash
    python camera_calibration.py
    ```
3.  **Follow On-Screen Instructions**: A window will appear showing the camera feed. Position your checkerboard at various angles and distances from the camera. Press **SPACE** or **CLICK** to capture an image. The script needs about 30 good images to produce an accurate calibration.
4.  **Verify Output**: Once complete, the script will generate a `camera_calibration_data.npz` file. This file is essential for all subsequent steps.

### Step 2: Extrinsic Camera Calibration

This step teaches the system where the camera is located and how it is oriented relative to your defined 3D world (the 1x1x1m cube).

1.  **Set up the World**: Place your physical 1x1x1 meter reference cube in the location you want to track.
2.  **Position the Camera**: Place the camera in its final, fixed position where it has a clear view of the cube.
3.  **Run the Main Program**: The extrinsic calibration is part of the startup sequence of the main application.
    ```bash
    python main.py
    ```
4.  **Follow On-Screen Instructions**:
    * A window titled "Extrinsic Calibration - Click Cube Corners" will appear.
    * You must click on the **8 corners of the cube** in a specific order, as printed in the terminal:
        1.  **Bottom Face First (4 clicks)**: Start with the origin (0,0,0) and proceed around the bottom face.
        2.  **Top Face Second (4 clicks)**: Click the corresponding corners on the top face.
    * After clicking the 8 points, press the **'c'** key to calculate the camera's pose.
    * A preview will show the world axes projected onto the cube. If it looks correct, press **'y'** to accept and save. The calibration data is saved to `camera_extrinsic_data.npz`.

### Step 3: Running the Tracker

Once both calibration files (`.npz`) exist, the system will skip the calibration steps and start tracking immediately.

1.  **Run the Main Program**:
    ```bash
    python main.py
    ```
2.  **Open the Web UI**: Open a web browser and navigate to the IP address of your Raspberry Pi on port 8000 (e.g., `http://<pi-ip-address>:8000`).
3.  **Start Tracking**: The web dashboard will load, showing the annotated camera feed on the left and the 3D visualization on the right. As the volleyball moves within the calibrated cube, its position will be updated in the 3D view.

To shut down the application, press **'q'** in the "2D Volleyball Tracking Overlay" window or press **Ctrl+C** in the terminal.
