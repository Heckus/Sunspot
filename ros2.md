# README: Headless ROS 2 on Raspberry Pi with Distrobox

This document outlines the successful procedure for setting up a robust, hardware-enabled ROS 2 Humble development environment on a headless Raspberry Pi. It also serves as a user guide for interacting with this new environment.

The method uses `distrobox` to create a containerized Ubuntu environment on top of Raspberry Pi OS. This provides a clean, isolated workspace with full access to the Pi's hardware (GPIO, I2C, SPI, USB, Cameras) without modifying the host operating system.

---

## Part A: The Proven Installation Steps

These are the final, successful steps we took to create the environment. This process assumes you have a Raspberry Pi running a 64-bit version of Raspberry Pi OS, have it connected to your network, and have installed `docker` and `distrobox` (version 1.8.1.2 or similar).

### Step 1: Clean Up Previous Attempts (If Necessary)

To ensure a clean slate, it's always best to remove any previously failed containers with the same name.

```bash
distrobox rm ros2-robotics
```

### Step 2: Create the Hardware-Enabled Container

This is the definitive command that creates the container. It uses a verified ARM64 base image for ROS 2 and passes all necessary flags to grant it access to the Raspberry Pi's hardware and prepare it for an init system.

```bash
distrobox create --name ros2-robotics \
  --image arm64v8/ros:humble-ros-base \
  --platform linux/arm64 \
  --hostname ros-container \
  --init \
  --unshare-ipc \
  --additional-packages "systemd" \
  --volume /dev/shm:/dev/shm \
  --additional-flags "--device /dev/gpiomem:/dev/gpiomem --device /dev/gpiochip0:/dev/gpiochip0 --device /dev/i2c-1:/dev/i2c-1 --device /dev/spidev0.0:/dev/spidev0.0 --device /dev/spidev0.1:/dev/spidev0.1 --group-add dialout --group-add video --cap-add=SYS_NICE --device-cgroup-rule='c 81:* rwm' --device-cgroup-rule='c 188:* rwm' --device-cgroup-rule='c 189:* rwm'"
```

* **Key Insight**: The `--init` flag requires an init system to be present. The base image is minimal and doesn't include one, so we must add `--additional-packages "systemd"` to install it during creation.

### Step 3: Enter the Container

Once successfully created, you can enter the container's environment.

```bash
distrobox enter ros2-robotics
```

You will now be at a shell prompt inside your isolated Ubuntu environment.

### Step 4: Upgrade to a Full Desktop Install (One-Time Setup)

The `ros-base` image is minimal. To get GUI tools like RViz and Gazebo, you need to install the full desktop package. This only needs to be done once.

```bash
# Inside the ros2-robotics container
sudo apt-get update && sudo apt-get install -y ros-humble-desktop
```

Your environment is now fully configured.

---

## Part B: Interfacing with Your ROS 2 Environment

This is your guide to using the container for robotics projects.

### Daily Workflow

1.  **Connect to your Pi**:
    * `ssh hecke@<your_pi_ip_address>`
2.  **Enter your ROS 2 Environment**:
    * `distrobox enter ros2-robotics`
3.  **Start working**: You can now run `ros2 launch`, `colcon build`, etc.
4.  **Exiting**:
    * Simply type `exit` to leave the container and return to the Raspberry Pi OS shell. The container will continue running in the background.
5.  **Stopping the Container (Optional)**:
    * To save resources, you can stop the container completely with `distrobox stop ros2-robotics`.

### Running ROS 2 Nodes (A "Hello World" Test)

To verify that ROS 2 is working correctly, you can run a simple talker/listener demo. You will need two separate terminals connected to your Pi.

* **Terminal 1**:
    ```bash
    ssh hecke@<your_pi_ip_address>
    distrobox enter ros2-robotics
    ros2 run demo_nodes_cpp talker
    ```

* **Terminal 2**:
    ```bash
    ssh hecke@<your_pi_ip_address>
    distrobox enter ros2-robotics
    ros2 run demo_nodes_py listener
    ```

You should see messages being published in Terminal 1 and received in Terminal 2, confirming your ROS 2 communication is working.

### Accessing Hardware

The container was created with broad hardware access.

* **GPIO, I2C, SPI**: These are directly available. You can use standard Python libraries like `gpiod` or `smbus2` from within a ROS 2 Python node just as you would on a native install.
* **USB Devices (Cameras, Lidars, Arduinos)**: Any USB device you plug into the Pi will automatically appear inside the container (e.g., as `/dev/video0`, `/dev/ttyUSB0`, `/dev/ttyACM0`). You can use standard ROS 2 packages to interface with them.
    * *Example*: `ros2 run v4l2_camera v4l2_camera_node`

### Using Graphical Tools like RViz

Since the Pi is headless, you must forward the display to your main computer.

* **Method 1: X11 Forwarding (Simpler)**
    1.  Connect to the Pi using `ssh -X hecke@<your_pi_ip_address>`.
    2.  `distrobox enter ros2-robotics`.
    3.  Launch RViz: `rviz2`. The window will appear on your computer's desktop.

* **Method 2: VNC (More Performant)**
    1.  Ensure you have a VNC server and a lightweight desktop (like XFCE) running on your Pi.
    2.  Connect from your main computer using a VNC Viewer.
    3.  Inside the VNC desktop session on the Pi, open a terminal.
    4.  `distrobox enter ros2-robotics`.
    5.  `rviz2`.

### Developing and Compiling Code

This is the most powerful feature of this setup.

1.  **Shared Home Directory**: Your user's home directory (`/home/hecke`) on the Raspberry Pi is the **exact same** as the home directory inside the container.
2.  **Get Your Code**: On the Raspberry Pi (or via an SFTP client like FileZilla), clone your ROS 2 workspace into your home directory:
    ```bash
    # On the Raspberry Pi host, NOT inside the container
    git clone <your_ros2_workspace_url> ~/ros2_ws
    ```
3.  **Build Your Code**: Enter the container, navigate to your workspace, and build it with `colcon`.
    ```bash
    distrobox enter ros2-robotics
    cd ~/ros2_ws
    colcon build
    ```
Because the files are shared, any changes you make to the code on the Pi are instantly reflected inside the container, and vice versa.
