🏐 3D Volleyball Tracker with ROS2 🤖

This repository contains the source code for a real-time 3D volleyball tracking system developed for a Raspberry Pi. The system uses a camera, a custom AI model, and the Robot Operating System 2 (ROS2) to detect a volleyball and determine its precise 3D position.

This project serves as a comprehensive demonstration of skills in robotics, AI, and computer vision, from low-level hardware communication to high-level software architecture.

🌟 Overview

The core objective of this project is to create a robust and scalable system that can:

-Capture a live video stream from a camera connected to a Raspberry Pi.

-Detect a volleyball in the video feed using a custom-trained YOLO object detection model.

-Calculate the 3D coordinates (x, y, z) of the detected volleyball using camera calibration data.

-Publish the 3D position and other relevant data as ROS2 topics, allowing for seamless integration with other robotic systems.

🚀 Key Features

-Real-time Performance: Optimized for high-speed processing on a Raspberry Pi.

-AI-Powered Detection: Utilizes a custom-trained YOLOv5 model for accurate and efficient volleyball detection.

-3D Position Estimation: Implements computer vision techniques to translate 2D image coordinates into real-world 3D positions.

-Modular ROS2 Architecture: Built with a modular node-based structure in ROS2, allowing for easy testing, debugging, and expansion.

-Calibration Utilities: Includes dedicated ROS2 nodes for performing extrinsic camera calibration.

-System Monitoring: Features a battery monitoring node to track the robot's power status.

-Visualization Ready: Comes with RViz configuration files for easy visualization of the robot model and sensor data.

🛠️ Tech Stack & Skills Demonstrated

This project showcases a wide range of in-demand skills in robotics and software engineering:

-Programming: Advanced proficiency in Python for developing complex, multi-node robotic systems.

-Robotics:

--ROS2: Deep understanding of the ROS2 framework, including creating nodes, topics, services, launch files, and URDF models.

--System Integration: Expertise in integrating various hardware and software components into a cohesive robotic system.

--TF2 (Transformations): Using the TF2 library to manage and broadcast coordinate frame transformations within ROS2.

-Computer Vision:

--OpenCV: Extensive use of OpenCV for image processing, camera interfacing, and calibration.

--Camera Calibration: Practical application of camera calibration techniques to solve for intrinsic and extrinsic parameters.

-Artificial Intelligence:

--Object Detection: Experience training and deploying custom object detection models (YOLO).

--PyTorch: Integrating PyTorch-based models into a real-time application.

-Hardware & Embedded Systems:

--Raspberry Pi: Setting up and developing for a Raspberry Pi in a robotics context.

--Sensor Interfacing: Interfacing with cameras and system-level utilities (e.g., battery monitoring).

🔧 System Architecture

The system is composed of several key ROS2 nodes that work in concert:

-camera_node: Captures images from the camera and publishes them to a topic.

-detection_node: Subscribes to the image topic, runs the YOLO model to detect the volleyball, and publishes the 2D bounding box coordinates.

-calculation_node: Subscribes to the detection results and uses calibration data to calculate and publish the 3D position of the volleyball.

-extrinsic_calibration_node: A utility node to help perform and save the camera's extrinsic calibration.

-battery_monitor_node: A utility node that periodically checks and publishes the device's battery status.
