import cv2

# Open libcamera pipeline
cap = cv2.VideoCapture("libcamerasrc ! video/x-raw,format=RGB,width=640,height=480 ! videoconvert ! appsink", cv2.CAP_GSTREAMER)
if not cap.isOpened():
    print("Failed to open camera!")
else:
    print("Camera works!")
cap.release()