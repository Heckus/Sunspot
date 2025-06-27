import cv2
import numpy as np
import os

# =============================================================================
# --- Configuration ---
# =============================================================================
INPUT_VIDEO_PATH = 'drive/MyDrive/Colab Notebooks/numbers_detection/video_samples/input/IMG_9088.mp4'
PROCESSING_WIDTH = 1280
PROCESSING_HEIGHT = 720

def nothing(x):
    """Callback function for trackbars. Does nothing."""
    pass

# --- Video Loading and Validation ---
if not os.path.exists(INPUT_VIDEO_PATH):
    print(f"[ERROR] Video file not found: {INPUT_VIDEO_PATH}")
    exit()

video = cv2.VideoCapture(INPUT_VIDEO_PATH)
if not video.isOpened():
    print(f"[ERROR] Could not open video file. It might be corrupt or an incorrect codec.")
    exit()

total_frames = int(video.get(cv2.CAP_PROP_FRAME_COUNT))
ret, frame = video.read()
if not ret:
    print("[ERROR] Could not read the first frame from the video.")
    exit()

# --- Window and Trackbar Creation for YELLOW ---
cv2.namedWindow("Yellow Trackbars")
cv2.createTrackbar("Lower H", "Yellow Trackbars", 20, 179, nothing)
cv2.createTrackbar("Lower S", "Yellow Trackbars", 150, 255, nothing)
cv2.createTrackbar("Lower V", "Yellow Trackbars", 150, 255, nothing)
cv2.createTrackbar("Upper H", "Yellow Trackbars", 35, 179, nothing)
cv2.createTrackbar("Upper S", "Yellow Trackbars", 255, 255, nothing)
cv2.createTrackbar("Upper V", "Yellow Trackbars", 255, 255, nothing)

# --- Window and Trackbar Creation for BLUE ---
cv2.namedWindow("Blue Trackbars")
cv2.createTrackbar("Lower H", "Blue Trackbars", 90, 179, nothing)
cv2.createTrackbar("Lower S", "Blue Trackbars", 100, 255, nothing)
cv2.createTrackbar("Lower V", "Blue Trackbars", 100, 255, nothing)
cv2.createTrackbar("Upper H", "Blue Trackbars", 130, 179, nothing)
cv2.createTrackbar("Upper S", "Blue Trackbars", 255, 255, nothing)
cv2.createTrackbar("Upper V", "Blue Trackbars", 255, 255, nothing)

# --- Timeline Trackbar ---
cv2.namedWindow("Timeline Control")
cv2.createTrackbar("Timeline", "Timeline Control", 0, total_frames - 1, nothing)


print("[INFO] Tuner started. Press SPACE to play/pause. Drag Timeline to scrub. Press 'q' to quit.")
is_paused = True

while True:
    # --- Timeline Synchronization Logic ---
    timeline_pos = cv2.getTrackbarPos("Timeline", "Timeline Control")
    current_frame_pos = int(video.get(cv2.CAP_PROP_POS_FRAMES))

    if abs(timeline_pos - current_frame_pos) > 1:
        video.set(cv2.CAP_PROP_POS_FRAMES, timeline_pos)
        ret, frame = video.read()
        if not ret: continue

    if not is_paused:
        ret, frame = video.read()
        if not ret:
            is_paused = True
            cv2.setTrackbarPos("Timeline", "Timeline Control", total_frames - 1)
        else:
            cv2.setTrackbarPos("Timeline", "Timeline Control", current_frame_pos)

    frame_resized = cv2.resize(frame, (PROCESSING_WIDTH, PROCESSING_HEIGHT))
    hsv = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2HSV)

    # --- Get HSV values for YELLOW ---
    yl_h, yl_s, yl_v = cv2.getTrackbarPos("Lower H", "Yellow Trackbars"), cv2.getTrackbarPos("Lower S", "Yellow Trackbars"), cv2.getTrackbarPos("Lower V", "Yellow Trackbars")
    yu_h, yu_s, yu_v = cv2.getTrackbarPos("Upper H", "Yellow Trackbars"), cv2.getTrackbarPos("Upper S", "Yellow Trackbars"), cv2.getTrackbarPos("Upper V", "Yellow Trackbars")
    lower_yellow = np.array([yl_h, yl_s, yl_v])
    upper_yellow = np.array([yu_h, yu_s, yu_v])
    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # --- Get HSV values for BLUE ---
    bl_h, bl_s, bl_v = cv2.getTrackbarPos("Lower H", "Blue Trackbars"), cv2.getTrackbarPos("Lower S", "Blue Trackbars"), cv2.getTrackbarPos("Lower V", "Blue Trackbars")
    bu_h, bu_s, bu_v = cv2.getTrackbarPos("Upper H", "Blue Trackbars"), cv2.getTrackbarPos("Upper S", "Blue Trackbars"), cv2.getTrackbarPos("Upper V", "Blue Trackbars")
    lower_blue = np.array([bl_h, bl_s, bl_v])
    upper_blue = np.array([bu_h, bu_s, bu_v])
    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # --- Combine masks ---
    combined_mask = cv2.bitwise_or(yellow_mask, blue_mask)
    result = cv2.bitwise_and(frame_resized, frame_resized, mask=combined_mask)

    # --- Display frames ---
    cv2.imshow("Original Frame", frame_resized)
    cv2.imshow("Combined HSV Mask", combined_mask)

    # --- Display values on Result window ---
    font = cv2.FONT_HERSHEY_SIMPLEX
    text_y1 = f"LOWER_YELLOW = np.array([{yl_h},{yl_s},{yl_v}])"
    text_y2 = f"UPPER_YELLOW = np.array([{yu_h},{yu_s},{yu_v}])"
    text_b1 = f"LOWER_BLUE = np.array([{bl_h},{bl_s},{bl_v}])"
    text_b2 = f"UPPER_BLUE = np.array([{bu_h},{bu_s},{bu_v}])"
    cv2.putText(result, text_y1, (10, 30), font, 0.7, (0, 255, 255), 2)
    cv2.putText(result, text_y2, (10, 60), font, 0.7, (0, 255, 255), 2)
    cv2.putText(result, text_b1, (10, 100), font, 0.7, (255, 255, 0), 2)
    cv2.putText(result, text_b2, (10, 130), font, 0.7, (255, 255, 0), 2)
    cv2.imshow("Result", result)

    key = cv2.waitKey(30) & 0xFF
    if key == ord('q'): break
    if key == ord(' '): is_paused = not is_paused

video.release()
cv2.destroyAllWindows()