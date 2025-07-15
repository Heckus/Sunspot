import cv2
from picamera2 import Picamera2

def get_focus_score(frame):
    """Calculates a focus score for a given single-channel image frame."""
    # Apply Laplacian operator
    laplacian = cv2.Laplacian(frame, cv2.CV_64F)
    # Calculate the variance
    focus_score = laplacian.var()
    return focus_score

# Initialize Picamera2
picam2 = Picamera2()

# Configure the camera for preview. A lower resolution is faster.
# Using a YUV format like 'YUV420' lets us grab the grayscale 'Y' channel easily.
config = picam2.create_preview_configuration(
    main={"size": (1280, 720), "format": "YUV420"}
)
picam2.configure(config)

# Start the camera feed
picam2.start()

print("Starting focus assistant. Press 'q' to quit.")

while True:
    # Capture a frame. The YUV420 format gives us the luminance (grayscale) channel
    # directly as the first part of the buffer, which is efficient.
    yuv_frame = picam2.capture_array("main")
    grey_frame = yuv_frame[:720, :] # Extract the Y (grayscale) component

    # Calculate the focus score
    score = get_focus_score(grey_frame)

    # --- Display the score on the frame ---
    # Convert the grey frame to color so we can draw colored text on it
    display_frame = cv2.cvtColor(grey_frame, cv2.COLOR_GRAY2BGR)

    # Position text in the top-left corner
    text = f"Focus Score: {score:.2f}"
    cv2.putText(display_frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.putText(display_frame, "Adjust lens to maximize score", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

    # Show the frame
    cv2.imshow("Focus Assistant", display_frame)

    # Check for 'q' key press to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Clean up
cv2.destroyAllWindows()
picam2.stop()
print("Script finished.")