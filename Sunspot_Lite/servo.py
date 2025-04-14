#!/usr/bin/env python3

import tkinter as tk
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory # Often needed for smoother PWM
import sys
import atexit

# --- Configuration ---
SERVO_PIN = 17  # BCM Pin for the Servo
# Adjust min/max pulse width if your servo behaves differently
# Standard SG90 servos usually work well with defaults (1ms to 2ms)
# which gpiozero maps to values -1 to 1.
# min_pulse = 0.0005 # Example: 0.5ms pulse width
# max_pulse = 0.0025 # Example: 2.5ms pulse width

# --- Servo Initialization ---
servo = None # Define servo variable outside try block
factory = None # Define factory variable outside try block

try:
    # Using PiGPIOFactory often results in less jittery servo movement
    # Make sure pigpiod daemon is running: sudo systemctl start pigpiod
    factory = PiGPIOFactory()
    servo = Servo(SERVO_PIN, pin_factory=factory) # min_pulse_width=min_pulse, max_pulse_width=max_pulse)
    print(f"Servo initialized on GPIO {SERVO_PIN} using pigpio.")

except Exception as e:
    print(f"Error initializing servo with pigpio: {e}")
    print("Have you installed gpiozero and pigpio?")
    print("Is the pigpiod service running? Try: sudo systemctl start pigpiod")
    # Fallback or exit if initialization fails
    try:
        print("Attempting fallback GPIO driver...")
        # Default RPi.GPIO or RPIO based factory might work but can be jittery
        servo = Servo(SERVO_PIN)
        print(f"Fallback successful: Servo initialized on GPIO {SERVO_PIN} using default driver.")
        print("Note: pigpio is recommended for smoother servo control.")
    except Exception as fallback_e:
        print(f"Fallback failed: {fallback_e}")
        print("Exiting. Please check GPIO setup, permissions, and libraries.")
        sys.exit(1) # Exit if servo cannot be initialized


# --- Servo Control Function (Modular Part) ---
def set_servo_position(value):
    """
    Sets the position of the servo.

    Args:
        value (float): The desired servo position, ranging from -1.0 (min angle)
                       to 1.0 (max angle). 0.0 is typically the center.
    """
    if servo: # Check if servo object exists
        # Ensure the value is within the valid range
        clamped_value = max(-1.0, min(1.0, value))
        try:
            print(f"Setting Servo (Pin {servo.pin}) to value: {clamped_value:.2f}")
            servo.value = clamped_value
        except Exception as e:
            print(f"Error setting servo position: {e}")
            # Handle potential errors during operation if needed
    else:
        print("Error: Servo object not initialized.")


# --- GUI Setup ---
def create_gui():
    """Creates and runs the Tkinter GUI for servo control."""
    root = tk.Tk()
    root.title("Raspberry Pi Servo Control")

    # --- GUI Callback Function ---
    def map_slider_to_servo_value(slider_value_str):
        """Maps the slider's 0-180 range to the servo's -1 to 1 range."""
        try:
            angle = float(slider_value_str)
             # Map 0-180 degrees to -1.0 to 1.0
            # Formula: value = (angle / total_angle_range) * 2.0 - 1.0
            value = (angle / 180.0) * 2.0 - 1.0
            return value
        except ValueError:
             print(f"Invalid slider value received: {slider_value_str}")
             return 0.0 # Return center position on error


    def update_servo(slider_value):
        """Callback for the Servo slider."""
        value = map_slider_to_servo_value(slider_value)
        set_servo_position(value)

    # --- Create Widgets ---
    main_frame = tk.Frame(root, padx=10, pady=10)
    main_frame.pack()

    # Servo Controls
    label = tk.Label(main_frame, text=f"Servo Control (GPIO {SERVO_PIN})")
    label.pack(pady=(0, 5))
    # Using 0-180 for slider range as it often corresponds to standard servo degrees
    # Resolution determines the steps (e.g., 1 means integer steps)
    slider = tk.Scale(main_frame, from_=0, to=180, orient=tk.HORIZONTAL,
                      length=300, resolution=1, command=update_servo)
    slider.set(90) # Start at center position (90 degrees maps to 0.0 value)
    slider.pack(pady=(0, 15))


    # --- Set Initial Position ---
    # Call update once to set servo to the initial slider position
    update_servo(slider.get())

    # --- Start GUI ---
    print("Starting GUI...")
    root.mainloop()

# --- Cleanup Function ---
def cleanup_gpio():
    """Closes servo connections gracefully."""
    print("Exiting application. Cleaning up GPIO...")
    global servo # Ensure we are referencing the global servo object
    global factory
    if servo:
        try:
            servo.value = None # Detach servo (stops sending PWM)
            servo.close()
            print("Servo connection closed.")
        except Exception as e:
            print(f"Error closing servo: {e}")
    if factory:
        try:
            factory.close() # Close pin factory if used
            print("Pin factory closed.")
        except Exception as e:
            print(f"Error closing pin factory: {e}")

    print("GPIO cleanup attempt complete.")

# Register the cleanup function to run when the script exits
atexit.register(cleanup_gpio)

# --- Main Execution ---
if __name__ == "__main__":
    # Check if pigpiod service needs to be started (optional but recommended)
    # You might need to run: sudo systemctl enable pigpiod
    #                      sudo systemctl start pigpiod
    # Or handle this check more robustly depending on your setup.
    print("Make sure the pigpio daemon is running for best results:")
    print(" sudo systemctl start pigpiod")
    print("------------------------------------")

    create_gui() # Start the GUI
