#!/usr/bin/env python3

import tkinter as tk
import sys
import os
import time
import atexit
import signal
import subprocess # To potentially check pin function

# --- Configuration ---
# NOTE: Verify these values for your specific Pi 5 setup!
# Check /sys/class/pwm/ or use tools to find the correct chip/channel for your GPIO.
# Assuming GPIO 12 corresponds to pwmchip0, channel 0. This might be different!
# Common alternatives for GPIO 12/13 on Pi 5 might be pwmchip2/pwmchip3.
PWM_CHIP = 2
PWM_CHANNEL = 0
GPIO_PIN = 12 # The GPIO pin we are intending to use (for reference/checks)

# Servo Configuration
PERIOD_NS = 20000000  # 20ms (50Hz) period in nanoseconds
MIN_DUTY_NS = 500000   # 0.5ms pulse width min (adjust if needed)
MAX_DUTY_NS = 2500000  # 2.5ms pulse width max (adjust if needed)
CENTER_DUTY_NS = 1500000 # 1.5ms pulse width center
DUTY_RANGE_NS = MAX_DUTY_NS - MIN_DUTY_NS

# --- Sysfs PWM Helper Functions ---
PWM_BASE_PATH = f"/sys/class/pwm/pwmchip{PWM_CHIP}/"
PWM_PATH = f"{PWM_BASE_PATH}pwm{PWM_CHANNEL}/"

def run_command(cmd):
    """Runs a shell command and returns its output or raises an exception."""
    try:
        result = subprocess.run(cmd, shell=True, check=True, capture_output=True, text=True)
        return result.stdout.strip()
    except subprocess.CalledProcessError as e:
        print(f"Error running command '{cmd}': {e}", file=sys.stderr)
        print(f"Stderr: {e.stderr}", file=sys.stderr)
        raise
    except FileNotFoundError:
        print(f"Error: Command not found for '{cmd}'. Is the tool installed?", file=sys.stderr)
        raise

def check_pin_function(pin):
    """Attempts to check the function of the GPIO pin (requires raspi-gpio)."""
    # This is an optional check, requires 'raspi-gpio' tool
    try:
        output = run_command(f"raspi-gpio get {pin}")
        print(f"GPIO {pin} Status:\n{output}")
        # You might parse output here to check if it's configured for PWM (e.g., ALT function)
    except Exception as e:
        print(f"Could not check GPIO {pin} function (raspi-gpio tool might be missing or failed): {e}")

def pwm_write(chip, channel, property, value):
    """Writes a value to a PWM sysfs file."""
    path_base = f"/sys/class/pwm/pwmchip{chip}/"
    path_channel = f"{path_base}pwm{channel}/"
    target_path = ""

    if property in ["period", "duty_cycle", "enable", "polarity"]:
         target_path = os.path.join(path_channel, property)
    elif property == "export":
         target_path = os.path.join(path_base, property)
         value = str(channel) # Value for export/unexport is the channel number
    elif property == "unexport":
         target_path = os.path.join(path_base, property)
         value = str(channel)
    else:
        print(f"Error: Unknown PWM property '{property}'")
        return False

    # Check if path exists before writing (except for export/unexport)
    if property not in ["export", "unexport"] and not os.path.exists(target_path):
         print(f"Error: PWM path does not exist: {target_path}", file=sys.stderr)
         print(" Is the PWM channel exported? Is the chip/channel number correct?", file=sys.stderr)
         return False

    try:
        with open(target_path, "w") as f:
            f.write(str(value))
        # print(f"Wrote '{value}' to '{target_path}'") # Debug
        return True
    except IOError as e:
        print(f"Error writing '{value}' to '{target_path}': {e}", file=sys.stderr)
        print(" Check permissions (run with sudo?) and path existence.", file=sys.stderr)
        return False
    except Exception as e:
        print(f"Unexpected error writing to '{target_path}': {e}", file=sys.stderr)
        return False

def pwm_setup():
    """Exports and configures the PWM channel."""
    print(f"Attempting to setup PWM: Chip={PWM_CHIP}, Channel={PWM_CHANNEL} for GPIO {GPIO_PIN}")
    # Optional: Check current pin function
    # check_pin_function(GPIO_PIN)

    # Export the channel if not already exported
    if not os.path.exists(PWM_PATH):
        print(f"Exporting PWM channel {PWM_CHANNEL} on chip {PWM_CHIP}...")
        if not pwm_write(PWM_CHIP, PWM_CHANNEL, "export", PWM_CHANNEL):
             print("Failed to export PWM channel. Exiting.", file=sys.stderr)
             return False
        time.sleep(0.1) # Give sysfs time to create files

    # Check again if path exists after attempting export
    if not os.path.exists(PWM_PATH):
         print(f"Error: PWM path still does not exist after export attempt: {PWM_PATH}", file=sys.stderr)
         return False

    print("Setting PWM period...")
    if not pwm_write(PWM_CHIP, PWM_CHANNEL, "period", PERIOD_NS): return False
    print("Setting initial PWM duty cycle (center)...")
    if not pwm_write(PWM_CHIP, PWM_CHANNEL, "duty_cycle", CENTER_DUTY_NS): return False
    print("Setting PWM polarity to normal...")
    if not pwm_write(PWM_CHIP, PWM_CHANNEL, "polarity", "normal"): return False
    print("Enabling PWM output...")
    if not pwm_write(PWM_CHIP, PWM_CHANNEL, "enable", 1): return False

    print("PWM setup complete.")
    return True

def set_servo_duty_cycle(duty_ns):
    """Sets the servo position by writing to the duty_cycle file."""
    # Clamp value to be safe
    clamped_duty_ns = max(MIN_DUTY_NS, min(MAX_DUTY_NS, int(duty_ns)))
    # print(f"Setting duty cycle to: {clamped_duty_ns} ns") # Debug
    if not pwm_write(PWM_CHIP, PWM_CHANNEL, "duty_cycle", clamped_duty_ns):
        print("Failed to set duty cycle.", file=sys.stderr)


# --- GUI Setup ---
root = None # Make root global for cleanup function if needed
def create_gui():
    """Creates and runs the Tkinter GUI for servo control."""
    global root # Access the global root variable
    root = tk.Tk()
    root.title(f"Raspberry Pi Servo Control (Sysfs PWM{PWM_CHIP}/pwm{PWM_CHANNEL})")

    # --- GUI Callback Function ---
    def map_slider_to_duty_ns(slider_value_str):
        """Maps the slider's 0-180 range to the servo's duty cycle in ns."""
        try:
            angle = float(slider_value_str)
            proportion = angle / 180.0
            duty_ns = MIN_DUTY_NS + proportion * DUTY_RANGE_NS
            return int(duty_ns)
        except ValueError:
             print(f"Invalid slider value received: {slider_value_str}")
             return CENTER_DUTY_NS # Return center position on error

    def update_servo(slider_value):
        """Callback for the Servo slider."""
        duty_ns = map_slider_to_duty_ns(slider_value)
        set_servo_duty_cycle(duty_ns)

    # --- Create Widgets ---
    main_frame = tk.Frame(root, padx=10, pady=10)
    main_frame.pack()

    # Servo Controls
    label = tk.Label(main_frame, text=f"Servo Control (GPIO {GPIO_PIN} via Sysfs PWM{PWM_CHIP}/pwm{PWM_CHANNEL})")
    label.pack(pady=(0, 5))
    slider = tk.Scale(main_frame, from_=0, to=180, orient=tk.HORIZONTAL,
                      length=300, resolution=1, command=update_servo)
    # Set initial slider position to center (90 degrees)
    slider.set(90)
    slider.pack(pady=(0, 15))

    # --- Set Initial Position ---
    # Call update once to set servo to the initial slider position
    print("Setting initial servo position via GUI slider value...")
    update_servo(slider.get())

    # --- Start GUI ---
    print("Starting GUI... Press Ctrl+C in the terminal to exit.")
    # root.mainloop() # We will run this in the main block with error handling

# --- Cleanup Function ---
_cleanup_called = False
def cleanup_pwm(signum=None, frame=None):
    """Disables and unexports the PWM channel."""
    global _cleanup_called
    if _cleanup_called: return
    _cleanup_called = True

    print("\nExiting application. Cleaning up PWM...")
    global root

    # Attempt to close the GUI window gracefully
    if root:
        try:
            root.destroy()
            print("GUI window closed.")
        except tk.TclError as e:
            if "application has been destroyed" not in str(e): print(f"Error closing GUI window: {e}")
        except Exception as e: print(f"Unexpected error closing GUI: {e}")

    # Only try to disable/unexport if the path exists (implies it was exported)
    if os.path.exists(PWM_PATH):
        print("Disabling PWM output...")
        pwm_write(PWM_CHIP, PWM_CHANNEL, "enable", 0)
        time.sleep(0.1) # Short delay before unexporting
        print(f"Unexporting PWM channel {PWM_CHANNEL} on chip {PWM_CHIP}...")
        pwm_write(PWM_CHIP, PWM_CHANNEL, "unexport", PWM_CHANNEL)
    else:
        print("PWM channel path not found, skipping disable/unexport.")

    print("PWM cleanup attempt complete.")
    if signum is not None: sys.exit(0)


# Register the cleanup function
atexit.register(cleanup_pwm)
signal.signal(signal.SIGINT, cleanup_pwm)
signal.signal(signal.SIGTERM, cleanup_pwm)

# --- Main Execution ---
if __name__ == "__main__":
    print("--- Raspberry Pi Servo Control via Sysfs ---")
    print(f"Using PWM Chip: {PWM_CHIP}, Channel: {PWM_CHANNEL} (intended for GPIO {GPIO_PIN})")
    print("IMPORTANT: This script likely requires root privileges (run with 'sudo').")
    print("IMPORTANT: Verify PWM chip/channel mapping for GPIO 12 on your system.")
    print("------------------------------------")

    if os.geteuid() != 0:
        print("Warning: Script not running as root. Sysfs access might fail.", file=sys.stderr)

    # Attempt to set up PWM
    if not pwm_setup():
        print("\nPWM setup failed. Please check errors above.", file=sys.stderr)
        print("Verify permissions, PWM chip/channel numbers, and Device Tree overlays.", file=sys.stderr)
        sys.exit(1)

    # Proceed only if PWM setup was successful
    try:
        create_gui() # Setup the GUI components
        if root:
             root.mainloop() # Start the Tkinter event loop
        else:
             print("Error: GUI root window not created.")

    except KeyboardInterrupt:
        print("\nCtrl+C detected. Initiating cleanup...")
        pass # Cleanup handled by signal/atexit

    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}")

    finally:
        # Ensure cleanup is called if not already done
        if not _cleanup_called:
             print("Performing final cleanup check...")
             cleanup_pwm()

    print("Application finished.")

