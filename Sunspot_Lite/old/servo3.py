#!/usr/bin/env python3

"""
Minimal library for controlling a single servo using the Linux sysfs PWM interface.
Designed for Raspberry Pi 5, but requires verification of PWM chip/channel mapping.

Usage:
1. Configure PWM_CHIP, PWM_CHANNEL, and Servo settings below.
2. Import the module: `import servo_sysfs_library as servo`
3. Initialize PWM: `if not servo.pwm_setup(): sys.exit(1)`
4. Control servo: `servo.set_servo_angle(90)`
5. Ensure cleanup: Register `servo.cleanup_pwm` with `atexit` or use try/finally.
   `import atexit; atexit.register(servo.cleanup_pwm)`

Requires root privileges (run scripts using this module with 'sudo').
"""

import sys
import os
import time
import subprocess
import atexit

# --- Configuration ---
# NOTE: Verify these values for your specific Pi 5 setup!
# Check /sys/class/pwm/ or use tools to find the correct chip/channel for your GPIO.
# Assuming GPIO 12 corresponds to pwmchip0, channel 0. This might be different!
# Common alternatives for GPIO 12/13 on Pi 5 might be pwmchip2/pwmchip3.
PWM_CHIP = 0
PWM_CHANNEL = 0
GPIO_PIN = 12 # The GPIO pin we are intending to use (for reference)

# Servo Configuration
PERIOD_NS = 20000000  # 20ms (50Hz) period in nanoseconds
MIN_DUTY_NS = 500000   # 0.5ms pulse width min (for angle 0)
MAX_DUTY_NS = 2500000  # 2.5ms pulse width max (for angle 180)
CENTER_DUTY_NS = 1500000 # 1.5ms pulse width center
DUTY_RANGE_NS = MAX_DUTY_NS - MIN_DUTY_NS

# --- Internal State ---
PWM_BASE_PATH = f"/sys/class/pwm/pwmchip{PWM_CHIP}/"
PWM_PATH = f"{PWM_BASE_PATH}pwm{PWM_CHANNEL}/"
_pwm_exported = False # Track if we successfully exported

# --- Sysfs PWM Helper Functions ---

def _pwm_write(property, value):
    """Internal helper to write a value to a PWM sysfs file."""
    global PWM_CHIP, PWM_CHANNEL # Access global config
    path_base = f"/sys/class/pwm/pwmchip{PWM_CHIP}/"
    path_channel = f"{path_base}pwm{PWM_CHANNEL}/"
    target_path = ""

    if property in ["period", "duty_cycle", "enable", "polarity"]:
        target_path = os.path.join(path_channel, property)
    elif property == "export":
        target_path = os.path.join(path_base, property)
        value = str(PWM_CHANNEL) # Value for export/unexport is the channel number
    elif property == "unexport":
        target_path = os.path.join(path_base, property)
        value = str(PWM_CHANNEL)
    else:
        print(f"Error: Unknown PWM property '{property}'", file=sys.stderr)
        return False

    # Check if path exists before writing (except for export/unexport)
    if property not in ["export", "unexport"] and not os.path.exists(target_path):
        print(f"Error: PWM path does not exist: {target_path}", file=sys.stderr)
        print(" Is the PWM channel exported? Is the chip/channel number correct?", file=sys.stderr)
        return False

    try:
        with open(target_path, "w") as f:
            f.write(str(value))
        # print(f"DEBUG: Wrote '{value}' to '{target_path}'") # Uncomment for debug
        return True
    except IOError as e:
        print(f"Error writing '{value}' to '{target_path}': {e}", file=sys.stderr)
        print(" Check permissions (run with sudo?) and path existence.", file=sys.stderr)
        return False
    except Exception as e:
        print(f"Unexpected error writing to '{target_path}': {e}", file=sys.stderr)
        return False

# --- Public Control Functions ---

def pwm_setup():
    """
    Exports and configures the PWM channel for servo control.

    Must be called before using set_servo_angle.

    Returns:
        bool: True on success, False on failure.
    """
    global _pwm_exported, PWM_PATH, PWM_BASE_PATH # Access and potentially modify state

    # Update paths in case chip/channel were changed after import
    PWM_BASE_PATH = f"/sys/class/pwm/pwmchip{PWM_CHIP}/"
    PWM_PATH = f"{PWM_BASE_PATH}pwm{PWM_CHANNEL}/"

    print(f"Attempting to setup PWM: Chip={PWM_CHIP}, Channel={PWM_CHANNEL} (intended for GPIO {GPIO_PIN})")

    # Export the channel if not already exported
    if not os.path.exists(PWM_PATH):
        if not os.path.exists(PWM_BASE_PATH):
             print(f"Error: PWM chip path does not exist: {PWM_BASE_PATH}", file=sys.stderr)
             print(" Is the PWM_CHIP number correct?", file=sys.stderr)
             return False

        print(f"Exporting PWM channel {PWM_CHANNEL} on chip {PWM_CHIP}...")
        if not _pwm_write("export", PWM_CHANNEL):
            print("Failed to export PWM channel.", file=sys.stderr)
            return False
        time.sleep(0.1) # Give sysfs time to create files
        _pwm_exported = True # Assume export worked if write didn't return False
    else:
         print("PWM channel already exported.")
         _pwm_exported = True # Mark as exported if path exists

    # Check again if path exists after attempting export
    if not os.path.exists(PWM_PATH):
        print(f"Error: PWM path still does not exist after export attempt: {PWM_PATH}", file=sys.stderr)
        _pwm_exported = False
        return False

    # Configure PWM properties
    print("Setting PWM period...")
    if not _pwm_write("period", PERIOD_NS): return False
    print("Setting initial PWM duty cycle (center)...")
    if not _pwm_write("duty_cycle", CENTER_DUTY_NS): return False
    print("Setting PWM polarity to normal...")
    if not _pwm_write("polarity", "normal"): return False
    print("Enabling PWM output...")
    if not _pwm_write("enable", 1): return False

    print("PWM setup complete.")
    return True

def set_servo_angle(angle):
    """
    Sets the servo position to the specified angle.

    Args:
        angle (float or int): The desired angle (typically 0 to 180).
                              Values outside this range will be clamped.
    """
    global MIN_DUTY_NS, MAX_DUTY_NS, DUTY_RANGE_NS # Access global config

    if not _pwm_exported:
        print("Error: PWM not setup or export failed. Call pwm_setup() first.", file=sys.stderr)
        return

    # Clamp angle to 0-180 range
    clamped_angle = max(0.0, min(180.0, float(angle)))

    # Map angle to duty cycle
    proportion = clamped_angle / 180.0
    duty_ns = MIN_DUTY_NS + proportion * DUTY_RANGE_NS
    clamped_duty_ns = int(max(MIN_DUTY_NS, min(MAX_DUTY_NS, duty_ns)))

    # print(f"Setting angle {clamped_angle:.1f} -> duty cycle {clamped_duty_ns} ns") # Debug
    if not _pwm_write("duty_cycle", clamped_duty_ns):
        print("Failed to set duty cycle.", file=sys.stderr)


def cleanup_pwm():
    """
    Disables and unexports the PWM channel.

    IMPORTANT: This function MUST be called before your script exits
    (e.g., using atexit.register or a try...finally block) to release
    the PWM channel properly.
    """
    global _pwm_exported
    print("\nCleaning up PWM...")

    # Only try to disable/unexport if the path exists and we think we exported it
    if _pwm_exported and os.path.exists(PWM_PATH):
        print("Disabling PWM output...")
        _pwm_write("enable", 0)
        time.sleep(0.1) # Short delay before unexporting
        print(f"Unexporting PWM channel {PWM_CHANNEL} on chip {PWM_CHIP}...")
        if _pwm_write("unexport", PWM_CHANNEL):
             _pwm_exported = False # Mark as unexported on success
    elif _pwm_exported:
         print("PWM channel path not found, but was marked as exported. Skipping disable/unexport.", file=sys.stderr)
    else:
         print("PWM channel not marked as exported, skipping cleanup actions.")

    print("PWM cleanup attempt complete.")


# --- Example Usage (when run directly) ---
if __name__ == "__main__":
    print("--- Servo Sysfs Library Direct Execution Example ---")
    print("IMPORTANT: This example requires root privileges (run with 'sudo').")
    print("IMPORTANT: Ensure PWM/Servo configurations at the top are correct.")
    print("----------------------------------------------------")

    if os.geteuid() != 0:
        print("Warning: Script not running as root. Sysfs access might fail.", file=sys.stderr)
        # Decide if you want to exit or just warn
        # sys.exit("This script needs root privileges to control PWM via sysfs.")


    # --- Setup ---
    # Register cleanup function to run on exit
    atexit.register(cleanup_pwm)

    # Attempt PWM setup
    if not pwm_setup():
        print("\nPWM setup failed in example.", file=sys.stderr)
        sys.exit(1) # Exit if setup fails

    # --- Control ---
    try:
        print("\nStarting servo movement sequence...")
        angles = [0, 45, 90, 135, 180, 90, 0]
        for angle in angles:
            print(f"Setting angle to: {angle}")
            set_servo_angle(angle)
            time.sleep(1.0) # Wait 1 second between movements

        print("\nSequence complete. Servo at 0 degrees.")
        print("Script will exit shortly, triggering cleanup...")
        time.sleep(2)

    except KeyboardInterrupt:
        print("\nCtrl+C detected during example execution.")
        # Cleanup is handled by atexit

    except Exception as e:
         print(f"\nAn error occurred during the example: {e}")
         # Cleanup should still be attempted by atexit

    # No finally block needed here as atexit handles cleanup

    print("Example finished.")
    # Note: cleanup_pwm() will be called automatically by atexit now

