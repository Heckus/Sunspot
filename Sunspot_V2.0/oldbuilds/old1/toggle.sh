#!/bin/bash

# === Configuration ===
# NetworkManager connection profile name for the AP
AP_PROFILE="PiCamAP"
# Systemd service file name for the streaming application
STREAM_SERVICE="picam-stream.service"
# The Wi-Fi interface to use for the AP
WIFI_IFACE="wlan0"
# Wait time in seconds between major commands
WAIT_TIME=3 # Reduced wait time slightly, adjust if needed
# === End Configuration ===

# --- Helper Functions ---

# Function to print error messages to stderr
error_msg() {
    echo "[ERROR] $1" >&2
}

# Function to print info messages
info_msg() {
    echo "[INFO] $1"
}

# Function to print warning messages
warn_msg() {
    echo "[WARN] $1" >&2
}

# Function to execute a command and check its exit status
# Usage: run_command "Description" command_to_run args...
run_command() {
    local description="$1"
    shift # Remove description from arguments
    local command_str="$@"

    info_msg "$description..."
    # Use eval carefully; needed here for potential pipes/redirects within commands passed as strings
    # Ensure commands passed do not contain untrusted user input.
    eval "$command_str"
    local exit_code=$?

    if [ $exit_code -ne 0 ]; then
        error_msg "Failed to execute: '$command_str' (Exit code: $exit_code)"
        return 1 # Signal failure
    else
        info_msg "Success: '$description'"
        # Only wait on success to avoid delays during failures
        info_msg "Waiting ${WAIT_TIME}s..."
        sleep "$WAIT_TIME"
        return 0 # Signal success
    fi
}

# --- Main Logic Functions ---

enable_ap() {
    info_msg "--- Enabling AP Mode and Service ---"

    # 1. Check if AP profile exists
    if ! nmcli connection show "$AP_PROFILE" > /dev/null 2>&1; then
        error_msg "NetworkManager profile '$AP_PROFILE' not found. Cannot enable AP mode."
        error_msg "Please create the AP profile first (e.g., using 'nmcli device wifi hotspot ...' or nmtui/nm-connection-editor)."
        exit 1
    fi
    info_msg "AP profile '$AP_PROFILE' found."
    sleep 1

    # 2. Enable the service to start on boot
    run_command "Enable service '$STREAM_SERVICE' on boot" \
        systemctl enable "$STREAM_SERVICE" || return 1 # Stop if enable fails

    # 3. Ensure the AP profile is set to autoconnect (Fix for Problem A)
    run_command "Set AP profile '$AP_PROFILE' autoconnect to 'yes'" \
        nmcli connection modify "$AP_PROFILE" connection.autoconnect yes
    if [ $? -ne 0 ]; then
        error_msg "Failed to set autoconnect=yes on '$AP_PROFILE'. AP might not activate automatically on boot!"
        # Decide whether to continue. Let's warn but continue.
        warn_msg "Continuing enable sequence..."
    fi

    # 4. Activate the AP NetworkManager profile (if not already active)
    if nmcli -t -f DEVICE,STATE con show --active | grep -q "^${WIFI_IFACE}:activated$" && \
       nmcli -t -f NAME con show --active --active | grep -q "^${AP_PROFILE}$"; then
        warn_msg "AP profile '$AP_PROFILE' seems already active on '$WIFI_IFACE'. Skipping activation."
        sleep "$WAIT_TIME" # Still wait even if skipped
    else
        run_command "Activate AP profile '$AP_PROFILE' on '$WIFI_IFACE'" \
            nmcli connection up "$AP_PROFILE" ifname "$WIFI_IFACE" # Be explicit about the interface
        if [ $? -ne 0 ]; then
             error_msg "AP activation failed. Check profile '$AP_PROFILE' configuration and network logs ('journalctl -u NetworkManager')."
             # Continue, service might work on next boot if enable/autoconnect succeeded
        fi
    fi

    # 5. Start the stream service (if not already running)
    if systemctl is-active --quiet "$STREAM_SERVICE"; then
        warn_msg "Stream service '$STREAM_SERVICE' is already running."
        sleep "$WAIT_TIME"
    else
        run_command "Start stream service '$STREAM_SERVICE'" \
            systemctl start "$STREAM_SERVICE"
         if [ $? -ne 0 ]; then
            error_msg "Failed to start service. Check 'sudo systemctl status $STREAM_SERVICE' and 'sudo journalctl -u $STREAM_SERVICE'."
            # Continue
        fi
    fi

    info_msg "--- AP Mode Enable Sequence Finished ---"
}

disable_ap() {
    info_msg "--- Disabling AP Mode, Service, and Rebooting ---"

    # 1. Disable the service from starting automatically on boot (Do this first)
    run_command "Disable service '$STREAM_SERVICE' from starting on boot" \
        systemctl disable "$STREAM_SERVICE"
    # Add verification step for Problem B
    if [ $? -eq 0 ]; then
        # Check if disable command seemed to work
        sleep 1 # Give systemd a moment
        if systemctl is-enabled --quiet "$STREAM_SERVICE"; then
            # It should be disabled, but is-enabled still returns true (0)
             error_msg "Verification Check Failed: Service '$STREAM_SERVICE' still reports as enabled after 'systemctl disable'."
             warn_msg "Proceeding with disable/reboot, but service might restart on boot. Investigate systemd config if issue persists."
             sleep 3 # Pause to see warning
        else
            # is-enabled returns false (>0 exit code), which means it IS disabled.
            info_msg "Verified: Service '$STREAM_SERVICE' is now disabled for next boot."
            sleep "$WAIT_TIME"
        fi
    else
        warn_msg "Could not disable service (command failed or maybe already disabled?). Continuing."
    fi


    # 2. Set AP profile to NOT autoconnect (Crucial for disabling AP mode reliably)
    if nmcli con show "$AP_PROFILE" > /dev/null 2>&1; then
        run_command "Set AP profile '$AP_PROFILE' autoconnect to 'no'" \
            nmcli connection modify "$AP_PROFILE" connection.autoconnect no
        if [ $? -ne 0 ]; then
             error_msg "Failed to set autoconnect=no on '$AP_PROFILE'. AP might reactivate on boot! Check permissions/profile."
             # Critical warning, but continue with deactivation/reboot
        fi
    else
        warn_msg "AP profile '$AP_PROFILE' not found. Cannot modify autoconnect setting."
        sleep "$WAIT_TIME"
    fi

    # 3. Deactivate the AP connection (if active)
    # Check if the profile name is listed as active
    if nmcli -t -f NAME con show --active | grep -q "^${AP_PROFILE}$"; then
         run_command "Deactivate AP profile '$AP_PROFILE'" \
            nmcli connection down "$AP_PROFILE"
         if [ $? -ne 0 ]; then
            error_msg "Failed to cleanly deactivate AP profile '$AP_PROFILE'. Proceeding with reboot."
         fi
    else
        warn_msg "AP profile '$AP_PROFILE' does not appear to be active. Skipping deactivation."
        sleep "$WAIT_TIME"
    fi

    # 4. Stop the service (Do this *after* disabling autostart)
    # This allows the service to run up until the point disable is triggered, e.g., by a button press handled by the service
    if ! systemctl is-active --quiet "$STREAM_SERVICE"; then
        warn_msg "Stream service '$STREAM_SERVICE' appears to be already stopped."
        sleep "$WAIT_TIME"
    else
        run_command "Stop stream service '$STREAM_SERVICE'" \
            systemctl stop "$STREAM_SERVICE"
         if [ $? -ne 0 ]; then
             error_msg "Failed to stop service '$STREAM_SERVICE' cleanly. Check status and logs. Continuing disable process."
             # Don't return, proceed with reboot
         else
             # Give extra time ONLY if stop command was issued and succeeded, allows graceful shutdown
             info_msg "Giving service extra time (5s) to shut down gracefully..."
             sleep 5
         fi
    fi

    # 5. Reboot the system
    info_msg "--- Initiating System REBOOT NOW ---"
    sleep 2 # Short final pause
    # Use systemctl reboot for potentially cleaner shutdown
    # /sbin/reboot should also work fine
    systemctl reboot

    # If reboot command fails (e.g., permissions issues, systemctl issues)
    error_msg "FATAL: Failed to execute reboot command. Check permissions or try '/sbin/reboot'."
    exit 1 # Exit with error if reboot fails to execute

    # Script execution should stop here if reboot is successful
}

usage() {
    echo "Usage: sudo $0 {enable|disable}"
    exit 1
}

# --- Main Script Execution ---

# Check if running as root
if [[ $EUID -ne 0 ]]; then
   error_msg "This script must be run as root (use sudo)."
   exit 1
fi

# Check for argument
if [ -z "$1" ]; then
    error_msg "Missing argument."
    usage
fi

# Process argument
case "$1" in
    enable)
        enable_ap
        ;;
    disable)
        disable_ap
        ;;
    *)
        error_msg "Invalid argument '$1'."
        usage
        ;;
esac

# Display final status ONLY if 'enable' was called and script didn't exit early
if [ "$1" == "enable" ]; then
    echo ""
    echo "--------- Current Status After Enable Attempt ---------"
    echo "Active Connections on '$WIFI_IFACE':"
    # Show more details: Name, UUID, Type, Device, State
    nmcli -t -f NAME,UUID,TYPE,DEVICE,STATE con show --active | grep "$WIFI_IFACE" || echo "  No active connection found for $WIFI_IFACE"
    echo "Details for AP profile '$AP_PROFILE':"
    if nmcli con show "$AP_PROFILE" > /dev/null 2>&1; then
        # Show activation state and autoconnect status
        STATE=$(nmcli -g GENERAL.STATE con show "$AP_PROFILE" 2>/dev/null || echo "N/A")
        AUTOCONNECT=$(nmcli -g connection.autoconnect con show "$AP_PROFILE" 2>/dev/null || echo "N/A")
        DEVICE=$(nmcli -g GENERAL.DEVICES con show "$AP_PROFILE" 2>/dev/null || echo "N/A")
        echo "  Current State: $STATE"
        echo "  Autoconnect: $AUTOCONNECT"
        echo "  Associated Device(s): $DEVICE"
    else
        echo "  Profile '$AP_PROFILE' not found."
    fi
    echo "State of Stream Service '$STREAM_SERVICE': $(systemctl is-active "$STREAM_SERVICE")"
    echo "Enabled status of Stream Service: $(systemctl is-enabled "$STREAM_SERVICE")"
    echo "------------------------------------------------------"
fi

exit 0