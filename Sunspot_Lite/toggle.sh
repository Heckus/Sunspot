#!/bin/bash

# === Configuration ===
# NetworkManager connection profile name for the AP
AP_PROFILE="PiCamAP"
# Systemd service file name for the streaming application
STREAM_SERVICE="picam-stream.service"
# The Wi-Fi interface to use for the AP
WIFI_IFACE="wlan0"
# Wait time in seconds between major commands
WAIT_TIME=5
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
    eval "$command_str" # Use eval to handle commands with pipes/redirects if needed, careful with quoting
    local exit_code=$?

    if [ $exit_code -ne 0 ]; then
        error_msg "Failed to execute: '$command_str' (Exit code: $exit_code)"
        return 1 # Signal failure
    else
        info_msg "Success: '$description'"
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
    sleep 1 # Short pause

    # 2. Enable the service to start on boot
    # We enable it first, so if AP activation fails, it's still set for next boot.
    run_command "Enable service '$STREAM_SERVICE' on boot" \
        systemctl enable "$STREAM_SERVICE" || return 1 # Stop if enable fails

    # 3. Activate the AP NetworkManager profile
    # Check if it's already active on the correct interface
    if nmcli -t -f DEVICE,STATE con show --active | grep -q "^${WIFI_IFACE}:activated$" && \
       nmcli -t -f NAME con show --active --active | grep -q "^${AP_PROFILE}$"; then
        warn_msg "AP profile '$AP_PROFILE' seems already active on '$WIFI_IFACE'. Skipping activation."
        sleep "$WAIT_TIME" # Still wait even if skipped
    else
        # Attempt to bring the connection up
        run_command "Activate AP profile '$AP_PROFILE'" \
            nmcli connection up "$AP_PROFILE"
        # Check specifically if activation failed
        if [ $? -ne 0 ]; then
             error_msg "AP activation failed. Check profile '$AP_PROFILE' configuration and network logs ('journalctl -u NetworkManager')."
             # Decide whether to continue: Let's continue for now, service might work on next boot.
             # return 1
        fi
    fi

    # 4. Start the stream service (if not already running)
    if systemctl is-active --quiet "$STREAM_SERVICE"; then
        warn_msg "Stream service '$STREAM_SERVICE' is already running."
        sleep "$WAIT_TIME" # Still wait
    else
        run_command "Start stream service '$STREAM_SERVICE'" \
            systemctl start "$STREAM_SERVICE"
         if [ $? -ne 0 ]; then
            error_msg "Failed to start service. Check 'sudo systemctl status $STREAM_SERVICE' and 'sudo journalctl -u $STREAM_SERVICE'."
            # Decide whether to continue: Let's continue for now.
            # return 1
        fi
    fi

    info_msg "--- AP Mode Enable Sequence Finished ---"
}

disable_ap() {
    info_msg "--- Disabling AP Mode, Service, and Rebooting ---"

    # 1. Stop the service (if running)
    if ! systemctl is-active --quiet "$STREAM_SERVICE"; then
        warn_msg "Stream service '$STREAM_SERVICE' appears to be already stopped."
        sleep "$WAIT_TIME" # Still wait
    else
        run_command "Stop stream service '$STREAM_SERVICE'" \
            systemctl stop "$STREAM_SERVICE"
         if [ $? -ne 0 ]; then
             error_msg "Failed to stop service '$STREAM_SERVICE' cleanly. Check status and logs. Continuing disable process."
             # Don't return, proceed with disable/reboot
         else
             # Give extra time ONLY if stop command was issued and succeeded
             info_msg "Giving service extra time (10s) to shut down gracefully..."
             sleep 10
         fi
    fi

    # 2. Disable the service from starting automatically on boot
    run_command "Disable service '$STREAM_SERVICE' from starting on boot" \
        systemctl disable "$STREAM_SERVICE" || warn_msg "Could not disable service (maybe already disabled?). Continuing."

    # 3. IMPORTANT: Set AP profile to NOT autoconnect
    if nmcli con show "$AP_PROFILE" > /dev/null 2>&1; then
        run_command "Set AP profile '$AP_PROFILE' autoconnect to 'no'" \
            nmcli connection modify "$AP_PROFILE" connection.autoconnect no
        if [ $? -ne 0 ]; then
             error_msg "Failed to set autoconnect=no on '$AP_PROFILE'. AP might reactivate on boot!"
             # Critical warning, but continue with deactivation/reboot
        fi
    else
        warn_msg "AP profile '$AP_PROFILE' not found. Cannot modify autoconnect setting."
        sleep "$WAIT_TIME" # Still wait
    fi

    # 4. Deactivate the AP connection (if active)
    # Check if the profile name is listed as active
    if nmcli -t -f NAME con show --active | grep -q "^${AP_PROFILE}$"; then
         run_command "Deactivate AP profile '$AP_PROFILE'" \
            nmcli connection down "$AP_PROFILE"
         if [ $? -ne 0 ]; then
            error_msg "Failed to cleanly deactivate AP profile '$AP_PROFILE'. Proceeding with reboot."
            # Don't return, reboot is the final step
         fi
    else
        warn_msg "AP profile '$AP_PROFILE' does not appear to be active. Skipping deactivation."
        sleep "$WAIT_TIME" # Still wait
    fi

    # 5. Reboot the system
    info_msg "--- Initiating System REBOOT NOW ---"
    sleep 2 # Short final pause
    /sbin/reboot

    # If reboot command fails (e.g., permissions issues)
    error_msg "FATAL: Failed to execute reboot command. Check permissions for /sbin/reboot."
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
    nmcli -t -f NAME,DEVICE,STATE con show --active | grep "$WIFI_IFACE" || echo "  No active connection found for $WIFI_IFACE"
    echo "Details for AP profile '$AP_PROFILE':"
    if nmcli con show "$AP_PROFILE" > /dev/null 2>&1; then
        # Show activation state and autoconnect status
        STATE=$(nmcli -g GENERAL.STATE con show "$AP_PROFILE" 2>/dev/null || echo "N/A")
        AUTOCONNECT=$(nmcli -g connection.autoconnect con show "$AP_PROFILE" 2>/dev/null || echo "N/A")
        echo "  Activation State: $STATE"
        echo "  Autoconnect: $AUTOCONNECT"
    else
        echo "  Profile '$AP_PROFILE' not found."
    fi
    echo "State of Stream Service '$STREAM_SERVICE': $(systemctl is-active "$STREAM_SERVICE")"
    echo "Enabled status of Stream Service: $(systemctl is-enabled "$STREAM_SERVICE")"
    echo "------------------------------------------------------"
fi

exit 0