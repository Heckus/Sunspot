#!/bin/bash

# === Configuration ===
# NetworkManager connection profile name for the AP
AP_PROFILE="PiCamAP"
# Systemd service file name for the streaming application
STREAM_SERVICE="picam-stream.service"
# Set to your main Wi-Fi interface if not wlan0
WIFI_IFACE="wlan0"
# === End Configuration ===

# Check if running as root
if [[ $EUID -ne 0 ]]; then
   echo "Error: This script must be run as root (use sudo)."
   exit 1
fi

# --- Functions ---

enable_ap() {
    echo "[INFO] Attempting to ENABLE AP mode and start service..."

    # 1. Enable the service to start on boot
    echo "[INFO] Enabling service '$STREAM_SERVICE' to start on boot..."
    systemctl enable "$STREAM_SERVICE"
    if [ $? -ne 0 ]; then
        echo "[ERROR] Failed to enable service '$STREAM_SERVICE'. Check 'systemctl status $STREAM_SERVICE'." >&2
        # Continue despite error? Or exit? For now, continue.
    fi

    # 2. Switch NetworkManager to AP mode
    # Check if AP profile exists
    if ! nmcli con show "$AP_PROFILE" > /dev/null 2>&1; then
        echo "[ERROR] NetworkManager profile '$AP_PROFILE' not found. Cannot enable AP mode." >&2
        # If AP mode is critical, exit here
        exit 1
    fi

    # Check if AP profile is already active
    if nmcli -t -f DEVICE,STATE con show --active | grep -q "^$WIFI_IFACE:activated$" && nmcli -t -f NAME con show --active | grep -q "^$AP_PROFILE$"; then
        echo "[WARN] AP profile '$AP_PROFILE' seems to be already active on $WIFI_IFACE."
    else
        # Deactivate other connections on the Wi-Fi interface first
        echo "[INFO] Deactivating other connections on $WIFI_IFACE (if any)..."
        # Use nmcli dev disconnect instead of con down for broader device deactivation
        nmcli device disconnect "$WIFI_IFACE" || echo "[WARN] Could not disconnect $WIFI_IFACE (maybe already disconnected or profile inactive)."
        sleep 1

        echo "[INFO] Activating AP profile '$AP_PROFILE'..."
        nmcli connection up "$AP_PROFILE"
        if [ $? -ne 0 ]; then
            echo "[ERROR] Failed to activate AP profile '$AP_PROFILE'. Check 'nmcli connection show' and network logs." >&2
            # Exit if AP activation fails? Depends on requirements.
            # exit 1
        else
             echo "[INFO] AP profile '$AP_PROFILE' activated."
             sleep 1 # Give it a moment to stabilise
        fi
    fi

    # 3. Start the service
    if systemctl is-active --quiet "$STREAM_SERVICE"; then
        echo "[WARN] Stream service '$STREAM_SERVICE' is already running."
    else
        echo "[INFO] Starting stream service '$STREAM_SERVICE'..."
        systemctl start "$STREAM_SERVICE"
        if [ $? -ne 0 ]; then
            echo "[ERROR] Failed to start service '$STREAM_SERVICE'. Check 'sudo systemctl status $STREAM_SERVICE' and 'sudo journalctl -u $STREAM_SERVICE'." >&2
        else
             echo "[INFO] Service '$STREAM_SERVICE' started."
        fi
    fi

    echo "[INFO] AP mode enable sequence finished."
}

disable_ap() {
    echo "[INFO] Attempting to DISABLE AP mode, stop service, and REBOOT..."

    # 1. Stop the service (This should trigger graceful shutdown in the Python script)
    if ! systemctl is-active --quiet "$STREAM_SERVICE"; then
         echo "[WARN] Stream service '$STREAM_SERVICE' seems to be already stopped."
    else
        echo "[INFO] Stopping stream service '$STREAM_SERVICE'..."
        systemctl stop "$STREAM_SERVICE"
        if [ $? -ne 0 ]; then
            echo "[ERROR] Failed to stop service '$STREAM_SERVICE' cleanly. Check status and logs." >&2
            # Consider trying kill if stop fails after a timeout? For now, just log error.
        else
            echo "[INFO] Service '$STREAM_SERVICE' stopped."
            echo "[INFO] Waiting a few seconds for service to shut down gracefully..."
            sleep 10 # Adjust wait time as needed for your script's cleanup
        fi
    fi

    # 2. Disable the service from starting on boot
    echo "[INFO] Disabling service '$STREAM_SERVICE' from starting on boot..."
    systemctl disable "$STREAM_SERVICE"
    if [ $? -ne 0 ]; then
        echo "[ERROR] Failed to disable service '$STREAM_SERVICE'." >&2
    else
        echo "[INFO] Service '$STREAM_SERVICE' disabled."
    fi

    # 3. Deactivate AP profile
    # Check if profile exists and is active
    if nmcli con show "$AP_PROFILE" > /dev/null 2>&1 && nmcli -t -f DEVICE,STATE con show --active | grep -q "^$WIFI_IFACE:activated$" && nmcli -t -f NAME con show --active | grep -q "^$AP_PROFILE$"; then
        echo "[INFO] Deactivating AP profile '$AP_PROFILE'..."
        nmcli connection down "$AP_PROFILE"
        if [ $? -ne 0 ]; then
            echo "[ERROR] Failed to deactivate AP profile '$AP_PROFILE' cleanly." >&2
        else
             echo "[INFO] AP profile '$AP_PROFILE' deactivated."
             sleep 1 # Give network time
        fi
    else
         echo "[WARN] AP profile '$AP_PROFILE' not found or already inactive."
    fi

    # 4. Reboot the system (Added as requested)
    echo "[INFO] Initiating system reboot NOW..."
    sleep 1 # Short pause before reboot command
    /sbin/reboot
    # If reboot command fails (e.g., permissions), log error.
    echo "[FATAL] Failed to execute reboot command. Check permissions and path." >&2
    exit 1 # Exit with error if reboot fails to execute

    # Script execution should stop here if reboot is successful
}

# Original reboot function - Keep separate or merge if desired,
# but disable_ap now handles the shutdown + reboot sequence.
# reboot_system() { ... }

usage() {
    echo "Usage: sudo $0 {enable|disable}"
    # Removed 'reboot' from usage as it's now part of 'disable'
    exit 1
}

# --- Main Script ---

if [ -z "$1" ]; then
    echo "[ERROR] Missing argument." >&2
    usage
fi

case "$1" in
    enable)
        enable_ap
        ;;
    disable)
        disable_ap
        ;;
    # Removed 'reboot' case
    *)
        echo "[ERROR] Invalid argument '$1'." >&2
        usage
        ;;
esac

# Display final status ONLY if 'enable' was called
if [ "$1" == "enable" ]; then
    echo ""
    echo "----- Current Status After Enable -----"
    echo "Active Connections on '$WIFI_IFACE':"
    nmcli -t -f NAME,DEVICE,STATE con show --active | grep "$WIFI_IFACE" || echo "  No active connection found for $WIFI_IFACE"
    echo "State of AP profile '$AP_PROFILE':"
    nmcli -t -f GENERAL.STATE con show "$AP_PROFILE" 2>/dev/null || echo "  Profile '$AP_PROFILE' not found or inactive."
    echo "State of Stream Service '$STREAM_SERVICE': $(systemctl is-active $STREAM_SERVICE)"
    echo "Enabled status of Stream Service: $(systemctl is-enabled $STREAM_SERVICE)"
    echo "------------------------------------"
fi

exit 0