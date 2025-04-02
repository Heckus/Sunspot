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

    # --- Optional: Ensure AP profile is set to autoconnect=yes if desired ---
    # If you want the AP to try connecting automatically on boot *after*
    # running 'enable' once (independent of the service enable state),
    # uncomment the following lines. Otherwise, leave commented; the AP
    # will only start when this script is run with 'enable' or the
    # service starts it (if the service handles networking).
    #
    # if nmcli con show "$AP_PROFILE" > /dev/null 2>&1; then
    #     echo "[INFO] Modifying AP profile '$AP_PROFILE' to set connection.autoconnect=yes..."
    #     nmcli connection modify "$AP_PROFILE" connection.autoconnect yes
    #     if [ $? -ne 0 ]; then
    #         echo "[WARN] Failed to modify AP profile '$AP_PROFILE' to enable autoconnect." >&2
    #     else
    #         echo "[INFO] AP profile '$AP_PROFILE' autoconnect enabled."
    #     fi
    #     sleep 1
    # else
    #     echo "[WARN] AP profile '$AP_PROFILE' not found, cannot set autoconnect."
    # fi
    # --- End Optional Autoconnect Section ---


    # 1. Enable the service to start on boot
    echo "[INFO] Enabling service '$STREAM_SERVICE' to start on boot..."
    systemctl enable "$STREAM_SERVICE"
    if [ $? -ne 0 ]; then
        echo "[ERROR] Failed to enable service '$STREAM_SERVICE'. Check 'systemctl status $STREAM_SERVICE'." >&2
        # Continue even if service enable fails for now
    fi

    # 2. Switch NetworkManager to AP mode
    # Check if AP profile exists
    if ! nmcli con show "$AP_PROFILE" > /dev/null 2>&1; then
        echo "[ERROR] NetworkManager profile '$AP_PROFILE' not found. Cannot enable AP mode." >&2
        exit 1
    fi

    # Check if AP profile is already active
    if nmcli -t -f DEVICE,STATE con show --active | grep -q "^$WIFI_IFACE:activated$" && nmcli -t -f NAME con show --active | grep -q "^$AP_PROFILE$"; then
        echo "[WARN] AP profile '$AP_PROFILE' seems to be already active on $WIFI_IFACE."
    else
        # Deactivate other connections on the Wi-Fi interface first
        echo "[INFO] Deactivating other connections on $WIFI_IFACE (if any)..."
        nmcli device disconnect "$WIFI_IFACE" || echo "[WARN] Could not disconnect $WIFI_IFACE (maybe already disconnected or profile inactive)."
        sleep 1

        echo "[INFO] Activating AP profile '$AP_PROFILE'..."
        nmcli connection up "$AP_PROFILE"
        if [ $? -ne 0 ]; then
            echo "[ERROR] Failed to activate AP profile '$AP_PROFILE'. Check 'nmcli connection show' and network logs." >&2
            # Consider exiting if AP activation is critical
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

# --- disable_ap function with autoconnect modification ---
disable_ap() {
    echo "[INFO] Attempting to DISABLE AP mode, stop service, modify profile, and REBOOT..."

    # 1. Stop the service
    if ! systemctl is-active --quiet "$STREAM_SERVICE"; then
         echo "[WARN] Stream service '$STREAM_SERVICE' seems to be already stopped."
    else
        echo "[INFO] Stopping stream service '$STREAM_SERVICE'..."
        systemctl stop "$STREAM_SERVICE"
        if [ $? -ne 0 ]; then
            echo "[ERROR] Failed to stop service '$STREAM_SERVICE' cleanly. Check status and logs." >&2
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

    # 3. Modify AP Profile to prevent autoconnect across reboots
    if nmcli con show "$AP_PROFILE" > /dev/null 2>&1; then
        echo "[INFO] Modifying AP profile '$AP_PROFILE' to set connection.autoconnect=no..."
        nmcli connection modify "$AP_PROFILE" connection.autoconnect no
        if [ $? -ne 0 ]; then
            echo "[ERROR] Failed to modify AP profile '$AP_PROFILE' to disable autoconnect." >&2
            # Continue regardless, as disconnect might still work temporarily
        else
            echo "[INFO] AP profile '$AP_PROFILE' autoconnect disabled."
        fi
    else
        echo "[WARN] AP profile '$AP_PROFILE' not found, cannot modify autoconnect setting."
    fi
    sleep 1 # Brief pause after modification

    # 4. Deactivate AP mode by disconnecting the device
    # Check if the AP profile is currently active on the interface first
    if nmcli -t -f DEVICE,STATE con show --active | grep -q "^$WIFI_IFACE:activated$" && nmcli -t -f NAME con show --active | grep -q "^$AP_PROFILE$"; then
        echo "[INFO] Disconnecting Wi-Fi interface '$WIFI_IFACE' to stop AP mode..."
        nmcli device disconnect "$WIFI_IFACE"
        if [ $? -ne 0 ]; then
            echo "[ERROR] Failed to disconnect device '$WIFI_IFACE' cleanly." >&2
            # Proceed to reboot even if disconnect fails? Yes, for now.
        else
             echo "[INFO] Device '$WIFI_IFACE' disconnected."
             echo "[INFO] Waiting a few seconds for network changes..."
             sleep 3 # Wait after disconnect, before reboot
        fi
    elif nmcli con show "$AP_PROFILE" > /dev/null 2>&1; then
         # Profile exists but isn't active on the interface
         echo "[WARN] AP profile '$AP_PROFILE' exists but is not currently active on $WIFI_IFACE."
         # Attempt disconnect anyway, in case the interface is up in a weird state
         nmcli device disconnect "$WIFI_IFACE" || echo "[WARN] Could not disconnect $WIFI_IFACE (maybe already disconnected)."
         sleep 1
    else
         # Profile doesn't exist or interface isn't managed/active
         echo "[WARN] AP profile '$AP_PROFILE' not found or interface $WIFI_IFACE not active/managed."
    fi

    # 5. Reboot the system
    echo "[INFO] Initiating system reboot NOW..."
    sleep 1 # Short pause before reboot command
    /sbin/reboot
    # If reboot command fails (e.g., permissions), log error.
    echo "[FATAL] Failed to execute reboot command. Check permissions and path." >&2
    exit 1 # Exit with error if reboot fails to execute

    # Script execution should stop here if reboot is successful
}
# --- End disable_ap function ---


usage() {
    echo "Usage: sudo $0 {enable|disable}"
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
    echo "Details for AP profile '$AP_PROFILE':"
    if nmcli con show "$AP_PROFILE" > /dev/null 2>&1; then
        # Show activation state and autoconnect status
        STATE=$(nmcli -g GENERAL.STATE con show "$AP_PROFILE")
        AUTOCONNECT=$(nmcli -g connection.autoconnect con show "$AP_PROFILE")
        echo "  Activation State: $STATE"
        echo "  Autoconnect: $AUTOCONNECT"
    else
        echo "  Profile '$AP_PROFILE' not found."
    fi
    echo "State of Stream Service '$STREAM_SERVICE': $(systemctl is-active $STREAM_SERVICE)"
    echo "Enabled status of Stream Service: $(systemctl is-enabled $STREAM_SERVICE)"
    echo "------------------------------------"
fi

exit 0