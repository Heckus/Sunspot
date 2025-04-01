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
    echo "[INFO] Attempting to ENABLE AP mode..."

    # Check if AP profile is already active
    if nmcli -t -f GENERAL.STATE con show "$AP_PROFILE" 2>/dev/null | grep -q "activated"; then
        echo "[WARN] AP profile '$AP_PROFILE' seems to be already active."
    else
        # Deactivate other connections on the Wi-Fi interface first (optional but safer)
        echo "[INFO] Deactivating other connections on $WIFI_IFACE (if any)..."
        nmcli device disconnect "$WIFI_IFACE" || echo "[WARN] Could not disconnect $WIFI_IFACE (maybe already disconnected)."
        sleep 1 # Give it a moment

        echo "[INFO] Activating AP profile '$AP_PROFILE'..."
        nmcli connection up "$AP_PROFILE"
        if [ $? -ne 0 ]; then
            echo "[ERROR] Failed to activate AP profile '$AP_PROFILE'. Check 'nmcli connection show'." >&2
            # Optionally exit here if AP activation is critical for the service
            # exit 1
        fi
        sleep 1 # Give it a moment to stabilise
    fi

    # Check if service is already active
    if systemctl is-active --quiet "$STREAM_SERVICE"; then
        echo "[WARN] Stream service '$STREAM_SERVICE' seems to be already running."
    else
        echo "[INFO] Starting stream service '$STREAM_SERVICE'..."
        systemctl start "$STREAM_SERVICE"
        if [ $? -ne 0 ]; then
            echo "[ERROR] Failed to start service '$STREAM_SERVICE'. Check 'systemctl status $STREAM_SERVICE'." >&2
        fi
    fi

    echo "[INFO] AP mode enable sequence finished."
}

disable_ap() {
    echo "[INFO] Attempting to DISABLE AP mode..."

    # Check if service is inactive
    if ! systemctl is-active --quiet "$STREAM_SERVICE"; then
         echo "[WARN] Stream service '$STREAM_SERVICE' seems to be already stopped."
    else
        echo "[INFO] Stopping stream service '$STREAM_SERVICE'..."
        systemctl stop "$STREAM_SERVICE"
        if [ $? -ne 0 ]; then
            echo "[ERROR] Failed to stop service '$STREAM_SERVICE' (maybe already stopped?)." >&2
        fi
    fi

    # Check if AP profile is inactive
    # Note: State might be 'activating' or 'deactivating', checking for 'activated' is safer
    if ! nmcli -t -f GENERAL.STATE con show "$AP_PROFILE" 2>/dev/null | grep -q "activated"; then
         echo "[WARN] AP profile '$AP_PROFILE' seems to be already inactive."
    else
        echo "[INFO] Deactivating AP profile '$AP_PROFILE'..."
        nmcli connection down "$AP_PROFILE"
        if [ $? -ne 0 ]; then
            echo "[ERROR] Failed to deactivate AP profile '$AP_PROFILE' (maybe already inactive?)." >&2
        fi
    fi

    echo "[INFO] AP mode disable sequence finished."
    echo "[INFO] You may need to manually activate your normal Wi-Fi client connection (e.g., using 'sudo nmtui' or 'sudo nmcli c up YourHomeProfile')."
}

usage() {
    echo "Usage: sudo $0 {enable|disable}"
    exit 1
}

# --- Main Script ---

# Check for argument
if [ -z "$1" ]; then
    echo "[ERROR] Missing argument." >&2
    usage
fi

# Execute action based on argument
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

# Display final status
echo ""
echo "----- Current Status -----"
echo "Active Network Connections on '$WIFI_IFACE':"
nmcli -t -f NAME,DEVICE,STATE connection show --active | grep "$WIFI_IFACE" || echo "  No active connection found for $WIFI_IFACE"
echo "State of AP profile '$AP_PROFILE':"
nmcli -t -f GENERAL.STATE con show "$AP_PROFILE" 2>/dev/null || echo "  Profile '$AP_PROFILE' not found or inactive."
echo "State of Stream Service '$STREAM_SERVICE': $(systemctl is-active $STREAM_SERVICE)"
echo "-------------------------"

exit 0