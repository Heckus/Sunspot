#!/bin/bash

# --- CONFIGURATION: CHANGE THESE TWO VALUES ---
PROJECT_DIR="/home/hecke/"
SERVICE_NAME="ros_container"
# ----------------------------------------------

# Check if the project directory exists
if [ ! -d "$PROJECT_DIR" ]; then
  notify-send "Docker Toggle Error" "Project directory not found at $PROJECT_DIR"
  exit 1
fi


# Navigate to the project directory
cd "$PROJECT_DIR" || exit

# Check if the service is currently running.
if docker compose ps --services --status=running | grep -q "^${SERVICE_NAME}$"; then
  # If it's running, stop it first.
  docker compose down
  # THEN, send the confirmation notification.
  notify-send "Docker Toggler" "Container for '${SERVICE_NAME}' has been stopped."
else
  # If it's not running, start it first.
  docker compose up -d
  # THEN, send the confirmation notification.
  notify-send "Docker Toggler" "Container for '${SERVICE_NAME}' has been started."
fi