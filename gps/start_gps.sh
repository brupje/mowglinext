#!/bin/bash
# =============================================================================
# GPS startup script — reads config from /config/mowgli_robot.yaml
#
# Launches:
#   1. ublox_gps_node  — u-blox driver on the configured serial port
#   2. ntrip_client    — RTK corrections (if ntrip_enabled is true)
# =============================================================================
set -euo pipefail

CONFIG="/config/mowgli_robot.yaml"

if [ ! -f "$CONFIG" ]; then
  echo "[start_gps.sh] ERROR: $CONFIG not found. Bind-mount config/mowgli/ to /config."
  exit 1
fi

# Parse YAML values using grep+sed (no python dependency).
# Looks for "key: value" under the mowgli: ros__parameters: block.
parse_yaml() {
  grep -E "^\s+${1}:" "$CONFIG" | head -1 | sed 's/.*:\s*//' | tr -d '"' | tr -d "'"
}

GPS_PORT=$(parse_yaml gps_port)
GPS_BAUD=$(parse_yaml gps_baudrate)
NTRIP_ENABLED=$(parse_yaml ntrip_enabled)
NTRIP_HOST=$(parse_yaml ntrip_host)
NTRIP_PORT=$(parse_yaml ntrip_port)
NTRIP_USER=$(parse_yaml ntrip_user)
NTRIP_PASSWORD=$(parse_yaml ntrip_password)
NTRIP_MOUNTPOINT=$(parse_yaml ntrip_mountpoint)

# Defaults
GPS_PORT="${GPS_PORT:-/dev/gps}"
GPS_BAUD="${GPS_BAUD:-921600}"
NTRIP_ENABLED="${NTRIP_ENABLED:-false}"

echo "[start_gps.sh] GPS port: $GPS_PORT @ ${GPS_BAUD} baud"
echo "[start_gps.sh] NTRIP enabled: $NTRIP_ENABLED"

source /opt/ros/jazzy/setup.bash

# Launch ublox_gps in background
ros2 launch ublox_gps ublox_gps_node-launch.py \
  "device_port:=${GPS_PORT}" \
  "uart1.baudrate:=${GPS_BAUD}" &
GPS_PID=$!

# Launch NTRIP client if enabled
if [ "$NTRIP_ENABLED" = "true" ]; then
  echo "[start_gps.sh] Starting NTRIP: ${NTRIP_HOST}:${NTRIP_PORT}/${NTRIP_MOUNTPOINT}"
  # Wait for ublox to initialise before sending corrections
  sleep 5
  ros2 run ntrip_client ntrip_ros \
    --ros-args \
    -p "host:=${NTRIP_HOST}" \
    -p "port:=${NTRIP_PORT}" \
    -p "mountpoint:=${NTRIP_MOUNTPOINT}" \
    -p "username:=${NTRIP_USER}" \
    -p "password:=${NTRIP_PASSWORD}" \
    -p "authentificate:=true" &
  NTRIP_PID=$!
fi

# Wait for any child to exit, then stop everything
wait -n || true
kill "$GPS_PID" 2>/dev/null || true
[ -n "${NTRIP_PID:-}" ] && kill "$NTRIP_PID" 2>/dev/null || true
wait
