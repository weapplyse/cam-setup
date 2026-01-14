#!/usr/bin/env bash
# camera-dual.sh — Dual IMX519 camera streamer for Jetson Orin Nano
#
# Streams both CAM0 and CAM1 side-by-side in a single stream on port 5000
# Based on camera.sh but modified for dual camera operation
#
# Usage:
#   ./camera-dual.sh              # Default mode 2 (1080p@60)
#   ./camera-dual.sh --mode 0     # Full resolution 4656x3496@9
#   ./camera-dual.sh --mode 1     # 4K 3840x2160@17
#   ./camera-dual.sh --mode 2     # 1080p 1920x1080@60 (default)
#   ./camera-dual.sh --mode 3     # 720p 1280x720@120
#   ./camera-dual.sh --help       # Show help

set -u
set -o pipefail

# -----------------------------
# IMX519 Sensor Mode Presets
# -----------------------------
# Mode 0: 4656x3496@9   - Full resolution, slow
# Mode 1: 3840x2160@17  - 4K, moderate speed
# Mode 2: 1920x1080@60  - 1080p, fast (default)
# Mode 3: 1280x720@120  - 720p, fastest

declare -A MODE_WIDTH=( [0]=4656 [1]=3840 [2]=1920 [3]=1280 )
declare -A MODE_HEIGHT=( [0]=3496 [1]=2160 [2]=1080 [3]=720 )
declare -A MODE_FPS=( [0]=9 [1]=17 [2]=60 [3]=120 )
declare -A MODE_OUT_W=( [0]=1164 [1]=960 [2]=960 [3]=640 )
declare -A MODE_OUT_H=( [0]=874 [1]=540 [2]=540 [3]=360 )
declare -A MODE_BITRATE=( [0]=12000 [1]=10000 [2]=8000 [3]=6000 )
declare -A MODE_DESC=( 
  [0]="Full (4656x3496@9fps)"
  [1]="4K (3840x2160@17fps)"
  [2]="1080p (1920x1080@60fps)"
  [3]="720p (1280x720@120fps)"
)

# -----------------------------
# Parse command line arguments
# -----------------------------
show_help() {
  cat << EOF
Usage: $(basename "$0") [OPTIONS]

Dual IMX519 camera streamer - streams CAM0 and CAM1 side-by-side

Options:
  --mode MODE    Sensor mode (0-3):
                   0 = Full resolution 4656x3496 @ 9fps
                   1 = 4K resolution 3840x2160 @ 17fps
                   2 = 1080p resolution 1920x1080 @ 60fps (default)
                   3 = 720p resolution 1280x720 @ 120fps
  --port PORT    TCP port for stream (default: 5000)
  --bitrate KBPS Encoding bitrate in kbps (default: auto based on mode)
  --help         Show this help message

Environment variables:
  SENSOR_MODE    Same as --mode
  PORT           Same as --port
  BITRATE_KBPS   Same as --bitrate

Examples:
  $(basename "$0")              # 1080p @ 60fps (default)
  $(basename "$0") --mode 1     # 4K @ 17fps
  $(basename "$0") --mode 3     # 720p @ 120fps for high-speed capture
  PORT=5001 $(basename "$0")    # Stream on port 5001

EOF
  exit 0
}

# Parse arguments
while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode)
      SENSOR_MODE="$2"
      shift 2
      ;;
    --port)
      PORT="$2"
      shift 2
      ;;
    --bitrate)
      BITRATE_KBPS="$2"
      shift 2
      ;;
    --help|-h)
      show_help
      ;;
    *)
      echo "Unknown option: $1" >&2
      echo "Use --help for usage information" >&2
      exit 1
      ;;
  esac
done

# -----------------------------
# User-tunable settings
# -----------------------------
PORT="${PORT:-5000}"
BIND_HOST="${BIND_HOST:-0.0.0.0}"

# Sensor mode (can be set via --mode or SENSOR_MODE env var)
SENSOR_MODE="${SENSOR_MODE:-2}"

# Validate sensor mode
if [[ ! "$SENSOR_MODE" =~ ^[0-3]$ ]]; then
  echo "Error: Invalid sensor mode '$SENSOR_MODE'. Must be 0, 1, 2, or 3." >&2
  echo "Use --help for usage information" >&2
  exit 1
fi

# Set resolution and FPS based on sensor mode
SRC_W="${MODE_WIDTH[$SENSOR_MODE]}"
SRC_H="${MODE_HEIGHT[$SENSOR_MODE]}"
FPS="${MODE_FPS[$SENSOR_MODE]}"

# Output resolution per camera (combined will be 2x width)
OUT_W="${OUT_W:-${MODE_OUT_W[$SENSOR_MODE]}}"
OUT_H="${OUT_H:-${MODE_OUT_H[$SENSOR_MODE]}}"

# Bitrate (auto-select based on mode if not specified)
BITRATE_KBPS="${BITRATE_KBPS:-${MODE_BITRATE[$SENSOR_MODE]}}"
KEYINT="${KEYINT:-$FPS}"

# Monitoring / restart behaviour
CHECK_INTERVAL="${CHECK_INTERVAL:-30}"
ERROR_WINDOW="${ERROR_WINDOW:-30}"
STRIKES_BEFORE_RESTART="${STRIKES_BEFORE_RESTART:-2}"
RETRY_DELAY="${RETRY_DELAY:-5}"

# Service name (for journalctl scans)
SERVICE_NAME="${SERVICE_NAME:-camera-dual.service}"

# Logs
LOG_FILE="${LOG_FILE:-/tmp/camera-dual.log}"

# Lock (prevents multiple instances)
LOCKFILE="${LOCKFILE:-/var/lock/camera-dual.lock}"

# -----------------------------
# Helpers
# -----------------------------
log() { echo "[$(date -Is)] $*"; }

have_sudo() { command -v sudo >/dev/null 2>&1 && sudo -n true 2>/dev/null; }

is_uint() { [[ "${1:-}" =~ ^[0-9]+$ ]]; }

restart_nvargus() {
  log "Restarting nvargus-daemon..."
  if have_sudo; then
    sudo -n systemctl restart nvargus-daemon.service 2>/dev/null || sudo -n systemctl restart nvargus-daemon 2>/dev/null || true
    sleep 2
  else
    log "WARNING: sudo not available; cannot restart nvargus-daemon."
  fi
}

release_port() {
  if have_sudo; then
    if sudo -n ss -tlnp 2>/dev/null | grep -qE "LISTEN.*:${PORT}\b"; then
      log "Releasing TCP port ${PORT}..."
      if command -v lsof >/dev/null 2>&1; then
        local pids
        pids="$(sudo -n lsof -t -iTCP:"$PORT" -sTCP:LISTEN 2>/dev/null | tr '\n' ' ')"
        if [[ -n "$pids" ]]; then
          sudo -n kill -TERM $pids 2>/dev/null || true
          sleep 1
          sudo -n kill -KILL $pids 2>/dev/null || true
        fi
      fi
    fi
  fi
}

count_file_tail_matches() {
  local f="$1"; local lines="$2"; local re="$3"
  [[ -f "$f" ]] || { echo 0; return; }
  local n
  n="$(tail -n "$lines" "$f" 2>/dev/null | grep -E "$re" 2>/dev/null | wc -l | tr -d '[:space:]')"
  is_uint "$n" || n=0
  echo "$n"
}

# -----------------------------
# Single-instance lock
# -----------------------------
mkdir -p "$(dirname "$LOCKFILE")" 2>/dev/null || true
exec 9>"$LOCKFILE"
flock -n 9 || { log "Another instance is running; exiting."; exit 0; }

# -----------------------------
# Pipeline control
# -----------------------------
GST_PID=""

start_pipeline() {
  : > "$LOG_FILE" 2>/dev/null || true
  release_port

  # Combined output width (side-by-side)
  local COMBINED_W=$((OUT_W * 2))

  log "Starting dual camera pipeline (CAM0 + CAM1 side-by-side)..."
  log "  Source: ${SRC_W}x${SRC_H}@${FPS} per camera"
  log "  Output: ${COMBINED_W}x${OUT_H} (${OUT_W}x${OUT_H} per camera)"
  log "  Port: ${PORT}"

  # GStreamer pipeline: capture both cameras, scale, composite side-by-side, encode, stream
  (
    exec /usr/bin/gst-launch-1.0 -e \
      nvarguscamerasrc sensor-id=0 sensor-mode="${SENSOR_MODE}" name=cam0 \
      ! "video/x-raw(memory:NVMM),width=${SRC_W},height=${SRC_H},framerate=${FPS}/1,format=NV12" \
      ! nvvidconv \
      ! "video/x-raw(memory:NVMM),width=${OUT_W},height=${OUT_H},format=NV12" \
      ! queue leaky=downstream max-size-buffers=2 \
      ! nvvidconv \
      ! "video/x-raw,format=RGBA" \
      ! queue name=q0 leaky=downstream max-size-buffers=2 \
      ! comp.sink_0 \
      nvarguscamerasrc sensor-id=1 sensor-mode="${SENSOR_MODE}" name=cam1 \
      ! "video/x-raw(memory:NVMM),width=${SRC_W},height=${SRC_H},framerate=${FPS}/1,format=NV12" \
      ! nvvidconv \
      ! "video/x-raw(memory:NVMM),width=${OUT_W},height=${OUT_H},format=NV12" \
      ! queue leaky=downstream max-size-buffers=2 \
      ! nvvidconv \
      ! "video/x-raw,format=RGBA" \
      ! queue name=q1 leaky=downstream max-size-buffers=2 \
      ! comp.sink_1 \
      compositor name=comp \
        sink_0::xpos=0 sink_0::ypos=0 \
        sink_1::xpos="${OUT_W}" sink_1::ypos=0 \
      ! "video/x-raw,width=${COMBINED_W},height=${OUT_H},framerate=${FPS}/1" \
      ! queue leaky=downstream max-size-buffers=2 \
      ! videoconvert \
      ! "video/x-raw,format=I420" \
      ! queue leaky=downstream max-size-buffers=2 \
      ! x264enc tune=zerolatency speed-preset=ultrafast key-int-max="${KEYINT}" bitrate="${BITRATE_KBPS}" threads=0 \
      ! h264parse config-interval=-1 \
      ! mpegtsmux alignment=7 \
      ! queue leaky=downstream max-size-buffers=4 \
      ! tcpserversink host="${BIND_HOST}" port="${PORT}" sync=false
  ) >>"$LOG_FILE" 2>&1 &

  GST_PID="$!"
  sleep 5  # Dual camera needs more time to initialize

  if ! kill -0 "$GST_PID" 2>/dev/null; then
    log "ERROR: gst-launch failed to start."
    tail -n 100 "$LOG_FILE" 2>/dev/null | sed 's/^/[gst] /' || true
    return 1
  fi

  log "Dual camera stream started successfully (PID: ${GST_PID})"
  return 0
}

stop_pipeline() {
  if [[ -n "${GST_PID}" ]] && kill -0 "${GST_PID}" 2>/dev/null; then
    log "Stopping pipeline (PID: ${GST_PID})..."
    kill -INT "${GST_PID}" 2>/dev/null || true
    for _ in {1..20}; do
      kill -0 "${GST_PID}" 2>/dev/null || break
      sleep 0.25
    done
    kill -TERM "${GST_PID}" 2>/dev/null || true
    wait "${GST_PID}" 2>/dev/null || true
  fi
  GST_PID=""
}

# -----------------------------
# Cleanup (idempotent)
# -----------------------------
CLEANED_UP=0
cleanup() {
  [[ "$CLEANED_UP" -eq 1 ]] && return
  CLEANED_UP=1

  trap - INT TERM EXIT

  log "Caught exit signal, cleaning up..."
  stop_pipeline
  release_port
  restart_nvargus
  log "Cleanup done."
}

trap cleanup INT TERM EXIT

# -----------------------------
# Main loop
# -----------------------------
restart_nvargus

log "=== Dual Camera Configuration ==="
log "  Mode: ${SENSOR_MODE} - ${MODE_DESC[$SENSOR_MODE]}"
log "  Source: ${SRC_W}x${SRC_H} @ ${FPS}fps per camera"
log "  Output: $((OUT_W * 2))x${OUT_H} (${OUT_W}x${OUT_H} per camera, side-by-side)"
log "  Bitrate: ${BITRATE_KBPS} kbps"
log "  Port: ${PORT}"
log "================================="

# Default focus value (can be overridden via FOCUS env var)
DEFAULT_FOCUS="${FOCUS:-250}"

set_focus() {
  if [[ -x /home/aspace/set-camera-focus.py ]] || command -v python3 >/dev/null 2>&1; then
    log "Setting focus to ${DEFAULT_FOCUS} on both cameras..."
    if python3 /home/aspace/set-camera-focus.py FOCUS="${DEFAULT_FOCUS}" --all 2>/dev/null; then
      log "Focus set successfully"
    else
      log "WARNING: Failed to set focus (continuing anyway)"
    fi
  fi
}

STRIKES=0

while true; do
  if ! start_pipeline; then
    STRIKES=$((STRIKES + 1))
    log "Pipeline failed to start (strike $STRIKES)"
    restart_nvargus
    sleep "$RETRY_DELAY"
    continue
  fi
  
  # Set focus after pipeline starts (nvargus must be running)
  sleep 2
  set_focus

  # Monitor while gst-launch is alive
  while kill -0 "$GST_PID" 2>/dev/null; do
    sleep "$CHECK_INTERVAL"

    # Error detection
    ERRORS="$(count_file_tail_matches "$LOG_FILE" 200 'Connection reset by peer|Receive worker failure|Argus client is exiting|Unexpected error|AlreadyAllocated|Timeout|Error')"
    is_uint "$ERRORS" || ERRORS=0

    if [[ "$ERRORS" -gt 0 ]]; then
      STRIKES=$((STRIKES + 1))
      log "WARNING: errors detected ($ERRORS) strike=$STRIKES"
      tail -n 50 "$LOG_FILE" 2>/dev/null | sed 's/^/[gst] /' || true

      if [[ "$STRIKES" -ge "$STRIKES_BEFORE_RESTART" ]]; then
        log "ERROR: strike threshold reached -> restarting pipeline + nvargus"
        break
      fi
    else
      if [[ "$STRIKES" -gt 0 ]]; then
        STRIKES=$((STRIKES - 1))
      fi
    fi
  done

  stop_pipeline
  release_port
  restart_nvargus
  sleep "$RETRY_DELAY"
done
