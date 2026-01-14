#!/usr/bin/env bash
# camera.sh — stable Jetson (Orin Nano + IMX519) streamer wrapper (self-healing)
#
# Fixes:
# - Avoids Argus "AlreadyAllocated" by enforcing single instance (flock)
# - Forces sensor-mode so Argus doesn't silently jump to 1080p@60
# - Uses leaky queues + downscale before CPU encode (prevents backpressure killing Argus)
# - Robust integer handling (no "integer expression expected")
# - Uses sudo (passwordless) where required (nvargus restart, kernel logs, port cleanup)
# - Idempotent cleanup (no double cleanup / trap spam)
# - Optional consumer-connection watchdog + alerting hooks

set -u
set -o pipefail

# -----------------------------
# User-tunable settings
# -----------------------------
PORT="${PORT:-5000}"
BIND_HOST="${BIND_HOST:-0.0.0.0}"

# Camera selection / modes
SENSOR_ID="${SENSOR_ID:-1}"
# IMX519: sensor-mode=1 is 3840x2160@17 on your device
SENSOR_MODE="${SENSOR_MODE:-1}"

SRC_W="${SRC_W:-3840}"
SRC_H="${SRC_H:-2160}"
FPS="${FPS:-17}"

# Output resolution (downscaled from source for CPU encode stability)
# Options:
#   1920x1080 (1080p) - lighter CPU load, lower quality
#   2560x1440 (1440p) - balanced quality/performance (default)
#   2880x1620 (1620p) - higher quality, more CPU load
#   3840x2160 (4K)    - no downscaling, highest quality but heaviest CPU load
# To disable downscaling completely, set OUT_W="${SRC_W}" and OUT_H="${SRC_H}"
# Can be overridden via camera-config.sh (OUT_W and OUT_H variables)
OUT_W="${OUT_W:-2560}"
OUT_H="${OUT_H:-1440}"

# Load config file if it exists (overrides defaults above)
CONFIG_FILE="${CONFIG_FILE:-/home/aspace/camera-config.sh}"
if [[ -f "$CONFIG_FILE" ]]; then
  # shellcheck disable=SC1090
  source "$CONFIG_FILE"
fi

# x264enc expects kbps (config uses bps, convert if needed)
if [[ -n "${BITRATE:-}" ]]; then
  BITRATE_KBPS=$((BITRATE / 1000))
else
  BITRATE_KBPS="${BITRATE_KBPS:-15000}"
fi
KEYINT="${KEYINT:-17}"

# Argus camera params
EE_MODE="${EE_MODE:-1}"
EE_STRENGTH="${EE_STRENGTH:-0.8}"
# Build exposure range from min/max if set
if [[ -n "${EXPOSURE_MIN:-}" && -n "${EXPOSURE_MAX:-}" ]]; then
  EXPOSURE_RANGE="${EXPOSURE_MIN} ${EXPOSURE_MAX}"
else
  EXPOSURE_RANGE="${EXPOSURE_RANGE:-34000 100000000}"
fi
WBMODE="${WBMODE:-0}"
# Build gain range from min/max if set
if [[ -n "${GAIN_MIN:-}" && -n "${GAIN_MAX:-}" ]]; then
  GAIN_RANGE="${GAIN_MIN} ${GAIN_MAX}"
else
  GAIN_RANGE="${GAIN_RANGE:-1 12}"
fi

# Monitoring / restart behaviour
CHECK_INTERVAL="${CHECK_INTERVAL:-30}"                # seconds
ERROR_WINDOW="${ERROR_WINDOW:-30}"                    # seconds for journal scans
STRIKES_BEFORE_RESTART="${STRIKES_BEFORE_RESTART:-2}" # consecutive error windows
RETRY_DELAY="${RETRY_DELAY:-5}"                       # seconds backoff

# Consumer connection watchdog (optional, helps detect "stream up but nobody consuming")
CONSUMER_SILENT_THRESHOLD="${CONSUMER_SILENT_THRESHOLD:-300}" # seconds
ALERT_SCRIPT="${ALERT_SCRIPT:-/usr/local/bin/send_camera_alert.py}"

# Service name (for journalctl scans). If you run it manually, this may be wrong—but harmless.
SERVICE_NAME="${SERVICE_NAME:-camera-stream.service}"

# Logs
LOG_FILE="${LOG_FILE:-/tmp/camera-stream.log}"

# Lock (prevents multiple instances -> fixes Argus AlreadyAllocated)
LOCKFILE="${LOCKFILE:-/var/lock/camera-stream.lock}"

# -----------------------------
# Helpers
# -----------------------------
log() { echo "[$(date -Is)] $*"; }

have_sudo() { command -v sudo >/dev/null 2>&1 && sudo -n true 2>/dev/null; }

is_uint() { [[ "${1:-}" =~ ^[0-9]+$ ]]; }

run_alert() {
  # usage: run_alert <event> <message>
  local ev="$1"
  local msg="$2"
  if [[ -x "$ALERT_SCRIPT" ]]; then
    python3 "$ALERT_SCRIPT" "$ev" "$msg" 2>/dev/null || true
  fi
}

count_journal_matches() {
  # usage: count_journal_matches <unit> <since> <regex>
  local unit="$1"; local since="$2"; local re="$3"
  local n
  n="$(journalctl -u "$unit" --since "$since" --no-pager 2>/dev/null \
      | grep -E "$re" 2>/dev/null | wc -l | tr -d '[:space:]')"
  is_uint "$n" || n=0
  echo "$n"
}

count_kernel_matches() {
  # usage: count_kernel_matches <since> <regex>
  local since="$1"; local re="$2"
  local n=0
  if have_sudo; then
    n="$(sudo -n journalctl -k --since "$since" --no-pager 2>/dev/null \
        | grep -E "$re" 2>/dev/null | wc -l | tr -d '[:space:]')"
    is_uint "$n" || n=0
  fi
  echo "$n"
}

count_file_tail_matches() {
  # usage: count_file_tail_matches <file> <lines> <regex>
  local f="$1"; local lines="$2"; local re="$3"
  [[ -f "$f" ]] || { echo 0; return; }
  local n
  n="$(tail -n "$lines" "$f" 2>/dev/null | grep -E "$re" 2>/dev/null | wc -l | tr -d '[:space:]')"
  is_uint "$n" || n=0
  echo "$n"
}

restart_nvargus() {
  log "Restarting nvargus-daemon..."
  if have_sudo; then
    sudo -n systemctl restart nvargus-daemon.service 2>/dev/null || sudo -n systemctl restart nvargus-daemon 2>/dev/null || true
  else
    log "WARNING: sudo not available; cannot restart nvargus-daemon."
  fi
}

set_camera_clocks() {
  if have_sudo; then
    log "Setting camera clocks to max..."
    sudo -n sh -c 'echo 1 > /sys/kernel/debug/bpmp/debug/clk/vi/mrq_rate_locked' 2>/dev/null || true
    sudo -n sh -c 'echo 1 > /sys/kernel/debug/bpmp/debug/clk/isp/mrq_rate_locked' 2>/dev/null || true
    sudo -n sh -c 'echo 1 > /sys/kernel/debug/bpmp/debug/clk/nvcsi/mrq_rate_locked' 2>/dev/null || true
    sudo -n sh -c 'echo 1 > /sys/kernel/debug/bpmp/debug/clk/emc/mrq_rate_locked' 2>/dev/null || true
    sudo -n sh -c 'cat /sys/kernel/debug/bpmp/debug/clk/vi/max_rate > /sys/kernel/debug/bpmp/debug/clk/vi/rate' 2>/dev/null || true
    sudo -n sh -c 'cat /sys/kernel/debug/bpmp/debug/clk/isp/max_rate > /sys/kernel/debug/bpmp/debug/clk/isp/rate' 2>/dev/null || true
    sudo -n sh -c 'cat /sys/kernel/debug/bpmp/debug/clk/nvcsi/max_rate > /sys/kernel/debug/bpmp/debug/clk/nvcsi/rate' 2>/dev/null || true
    sudo -n sh -c 'cat /sys/kernel/debug/bpmp/debug/clk/emc/max_rate > /sys/kernel/debug/bpmp/debug/clk/emc/rate' 2>/dev/null || true
    log "Camera clocks set to max"
  else
    log "WARNING: sudo not available; cannot set camera clocks"
  fi
}

release_port() {
  # Kill anything still listening on PORT. Prefer ss+pid, fall back to lsof.
  if have_sudo; then
    if sudo -n ss -tlnp 2>/dev/null | grep -qE "LISTEN.*:${PORT}\b"; then
      log "Releasing TCP port ${PORT}..."
      # Try to kill PIDs bound to the port
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
  else
    # best effort without sudo
    if ss -tln 2>/dev/null | grep -qE "LISTEN.*:${PORT}\b"; then
      log "WARNING: port ${PORT} is listening but no sudo to kill owner."
    fi
  fi
}

get_established_conn_count() {
  # Established TCP connections to the tcpserversink port
  local n
  if have_sudo; then
    n="$(sudo -n ss -tn state established "( sport = :${PORT} )" 2>/dev/null | tail -n +2 | wc -l | tr -d '[:space:]')"
  else
    n="$(ss -tn state established "( sport = :${PORT} )" 2>/dev/null | tail -n +2 | wc -l | tr -d '[:space:]')"
  fi
  is_uint "$n" || n=0
  echo "$n"
}

# -----------------------------
# Flip method handling (kept from your original idea)
# -----------------------------
FLIP_METHOD_VALUE=""
if [[ -f /home/aspace/camera-config.sh ]]; then
  # shellcheck disable=SC1091
  source /home/aspace/camera-config.sh
  # Expect config to set FLIP_METHOD=0..7
  if [[ -n "${FLIP_METHOD:-}" ]]; then
    FLIP_METHOD_VALUE="${FLIP_METHOD}"
    log "Video flip enabled from config: flip-method=${FLIP_METHOD_VALUE}"
  fi
fi

if [[ -z "$FLIP_METHOD_VALUE" ]]; then
  HOSTNAME="$(hostname)"
  if [[ "$HOSTNAME" == *"prod-6"* || "$HOSTNAME" == *"prod-24"* || "$HOSTNAME" == *"prod-51"* ]]; then
    FLIP_METHOD_VALUE="2" # rotate 180
    log "Video rotation (180 degrees) enabled for $HOSTNAME"
  fi
fi

FLIP_METHOD_ARG=""
if [[ -n "$FLIP_METHOD_VALUE" ]]; then
  FLIP_METHOD_ARG="flip-method=${FLIP_METHOD_VALUE}"
fi

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

  # Determine if we need downscaling
  if [[ "$OUT_W" -eq "$SRC_W" && "$OUT_H" -eq "$SRC_H" ]]; then
    log "Starting gst-launch (IMX519 sensor-mode=${SENSOR_MODE}, ${SRC_W}x${SRC_H}@${FPS} - no downscaling)..."
    # Build pipeline without downscaling step
    (
      exec /usr/bin/gst-launch-1.0 -e \
        nvarguscamerasrc \
          sensor-id="${SENSOR_ID}" \
          sensor-mode="${SENSOR_MODE}" \
          ee-mode="${EE_MODE}" \
          ee-strength="${EE_STRENGTH}" \
          exposuretimerange="${EXPOSURE_RANGE}" \
          wbmode="${WBMODE}" \
          gainrange="${GAIN_RANGE}" \
        ! "video/x-raw(memory:NVMM),width=${SRC_W},height=${SRC_H},framerate=${FPS}/1,format=NV12" \
        ! nvvidconv ${FLIP_METHOD_ARG:+$FLIP_METHOD_ARG} \
        ! queue leaky=downstream max-size-buffers=2 max-size-bytes=0 max-size-time=0 \
        ! nvvidconv \
        ! "video/x-raw,format=I420" \
        ! queue leaky=downstream max-size-buffers=2 max-size-bytes=0 max-size-time=0 \
        ! x264enc tune=zerolatency speed-preset=ultrafast key-int-max="${KEYINT}" bitrate="${BITRATE_KBPS}" threads=0 \
        ! h264parse config-interval=-1 \
        ! mpegtsmux alignment=7 \
        ! queue leaky=downstream max-size-buffers=4 max-size-bytes=0 max-size-time=0 \
        ! tcpserversink host="${BIND_HOST}" port="${PORT}" sync=false
    ) >>"$LOG_FILE" 2>&1 &
  else
    log "Starting gst-launch (IMX519 sensor-mode=${SENSOR_MODE}, ${SRC_W}x${SRC_H}@${FPS} -> ${OUT_W}x${OUT_H})..."
    # Build pipeline with downscaling step
    (
      exec /usr/bin/gst-launch-1.0 -e \
        nvarguscamerasrc \
          sensor-id="${SENSOR_ID}" \
          sensor-mode="${SENSOR_MODE}" \
          ee-mode="${EE_MODE}" \
          ee-strength="${EE_STRENGTH}" \
          exposuretimerange="${EXPOSURE_RANGE}" \
          wbmode="${WBMODE}" \
          gainrange="${GAIN_RANGE}" \
        ! "video/x-raw(memory:NVMM),width=${SRC_W},height=${SRC_H},framerate=${FPS}/1,format=NV12" \
        ! nvvidconv ${FLIP_METHOD_ARG:+$FLIP_METHOD_ARG} \
        ! "video/x-raw(memory:NVMM),width=${OUT_W},height=${OUT_H},format=NV12" \
        ! queue leaky=downstream max-size-buffers=2 max-size-bytes=0 max-size-time=0 \
        ! nvvidconv \
        ! "video/x-raw,format=I420" \
        ! queue leaky=downstream max-size-buffers=2 max-size-bytes=0 max-size-time=0 \
        ! x264enc tune=zerolatency speed-preset=ultrafast key-int-max="${KEYINT}" bitrate="${BITRATE_KBPS}" threads=0 \
        ! h264parse config-interval=-1 \
        ! mpegtsmux alignment=7 \
        ! queue leaky=downstream max-size-buffers=4 max-size-bytes=0 max-size-time=0 \
        ! tcpserversink host="${BIND_HOST}" port="${PORT}" sync=false
    ) >>"$LOG_FILE" 2>&1 &
  fi

  GST_PID="$!"
  sleep 2

  if ! kill -0 "$GST_PID" 2>/dev/null; then
    log "ERROR: gst-launch failed to start."
    tail -n 80 "$LOG_FILE" 2>/dev/null | sed 's/^/[gst] /' || true
    return 1
  fi

  log "Camera stream started successfully (PID: ${GST_PID})"
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
# Focus control
# -----------------------------
# Default focus value (can be overridden via FOCUS env var)
DEFAULT_FOCUS="${FOCUS:-250}"

set_focus() {
  if [[ -x /home/aspace/set-camera-focus.py ]] || command -v python3 >/dev/null 2>&1; then
    log "Setting focus to ${DEFAULT_FOCUS} on camera ${SENSOR_ID}..."
    if python3 /home/aspace/set-camera-focus.py FOCUS="${DEFAULT_FOCUS}" --cam "${SENSOR_ID}" 2>/dev/null; then
      log "Focus set successfully"
    else
      log "WARNING: Failed to set focus (continuing anyway)"
    fi
  fi
}

# -----------------------------
# Main loop
# -----------------------------
set_camera_clocks
restart_nvargus

STRIKES=0
LAST_CONN_TS="$(date +%s)"
CONSUMER_ALERTED=0

while true; do
  if ! start_pipeline; then
    STRIKES=$((STRIKES + 1))
    run_alert "camera_start_failed" "gst-launch failed to start on $(hostname) (strike $STRIKES)."
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

    # 1) Consumer connection watchdog (optional)
    CONNS="$(get_established_conn_count)"
    NOW="$(date +%s)"
    if [[ "$CONNS" -gt 0 ]]; then
      LAST_CONN_TS="$NOW"
      if [[ "$CONSUMER_ALERTED" -eq 1 ]]; then
        run_alert "consumer_recovered" "Consumer reconnected to port ${PORT} on $(hostname)."
        CONSUMER_ALERTED=0
      fi
    else
      SILENT_FOR=$((NOW - LAST_CONN_TS))
      if [[ "$SILENT_FOR" -ge "$CONSUMER_SILENT_THRESHOLD" && "$CONSUMER_ALERTED" -eq 0 ]]; then
        run_alert "consumer_missing" "No established consumer connections to port ${PORT} for ${SILENT_FOR}s on $(hostname)."
        CONSUMER_ALERTED=1
      fi
    fi

    # 2) Error detection — journal + logfile + kernel (ISP)
    ARGUS_J_ERRORS="$(count_journal_matches "$SERVICE_NAME" "${ERROR_WINDOW} seconds ago" 'Connection reset by peer|Receive worker failure|Argus client is exiting|Unexpected error in reading socket|AlreadyAllocated|Device .* is in use')"
    ARGUS_F_ERRORS="$(count_file_tail_matches "$LOG_FILE" 200 'Connection reset by peer|Receive worker failure|Argus client is exiting|Unexpected error in reading socket|AlreadyAllocated|Device .* is in use')"
    ISP_ERRORS="$(count_kernel_matches "${ERROR_WINDOW} seconds ago" 'wait for capture status failed|isp process get status failed')"
    SEGFAULTS="$(count_file_tail_matches "$LOG_FILE" 200 'Caught SIGSEGV|Segmentation fault')"

    # Normalize again (belt & suspenders)
    is_uint "$ARGUS_J_ERRORS" || ARGUS_J_ERRORS=0
    is_uint "$ARGUS_F_ERRORS" || ARGUS_F_ERRORS=0
    is_uint "$ISP_ERRORS" || ISP_ERRORS=0
    is_uint "$SEGFAULTS" || SEGFAULTS=0

    if [[ "$ARGUS_J_ERRORS" -gt 0 || "$ARGUS_F_ERRORS" -gt 0 || "$ISP_ERRORS" -gt 0 || "$SEGFAULTS" -gt 0 ]]; then
      STRIKES=$((STRIKES + 1))
      log "WARNING: errors detected (ArgusJournal=$ARGUS_J_ERRORS ArgusLog=$ARGUS_F_ERRORS ISP=$ISP_ERRORS Segfault=$SEGFAULTS) strike=$STRIKES"

      if [[ "$ISP_ERRORS" -gt 0 ]]; then
        run_alert "isp_hardware_error" "ISP errors detected (capture status failed) on $(hostname)."
      else
        run_alert "argus_socket_error" "Argus/socket instability detected on $(hostname)."
      fi

      tail -n 50 "$LOG_FILE" 2>/dev/null | sed 's/^/[gst] /' || true

      if [[ "$STRIKES" -ge "$STRIKES_BEFORE_RESTART" ]]; then
        log "ERROR: strike threshold reached -> restarting pipeline + nvargus"
        break
      fi
    else
      # decay strikes on healthy windows
      if [[ "$STRIKES" -gt 0 ]]; then
        STRIKES=$((STRIKES - 1))
      fi
    fi
  done

  # If we got here, gst died or we broke to restart
  stop_pipeline
  release_port
  restart_nvargus
  sleep "$RETRY_DELAY"
done
