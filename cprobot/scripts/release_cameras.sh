#!/usr/bin/env bash
# Kill any leftover run.sh camera streams so rpicam-app / rpicam-hello can acquire the camera.
# Run this on the HOST (not inside the container). If you get "failed to acquire camera", run: bash scripts/release_cameras.sh

set -euo pipefail

do_kill() {
  local sudo="$1"
  # 1) Kill what's on the stream ports (ffmpeg listeners)
  $sudo fuser -k 5000/tcp 5001/tcp 2>/dev/null || true
  # 2) Kill the whole process group for each rpicam-vid (stops the "while true" loop that restarts it)
  for pid in $(pgrep -x rpicam-vid 2>/dev/null || true); do
    [[ -z "$pid" ]] && continue
    pgid=$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')
    [[ -n "$pgid" ]] && $sudo kill -TERM -"$pgid" 2>/dev/null || true
  done
  $sudo pkill -x rpicam-vid 2>/dev/null || true
  $sudo pkill -f flask_stream.py 2>/dev/null || true
}

echo "[release_cameras] Stopping processes using cameras / stream ports (run on host, not in container)..."

do_kill ""

sleep 1
if pgrep -x rpicam-vid >/dev/null 2>&1; then
  echo "[release_cameras] Still running — trying with sudo (streams may have been started as root)..."
  do_kill "sudo"
  sleep 1
fi

if pgrep -x rpicam-vid >/dev/null 2>&1; then
  echo "[release_cameras] rpicam-vid still running. Try: sudo bash scripts/release_cameras.sh"
  exit 1
fi

echo "[release_cameras] Done. Try rpicam-app or rpicam-hello now."
