#!/usr/bin/env bash
set -euo pipefail

# Example launcher for SegMAV One-Process (Ackermann + overlay)
# Make sure DISPLAY is set if you want the preview window on the Jetson HDMI
export DISPLAY=${DISPLAY:-:0}
export XAUTHORITY=${XAUTHORITY:-$HOME/.Xauthority}

# Resolve named V4L2 symlink (e.g., /dev/astra_rgb) to /dev/videoN for safety
CAM_DEV=$(readlink -f /dev/astra_rgb 2>/dev/null || echo /dev/video0)

# Use MJPEG @ 1280x720 to keep decoding on CPU & reduce blur/smear
CAM_URI="v4l2://${CAM_DEV}?input-codec=mjpeg&width=1280&height=720&framerate=30"

RX="udpin:0.0.0.0:14601"
TX="udpout:127.0.0.1:14555"

chmod +x ./segmav_one_process.py

./segmav_one_process.py \
  "${CAM_URI}" display://0 \
  --tx-hub "${TX}" \
  --rx-hub "${RX}" \
  --nudge-hz 5.0 \
  --gps-weight 0.5 \
  --vel 1.0 \
  --max-angle 30 \
  --lower-crop 0.45 \
  --l1 3.0 \
  --xtrack-cap-m 1.0 \
  --switch-ahead-m 0.5 \
  --wp-cap-radius 6.0 \
  --yaw-kp 1.0 \
  --max-yaw-rate 0.6 \
  --min-turn-speed 0.3 \
  --log-tx
