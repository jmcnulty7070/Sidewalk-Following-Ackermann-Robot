SegMAV One-Process (Ackermann + Segmentation Overlay)
=====================================================

What this does
--------------
* Pulls mission from the FCU over the RX hub.
* Computes course-to-line guidance between waypoints (L1-style).
* Uses Jetson `segNet` (FCN-ResNet18 Cityscapes) to find sidewalk/road.
* Blends the waypoint vector with the segmentation vector.
* Sends BODY_NED forward velocity (vx) **and** yaw-rate (r) — ideal for Ackermann.
* Shows a preview window with **segmentation overlay** and HUD.

Requirements
------------
* Jetson with `jetson-inference` / `jetson-utils` installed.
* `pymavlink` installed.
* MAVLink router/telem endpoints:
  - RX hub (for reading telemetry/mission), e.g. `udpin:0.0.0.0:14601`
  - TX hub (for sending setpoint), e.g. `udpout:127.0.0.1:14555`
* Camera reachable by V4L2 (e.g., `/dev/video0` or symlink `/dev/astra_rgb`)

Install
-------
1) Copy these files to your Jetson:
   - segmav_one_process.py
   - run_segmav.sh
   - README_SegMAV.txt
2) Make them executable:
   chmod +x segmav_one_process.py run_segmav.sh

Run (with overlay)
------------------
./run_segmav.sh

Or run manually:
  export DISPLAY=:0
  ./segmav_one_process.py \
    "v4l2:///dev/video0?input-codec=mjpeg&width=1280&height=720&framerate=30" display://0 \
    --tx-hub udpout:127.0.0.1:14555 \
    --rx-hub udpin:0.0.0.0:14601 \
    --gps-weight 0.5 --vel 1.0 --yaw-kp 1.0 --max-yaw-rate 0.6 --log-tx

Notes
-----
* If you were seeing blur: use MJPEG with 720p @ 30 FPS as above.
* If `v4l2:///dev/astra_rgb` fails to parse, the script resolves it to the real `/dev/videoN`.
* RC5 High (>=1700 by default) enables TX. Toggle `[p]` to pause/resume. `[r]` refetches mission/params.
* For Rover firmware that ignores yaw-rate, steering will not occur. This script **does** populate yaw-rate.
  If you still go straight, verify:
    - MODE is GUIDED (or appropriate mode that accepts SET_POSITION_TARGET_LOCAL_NED)
    - AHRS yaw is valid (ATTITUDE arriving)
    - Firmware uses yaw-rate from SET_POSITION_TARGET_LOCAL_NED (some older builds ignore it)

Tuning
------
* --gps-weight : 0..1 (higher = follow sidewalk more, lower = follow mission more)
* --yaw-kp     : yaw-rate = yaw_kp * yaw_error (clamped by --max-yaw-rate)
* --max-angle  : limits the segmentation-induced heading change
* --lower-crop : fraction of image used at the bottom to compute sidewalk centroid

Hotkeys
-------
  [r]  refetch mission/params
  [p]  pause/resume TX
  [q]  quit

Troubleshooting
---------------
* No window?  Ensure DISPLAY is set and X runs: `export DISPLAY=:0 ; xdpyinfo | head -n5`
* No camera?  Try: `v4l2-ctl --list-devices` and test with `video-viewer v4l2:///dev/video0 display://0`
* Blurry?     Keep MJPEG, avoid hardware debayer; reduce exposure, use fixed focus if possible.
* No steering? Add `--log-tx` and confirm you see non-zero `r=...` in the console.
