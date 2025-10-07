# SegMAV (Jetson-Inference) — defaults baked for your rover

- **Camera:** `v4l2:///dev/video4` (Intel RealSense RGB UVC)
- **MAVLink:** `/dev/telem1,115200` (Jetson FTDI → Cube TELEM1)
- **SegNet:** `fcn-resnet18-cityscapes` (Cityscapes), class **8 = sidewalk** (7 = road)
- **CH5:** 3‑position switch → LOW=off, MID=preview, HIGH=NUDGE/BLEND
- **Preview:** Steven‑style overlay window (purple mask, red polygon, blue halves, white centroids/line)

## Install (once on Jetson)
```bash
sudo apt-get update
sudo apt-get install -y python3-venv python3-pip python3-opencv
# jetson-inference must already be installed

mkdir -p ~/segmav && cd ~/segmav
python3 -m venv venv
source venv/bin/activate
pip install --upgrade pip wheel numpy pymavlink
```

## Files
- `backend_segnet.py` — segNet wrapper
- `common_overlay.py` — overlay + angle smoothing
- `auto_nudge_segnet_preview.py` — **AUTO** + RC1 steering nudges (only when CH5 HIGH)
- `guided_wp_segnet_preview.py` — **GUIDED** + BODY_NED velocity+yaw (blends when CH5 HIGH)

## Run — AUTO mission + sidewalk nudges
```bash
cd ~/segmav
source venv/bin/activate
python3 auto_nudge_segnet_preview.py
```
*Defaults already set; add flags only if you want to change them (e.g., `--flip=rotate-180`).*

## Run — GUIDED + BODY_NED velocity+yaw (blended with sidewalk)
```bash
cd ~/segmav
source venv/bin/activate
python3 guided_wp_segnet_preview.py
```

### Notes
- CH5 LOW = OFF, MID = **preview only**, HIGH = **NUDGE/BLEND**.
- In AUTO, the script only biases **steering** (RC1) gently; ArduPilot flies the mission.
- In GUIDED, it streams `SET_POSITION_TARGET_LOCAL_NED` with **vx** and **yaw** (BODY_NED).
- Use `--class-id=7` if you want to follow the road class instead of sidewalk.
- Needs a GUI session on the Jetson to show the preview window.
