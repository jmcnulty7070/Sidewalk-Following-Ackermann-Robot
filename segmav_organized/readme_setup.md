# SegMAV — Jetson Orin Nano + Cube Orange Rover

This guide shows how to set up the Jetson, install the SegMAV scripts, and run missions with **semantic segmentation** to keep the rover on the sidewalk while following waypoints.

---

## 1. Make a Python environment and install packages

```bash
sudo apt-get update
sudo apt-get install -y python3-venv python3-pip python3-opencv

# make a project folder (if not already)
mkdir -p ~/segmav && cd ~/segmav
python3 -m venv venv
source venv/bin/activate
pip install --upgrade pip wheel
pip install numpy opencv-python pymavlink
# PyTorch (JetPack/cu118 wheel)
pip install torch torchvision --extra-index-url https://download.pytorch.org/whl/cu118
```

---

## 2. Put the scripts somewhere handy

Copy the folder you downloaded into your home, or just run from where it is:

```bash
# example: copy into your segmav folder
cp -r /mnt/data/segmav_organized ~/segmav_bundle
```

---

## 3. Make sure your serial link is stable

Use the **udev rule** we made so TELEM1 always shows up as `/dev/telem1`.

```bash
ls -l /dev/telem1
```

You should see a symlink pointing to `/dev/ttyUSB0` (or `USB1`).

---

## 4. AUTO mission + sidewalk nudges (CH5 3-pos switch)

1. Upload your mission in **Mission Planner**, set mode to **AUTO**, and **ARM**.
2. On the Jetson (with a display):

```bash
cd ~/segmav_bundle/segmav_organized/custom
source ~/segmav/venv/bin/activate
python3 auto_nudge.py   --device=/dev/telem1,115200   --flip=none   --kp_us_per_deg=4.0 --max_delta_us=120 --deadband_deg=2.0
```

- The window shows your camera (`/dev/video4`).
- **CH5 states**:
  - **LOW (<1300):** OFF (no overrides)
  - **MID (1300–1700):** PREVIEW only
  - **HIGH (>1700):** NUDGE (gentle steering bias on CH1)

---

## 5. GUIDED with velocity + yaw (BODY_NED) + WP blend

Mission must be uploaded to the Cube (the script reads it).

```bash
cd ~/segmav_bundle/segmav_organized/custom
source ~/segmav/venv/bin/activate
python3 guided_wp_seg.py   --device=/dev/telem1,115200   --vx=1.2 --acc-radius=2.0 --yaw-blend=0.35 --flip=none
```

The script will:

1. Pull your mission waypoints.
2. Switch to **GUIDED** and arm.
3. Stream `SET_POSITION_TARGET_LOCAL_NED` at ~15 Hz with forward velocity and yaw (BODY_NED).
4. If **CH5** is HIGH, it blends the **waypoint bearing** with the **segmentation-derived yaw** to stay on the sidewalk.

---

## Notes & Tuning

- **Camera:** Both scripts open `/dev/video4` and show a live preview with the mask inset.  
  Change `--flip` if needed: `rotate-180`, `flip-horizontal`, `flip-vertical`, or `none`.
- **CH5 switch:** Used **only by the scripts** (ArduPilot does not need any `RC5_OPTION`).
- **AUTO nudge gains:** Start gentle:
  - `--kp_us_per_deg=4.0`
  - `--max_delta_us=120`
  - `--deadband_deg=2.0`
- **GUIDED blend:** `--yaw-blend=0.35` = 35 % segmentation + 65 % waypoint course. Adjust `0.2–0.5` to taste.
- **LiDAR/BendyRuler:** Your earlier parameters remain valid (`SERIAL4_PROTOCOL=9`, `RNGFND1_TYPE=20`, `OA_TYPE=1`, etc.).
- **PyTorch:** On first run, YOLOP weights download automatically (requires internet once).

---

## Quick reminder

- CH5 LOW = **Stop**  
- CH5 MID = **Preview only**  
- CH5 HIGH = **Segmentation active**

Your **Cube Orange Rover** runs the mission from Mission Planner while the Jetson applies gentle steering corrections to stay on the sidewalk.
