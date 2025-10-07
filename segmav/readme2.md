No problem — here’s the full README content directly inline so you can copy-paste it yourself into a file named README.md on your Jetson:

# SegMAV — Jetson-Inference Semantic Segmentation (Minimal Project Layout)

This guide shows a simple, working-on-a-Jetson folder layout for semantic segmentation using **jetson-inference (segNet)**.  
Keep it minimal to start; you can expand later.

---

## 📂 Minimal Project Layout



~/segmav/
├─ venv/ # (optional) Python virtual environment
├─ run_auto.sh # (optional) helper script to launch AUTO nudges
├─ run_guided.sh # (optional) helper script to launch GUIDED blend
├─ backend_segnet.py # segNet wrapper (loads the model, returns a mask)
├─ common_overlay.py # drawing utils (purple mask, centroids, angle)
├─ auto_nudge_segnet_preview.py # AUTO mission + RC1 steering nudges
└─ guided_wp_segnet_preview.py # GUIDED + velocity+yaw (BODY_NED) blend


> The SegNet model itself is installed system-wide with **jetson-inference**.  
> You don’t store big model files in your project folder.

---

## 🧩 What Each File Does

- **`backend_segnet.py`** — wraps `jetson.inference.segNet("fcn-resnet18-cityscapes")`, converts the class map → a binary mask (e.g., sidewalk class).
- **`common_overlay.py`** — draws the Steven Dade–style overlay (purple mask, red polygon, blue half boxes, white centroids/line) and smooths the angle.
- **`auto_nudge_segnet_preview.py`** — opens your camera, shows a preview, and when your 3-pos switch is **HIGH**, sends small **RC1 steering nudges** while **AUTO** runs waypoints.
- **`guided_wp_segnet_preview.py`** — reads the mission, switches to **GUIDED**, streams **SET_POSITION_TARGET_LOCAL_NED** (vx + yaw BODY_NED), and blends the waypoint bearing with segmentation yaw when the switch is **HIGH**.

---

## 🗂 Optional “Nice-to-Have” Folders

If you like to keep things tidy:



~/segmav/
├─ scripts/ # the four .py files above (code)
├─ tools/ # extra one-off utilities (optional)
├─ logs/ # where you save run logs/video (optional)
└─ systemd/ # service files to auto-start on boot (optional)


If you move files into a subfolder (e.g., `scripts/`), adjust your run commands accordingly:
```bash
python3 scripts/auto_nudge_segnet_preview.py ...

⚙️ Install Once (Jetson)
sudo apt-get update
sudo apt-get install -y python3-venv python3-pip python3-opencv
# jetson-inference must be installed separately (per NVIDIA docs)

# optional venv (recommended)
cd ~/segmav
python3 -m venv venv
source venv/bin/activate
pip install --upgrade pip wheel
pip install numpy pymavlink

🚗 Run Examples
AUTO + Nudges (Live Preview)
cd ~/segmav
source venv/bin/activate
python3 auto_nudge_segnet_preview.py \
  --device=/dev/telem1,115200 \
  --camera=v4l2:///dev/video4 \
  --flip=none \
  --network=fcn-resnet18-cityscapes \
  --class-id=8

GUIDED + BODY_NED Velocity + Yaw (Live Preview)
cd ~/segmav
source venv/bin/activate
python3 guided_wp_segnet_preview.py \
  --device=/dev/telem1,115200 \
  --camera=v4l2:///dev/video4 \
  --flip=none \
  --network=fcn-resnet18-cityscapes \
  --class-id=8 \
  --vx=1.2 --acc-radius=2.0 --yaw-blend=0.35

📝 Quick Notes

jetson-inference supplies the model — just pass the name:

--network=fcn-resnet18-cityscapes


Cityscapes IDs: road = 7, sidewalk = 8 → set --class-id accordingly.

Preview window shows Steven’s style: purple mask, red polygon, blue halves, white centroids/line.

Camera path: v4l2:///dev/video4 (change if your camera differs).

MAVLink device: /dev/telem1,115200 (or /dev/ttyUSB0,115200 if your FTDI shows there).

✅ Next Steps

Drop these four scripts into your ~/segmav folder.

Install dependencies once.

Run either AUTO nudges or GUIDED blend with preview on the Jetson.

Tune --yaw-blend, --kp_us_per_deg, and --max_delta_us to fit your rover.
