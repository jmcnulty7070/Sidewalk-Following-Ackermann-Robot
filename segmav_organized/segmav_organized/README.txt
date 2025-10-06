SegMAV organized bundle (camera=/dev/video4, serial=/dev/telem1)

Folders:
- custom/
    - auto_nudge.py          (AUTO + RC1 steering nudges, CH5 controls OFF/PREVIEW/NUDGE)
    - guided_wp_seg.py       (GUIDED velocity+yaw, blends waypoints with segmentation)
    - yolop_backend.py       (YOLOP drivable-area mask, PyTorch hub)

Quick start:
1) Create venv & install deps:
   sudo apt-get update
   sudo apt-get install -y python3-venv python3-opencv python3-pip
   cd ~/segmav
   python3 -m venv venv
   source venv/bin/activate
   pip install --upgrade pip wheel numpy pymavlink opencv-python
   pip install torch torchvision --extra-index-url https://download.pytorch.org/whl/cu118

2) Run AUTO + nudges:
   cd /mnt/data/segmav_organized/custom
   source ../../segmav/venv/bin/activate  # or your venv path
   python3 auto_nudge.py --device=/dev/telem1,115200 --flip=none

3) Run GUIDED + blend:
   python3 guided_wp_seg.py --device=/dev/telem1,115200 --vx=1.2 --acc-radius=2.0 --yaw-blend=0.35

Notes:
- CH5 LOW<1300=off, 1300-1700=preview, >1700=active
- Window preview appears on the Jetson display (OpenCV).
- Ensure /dev/telem1 udev symlink exists; otherwise use /dev/ttyUSBx.
