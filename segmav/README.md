# Minimal SegNet sidewalk project (Jetson-Inference)

Steven Dade–style overlay + jetson-inference **segNet** (Cityscapes).

## Files
- `backend_segnet.py` — segNet wrapper (choose class_id: road=7, sidewalk=8)
- `common_overlay.py` — overlay/drawing + 3-frame angle smoother
- `auto_nudge_segnet_preview.py` — **AUTO** + RC1 steering nudges (CH5 HIGH)
- `guided_wp_segnet_preview.py` — **GUIDED** + BODY_NED velocity+yaw (blends when CH5 HIGH)

## Install (Jetson)
```bash
sudo apt-get update
sudo apt-get install -y python3-venv python3-pip python3-opencv
# jetson-inference installed separately

cd ~/segmav
python3 -m venv venv
source venv/bin/activate
pip install --upgrade pip wheel numpy pymavlink
```

## Run — AUTO + nudges
```bash
cd ~/segmav
source venv/bin/activate
python3 auto_nudge_segnet_preview.py   --device=/dev/telem1,115200   --camera=v4l2:///dev/video4   --flip=none   --network=fcn-resnet18-cityscapes   --class-id=8
```

## Run — GUIDED + BODY_NED
```bash
cd ~/segmav
source venv/bin/activate
python3 guided_wp_segnet_preview.py   --device=/dev/telem1,115200   --camera=v4l2:///dev/video4   --flip=none   --network=fcn-resnet18-cityscapes   --class-id=8   --vx=1.2 --acc-radius=2.0 --yaw-blend=0.35
```
