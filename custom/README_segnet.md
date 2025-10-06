# SegNet sidewalk segmentation with Steven-style preview

These scripts keep **Steven Dade's look** (purple mask, red polygon, blue half-boxes, white centroids & line, angle text) and use **jetson-inference segNet** on the Jetson.

- `custom/auto_nudge_segnet_preview.py` — **AUTO mission**, CH5=HIGH applies gentle **RC1 steering nudges** (AUTO still handles waypoints).
- `custom/guided_wp_segnet_preview.py` — **GUIDED**, streams **SET_POSITION_TARGET_LOCAL_NED** (velocity + yaw in BODY frame) and blends **waypoint bearing** with **segmentation yaw** when CH5=HIGH.

## Install (once)

```bash
sudo apt-get update
sudo apt-get install -y python3-pip python3-opencv
pip3 install pymavlink numpy
# jetson-inference should already be installed per NVIDIA docs
```

## Run (AUTO + nudges)

```bash
python3 custom/auto_nudge_segnet_preview.py   --device=/dev/telem1,115200   --camera=v4l2:///dev/video4   --flip=none   --network=fcn-resnet18-cityscapes   --class-id=8
```
- CH5 LOW: OFF, MID: preview, HIGH: nudges (RC override on CH1).

## Run (GUIDED + blend)

```bash
python3 custom/guided_wp_segnet_preview.py   --device=/dev/telem1,115200   --camera=v4l2:///dev/video4   --flip=none   --network=fcn-resnet18-cityscapes   --class-id=8   --vx=1.2 --acc-radius=2.0 --yaw-blend=0.35
```
- CH5 LOW: OFF, MID: preview, HIGH: blend waypoint course with segmentation yaw.

> **Cityscapes class IDs** commonly used: `road=7`, `sidewalk=8`. If your model maps differently, change `--class-id`.
