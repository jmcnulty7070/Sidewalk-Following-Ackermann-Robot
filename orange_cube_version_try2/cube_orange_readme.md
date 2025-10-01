
# Cube Orange + Jetson SegMAV (AUTO + vision nudges)
**Last updated:** 2025-10-01

This README gives you a clean, copy‑paste setup to run an **AUTO waypoint mission** on a Cube Orange while a **Jetson** does light **vision “nudges”** on steering to keep the car centered on a sidewalk/path — *SegMAV‑style*, with a live overlay.

You’ll get:
- Full AUTO + vision‑nudge **Python script** with overlay
- Optional **YOLOP** backend file (for drivable‑area masks)
- **Complete wiring map** (GPS / telemetry / Jetson / LiDAR / servo/ESC)
- **ArduPilot parameter** changes (outputs, serials, OA, rangefinder, E‑stop)
- **TFmini** UART & I²C setup notes (**including example hex** and gotchas)
- **FS‑i6X** channel mapping and switch setup
- **Run commands**, validation steps, and troubleshooting
- A ready‑to‑use **mavlink‑router** config (plus your earlier variant)

If you want me to tailor defaults (RC channels, serial speeds, camera source), say the word and I’ll update this file.

---

## 1) Top‑level picture (8th‑grade level steps)
**Goal:** Drive a waypoint mission in **AUTO** while the Jetson “bumps” steering a tiny bit to stay on the sidewalk.

1) **Plan the mission** in Mission Planner and upload it to the Cube.
2) **Start mavlink‑router** on the Jetson so apps can talk to the Cube.
3) **Run the Jetson vision script** — it shows the camera and a white line angle.
4) **Arm** and **start AUTO** in Mission Planner.
5) Flip your **3‑position switch HIGH** to enable vision nudges.  
   - LOW = off, MID = preview only, HIGH = nudges on.
6) The car follows waypoints **and** gets small steering bumps from vision.

> Tip: It should feel like gentle lane‑keeping, *not* a takeover.

---

## 2) Wiring map (Cube Orange)
**Power:** Use a **UBEC 6 V** (5–10 A) to power the servo rail. Share grounds between Jetson, Cube, and servo rail.

| Subsystem | Connect to | Notes |
|---|---|---|
| **GPS + Compass** (e.g., Matek M10Q‑5883) | **GPS1** JST (GPS) and **I2C** (Compass) | GPS1 provides 5 V and UART; compass on I²C port (SCL/SDA). |
| **Telemetry radio (SiK)** | **TELEM1** | Typical baud 57, 115, or 57600 depending on radio. |
| **Jetson ↔ Cube** | **TELEM2** via **FTDI USB‑TTL** to Jetson USB | Or use mavlink‑router on Jetson and point scripts to `udp:127.0.0.1:14555`. |
| **Steering servo** | **MAIN OUT 1** | Set `SERVO1_FUNCTION=26`. |
| **ESC / motor** | **MAIN OUT 3** | Set `SERVO3_FUNCTION=70`. |
| **TFmini LiDAR** | **UART (SERIAL4)** *or* **I²C** | Choose one interface; see LiDAR section. |
| **Camera** | Jetson **USB** (`/dev/video0`) or **CSI** ribbon (`csi://0`) | Use whichever you actually have. |

---

## 3) ArduPilot parameter changes (Rover)
Set these in **Mission Planner → Config/Tuning → Full Parameter Tree**, then **Write Params**:

### Outputs
- `SERVO1_FUNCTION = 26`  *(steering on OUT1)*  
- `SERVO3_FUNCTION = 70`  *(throttle on OUT3)*

### Telemetry / serials
- **SiK radio on TELEM1**: `SERIAL1_PROTOCOL = 2`, `SERIAL1_BAUD = 57` *(or 115)*
- **Jetson on TELEM2**: `SERIAL2_PROTOCOL = 2`, `SERIAL2_BAUD = 57` *(or 57600 to match FTDI/router)*

### Rangefinder (TFmini) — choose **one** path
**UART (recommended for TFmini/TFmini‑S):**
- `SERIAL4_PROTOCOL = 9`
- `SERIAL4_BAUD = 115` *(115200 bps)*
- `RNGFND1_TYPE = 20` *(Benewake TFmini)*
- `RNGFND1_MIN_CM = 30` *(example)*
- `RNGFND1_MAX_CM = 1200` *(example)*
- `RNGFND1_ORIENT = 25` *(forward)*

**I²C (if your unit supports I²C mode):**
- `RNGFND1_TYPE = 25` *(Benewake I²C)*
- `RNGFND1_ADDR = 16` *(0x10 default on many Benewake units)*
- Keep `SERIAL4_PROTOCOL` unused or set to `-1` if not using UART.

> Note: Benewake models vary; verify your exact model’s address and mode.

### E‑stop / safety
- Map a 2‑pos switch (e.g., **CH8**) to Emergency Stop: `RC8_OPTION = 31`

### Obstacle Avoidance (optional, if you later want full OA)
- Bendy Ruler & speed‑down are beyond “nudges”; start simple first.

---

## 4) FS‑i6X radio mapping
**Mode‑2** transmitter, bound to **FS‑iA6B** receiver.

**Functions Setup → Aux. Channels:**
- **CH9** → **SWC** (3‑pos) — SegMAV control  
  - LOW ≈ 1000 µs → **OFF**  
  - MID ≈ 1500 µs → **PREVIEW ONLY** (no overrides)  
  - HIGH ≈ 2000 µs → **NUDGES ON**
- **CH8** → **SWB** (2‑pos) — **Emergency Stop** (maps to `RC8_OPTION=31` in ArduPilot)

Check in **MP → Status → RCINx** that the PWM values move as expected.

---

## 5) Jetson folder layout
Create this on the Jetson:
```
~/segmav_auto/
├─ venv/                  # Python virtual environment
├─ requirements.txt
├─ mavsegmav_auto_nudge.py
├─ yolop_backend.py       # optional (for drivable-area masks)
├─ run_segmav.sh
└─ segmav.service         # optional (systemd)
```

### Install prerequisites (once)
```bash
sudo apt-get update
sudo apt-get install -y python3-venv python3-pip python3-opencv

mkdir -p ~/segmav_auto && cd ~/segmav_auto
python3 -m venv venv
source venv/bin/activate

# requirements
cat > requirements.txt << 'EOF'
numpy
opencv-python
pymavlink
# Optional better segmentation (YOLOP):
torch
torchvision
EOF

# Install (JetPack 5.x often needs CUDA 11.8 wheels for torch)
pip install --upgrade pip wheel
pip install --extra-index-url https://download.pytorch.org/whl/cu118 -r requirements.txt || true
# If torch fails now, comment it out in requirements.txt and re-run:
# pip install -r requirements.txt
```

---

## 6) Code — optional YOLOP backend
**File:** `~/segmav_auto/yolop_backend.py`
```python
import cv2, torch, numpy as np

class YOLOPBackend:
    """YOLOP drivable-area backend. Returns uint8 mask (255=navigable)."""
    def __init__(self, device=None, half=True):
        self.device = torch.device(device or ("cuda" if torch.cuda.is_available() else "cpu"))
        # first call downloads weights (internet required once)
        self.model = torch.hub.load("hustvl/YOLOP", "yolop", pretrained=True).to(self.device).eval()
        self.half = half and self.device.type == "cuda"
        if self.half:
            self.model.half()
        self.inp_w, self.inp_h = 640, 384

    @torch.inference_mode()
    def infer_mask(self, bgr_img: np.ndarray) -> np.ndarray:
        h0, w0 = bgr_img.shape[:2]
        img = cv2.resize(bgr_img, (self.inp_w, self.inp_h))
        img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        t = torch.from_numpy(img).permute(2,0,1).unsqueeze(0).to(self.device)
        if self.half: t = t.half()
        # (detect, drivable_area, lane_lines)
        _, da_seg_out, _ = self.model(t)
        da = torch.sigmoid(da_seg_out).squeeze().float().cpu().numpy()
        mask = (da > 0.5).astype(np.uint8) * 255
        return cv2.resize(mask, (w0, h0), interpolation=cv2.INTER_NEAREST)
```

---

## 7) Code — AUTO + vision nudges (overlay)
**File:** `~/segmav_auto/mavsegmav_auto_nudge.py`
```python
#!/usr/bin/env python3
import os, time, math, argparse, collections
import numpy as np
import cv2
from pymavlink import mavutil

# Optional YOLOP backend
try:
    from yolop_backend import YOLOPBackend
    YOLOP_AVAILABLE = True
except Exception:
    YOLOP_AVAILABLE = False

def parse_args():
    p = argparse.ArgumentParser(description="AUTO mission + Vision Steering Nudges (SegMAV-style)")
    p.add_argument("input", type=str, help="camera input (e.g., v4l2:///dev/video0 or csi://0)")
    p.add_argument("output", type=str, help="display://0 | file:///path.mp4")
    p.add_argument("--backend", choices=["yolop","segnet"], default="yolop",
                   help="vision backend: yolop (default) or segnet (stub)")
    p.add_argument("--device", type=str, default="udp:127.0.0.1:14555",
                   help="MAVLink device, e.g. udp:127.0.0.1:14555 or /dev/ttyUSB0,57600")
    p.add_argument("--rc", type=int, default=9, help="RC channel for 3-pos switch (LOW/MID/HIGH)")
    p.add_argument("--steer-ch", type=int, default=1, help="Steering PWM channel (CH1)")
    p.add_argument("--neutral", type=int, default=1500, help="Steering neutral PWM")
    p.add_argument("--pwm-max-delta", type=int, default=120, help="Max steering delta (µs)")
    p.add_argument("--kp", type=float, default=4.0, help="µs per degree of angle error")
    p.add_argument("--deadband-deg", type=float, default=2.0, help="ignore small errors")
    p.add_argument("--send-hz", type=float, default=15.0, help="RC override rate (Hz)")
    p.add_argument("--input-flip", type=str, default="none",
                   help="rotate-180|flip-horizontal|flip-vertical|none")
    return p.parse_args()

def open_input(src: str):
    if src.startswith("v4l2:///"):
        path = src[len("v4l2:///"):]
        cap = cv2.VideoCapture(path)
    elif src.startswith("csi://"):
        idx = int(src[len("csi://"):])
        cap = cv2.VideoCapture(idx)
    else:
        cap = cv2.VideoCapture(src)
    if not cap.isOpened():
        raise RuntimeError(f"Failed to open input: {src}")
    return cap

def open_output(sink: str, w: int, h: int, fps: float):
    if sink.startswith("display://"):
        return ("display", None)
    elif sink.startswith("file://"):
        path = sink[len("file://"):]
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        ts = time.strftime("-%Y%m%d-%H%M%S")
        base, ext = os.path.splitext(path)
        path2 = base + ts + (ext if ext else ".mp4")
        vw = cv2.VideoWriter(path2, fourcc, fps, (w, h))
        if not vw.isOpened():
            raise RuntimeError(f"Failed to open file writer: {path2}")
        return ("file", vw)
    else:
        return ("display", None)

def write_frame(out_mode, writer, frame):
    if out_mode == "display":
        cv2.imshow("SegMAV AUTO Nudges", frame)
        cv2.waitKey(1)
    elif out_mode == "file":
        writer.write(frame)

def open_mav(dev: str):
    if dev.startswith("/dev/") and "," in dev:
        port, baud = dev.split(",", 1)
        m = mavutil.mavlink_connection(port, baud=int(baud))
    else:
        m = mavutil.mavlink_connection(dev)
    m.wait_heartbeat()
    return m

def get_rc_channels(m, timeout=0.1):
    msg = m.recv_match(type="RC_CHANNELS", blocking=True, timeout=timeout)
    if not msg: return {}
    return {
        1: msg.chan1_raw, 2: msg.chan2_raw, 3: msg.chan3_raw, 4: msg.chan4_raw,
        5: msg.chan5_raw, 6: msg.chan6_raw, 7: msg.chan7_raw, 8: msg.chan8_raw
    }

def rc_override(m, ch_to_pwm: dict):
    # 8 channels; 0 = no override, 1000..2000 set value
    chs = [0]*8
    for ch, pwm in ch_to_pwm.items():
        if 1 <= ch <= 8 and pwm is not None:
            chs[ch-1] = int(pwm)
    m.mav.rc_channels_override_send(m.target_system, m.target_component, *chs)

def release_override(m):
    rc_override(m, {})  # all zeros

def flip_image(img, mode: str):
    if mode == "rotate-180":      return cv2.rotate(img, cv2.ROTATE_180)
    if mode == "flip-horizontal": return cv2.flip(img, 1)
    if mode == "flip-vertical":   return cv2.flip(img, 0)
    return img

def largest_contour(mask):
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts: return None
    return max(cnts, key=cv2.contourArea)

def centroid_of_region(submask):
    M = cv2.moments(submask, binaryImage=True)
    if M["m00"] <= 0: return None
    cx = int(M["m10"] / M["m00"]); cy = int(M["m01"] / M["m00"])
    return (cx, cy)

class AngleSmoother3:
    def __init__(self): self.buf = collections.deque(maxlen=3)
    def add(self, deg): self.buf.append(deg); return sum(self.buf)/len(self.buf) if self.buf else 0.0

def compute_angle_deg(mask, vis):
    """mask: uint8 0/255 drivable region. Split top/bottom, find centroids, compute angle."""
    h, w = mask.shape[:2]
    cnt = largest_contour(mask)
    if cnt is None or cv2.contourArea(cnt) < 500:
        return 0.0, vis

    cv2.drawContours(vis, [cnt], -1, (0,0,255), 2)

    midy = h // 2
    top = mask[:midy, :]
    bot = mask[midy:, :]

    def centroid(m):
        M = cv2.moments(m, binaryImage=True)
        if M["m00"] <= 0: return None
        return (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"]))

    c1 = centroid(top)
    c2 = centroid(bot)

    if c1 is None or c2 is None:
        return 0.0, vis

    cv2.circle(vis, (c1[0], c1[1]), 4, (255,255,255), -1)
    cv2.circle(vis, (c2[0], c2[1]+midy), 4, (255,255,255), -1)
    p1 = (c1[0], c1[1]); p2 = (c2[0], c2[1]+midy)
    cv2.line(vis, p1, p2, (255,255,255), 2)

    dx = (p2[0] - p1[0])
    dy = (p1[1] - p2[1])
    angle = math.degrees(math.atan2(dx, dy))  # + right, - left
    return angle, vis

def segnet_stub(bgr):
    # If you don't use YOLOP, replace this with your Jetson-Inference segnet call.
    return np.zeros((bgr.shape[0], bgr.shape[1]), dtype=np.uint8)

def main():
    args = parse_args()

    cap = open_input(args.input)
    ok, frame = cap.read()
    if not ok: raise RuntimeError("No frames from input")
    H, W = frame.shape[:2]
    fps_guess = 20.0

    out_mode, writer = open_output(args.output, W, H, fps_guess)

    backend = None
    if args.backend == "yolop":
        if not YOLOP_AVAILABLE:
            raise RuntimeError("YOLOP backend selected but yolop_backend not available")
        backend = YOLOPBackend()

    m = open_mav(args.device)
    print("[mavlink] connected:", args.device)

    smoother = AngleSmoother3()
    send_period = 1.0 / args.send_hz
    last_send = 0.0

    try:
        while True:
            ok, frame = cap.read()
            if not ok: break
            frame = flip_image(frame, args.input_flip)
            vis = frame.copy()

            if backend:
                road_mask = backend.infer_mask(frame)
            else:
                road_mask = segnet_stub(frame)

            raw_angle, vis = compute_angle_deg(road_mask, vis)
            angle_deg = smoother.add(raw_angle)
            cv2.putText(vis, f"angle={{angle_deg:+.1f}} deg", (10, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

            chans = get_rc_channels(m, timeout=0.005)
            sw = chans.get(args.rc, 1500)

            now = time.time()
            if sw >= 1800 and (now - last_send) >= send_period:
                if abs(angle_deg) < args.deadband_deg:
                    steer_pwm = args.neutral
                else:
                    delta = max(-args.pwm_max_delta, min(args.pwm_max_delta, args.kp * angle_deg))
                    steer_pwm = int(args.neutral + delta)

                rc_override(m, {args.steer_ch: steer_pwm})
                last_send = now
                cv2.putText(vis, f"OVERRIDE CH{{args.steer_ch}}: {{steer_pwm}}us",
                            (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
            else:
                release_override(m)
                cv2.putText(vis, "OVERRIDE: OFF", (10, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)

            mask_small = cv2.resize(road_mask, (W//4, H//4))
            vis[0:H//4, W-W//4:W, :] = cv2.cvtColor(mask_small, cv2.COLOR_GRAY2BGR)

            write_frame(out_mode, writer, vis)

    except KeyboardInterrupt:
        pass
    finally:
        release_override(m)
        if writer is not None:
            writer.release()
        cap.release()
        if out_mode == "display":
            cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
```

Make it executable:
```bash
chmod +x ~/segmav_auto/mavsegmav_auto_nudge.py
```

---

## 8) Code — launcher
**File:** `~/segmav_auto/run_segmav.sh`
```bash
#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
source venv/bin/activate

# Change camera if needed: csi://0 for ribbon CSI; /dev/video0 for USB
CAMERA="v4l2:///dev/video0"
OUTPUT="display://0"   # or file:///home/robot/segmav.mp4

python3 mavsegmav_auto_nudge.py   --backend=yolop   --rc=9 --steer-ch=1 --neutral=1500   --kp=4.0 --pwm-max-delta=120 --deadband-deg=2.0   --device=udp:127.0.0.1:14555   "$CAMERA" "$OUTPUT"
```
Make it executable:
```bash
chmod +x ~/segmav_auto/run_segmav.sh
```

---

## 9) Optional — systemd service
**File:** `~/segmav_auto/segmav.service`
```ini
[Unit]
Description=SegMAV AUTO + Nudges (Jetson)
After=network-online.target

[Service]
Type=simple
User=%i
WorkingDirectory=/home/%i/segmav_auto
Environment=DISPLAY=:0
ExecStart=/bin/bash -lc '/home/%i/segmav_auto/run_segmav.sh'
Restart=on-failure

[Install]
WantedBy=default.target
```
Install (replace `YOURUSER`):
```bash
sudo cp ~/segmav_auto/segmav.service /etc/systemd/system/segmav@YOURUSER.service
sudo systemctl daemon-reload
sudo systemctl enable segmav@YOURUSER
sudo systemctl start segmav@YOURUSER
journalctl -u segmav@YOURUSER -f
```

---

## 10) mavlink‑router config
### Recommended Jetson‑centric config
**File:** `/etc/mavlink-router/main.conf`
```ini
[General]
Log=/var/log/mavlink-router
MavlinkDialect=ardupilotmega

# ---- Master (Cube on TELEM2 via FTDI) ----
[UartEndpoint cube]
Device=/dev/ttyUSB0
Baud=57600
FlowControl=auto

# ---- Local UDP for apps on the Jetson (your scripts) ----
[UdpEndpoint jetson_local]
Mode=Client
Address=127.0.0.1
Port=14555

# ---- GCS over Wi‑Fi/LAN (Mission Planner/QGC) ----
[UdpEndpoint gcs_wifi]
Mode=Client
Address=192.168.1.124   # put your laptop IP here
Port=14550

# ---- Optional TCP server for tools (e.g., MP over TCP) ----
[TcpServer tcp_server]
Port=5760
```
Start and enable:
```bash
sudo mkdir -p /etc/mavlink-router
sudo nano /etc/mavlink-router/main.conf   # paste/edit as needed
sudo mavlink-routerd -v -c /etc/mavlink-router/main.conf  # foreground test
sudo systemctl enable mavlink-router
sudo systemctl restart mavlink-router
sudo systemctl status mavlink-router
```

### Your earlier variant (still OK)
```ini
[General]
TcpServerPort = 5760
ReportStats   = true
MavlinkDialect= ardupilotmega
Log           = /var/log/mavlink-router

[UartEndpoint telem1]
Device = /dev/telem1
Baud   = 115200
FlowControl = 0

[UdpEndpoint LocalHub_14553]
Mode    = Server
Address = 127.0.0.1
Port    = 14553

[UdpEndpoint SegMAV_TX_14555]
Mode    = Server
Address = 127.0.0.1
Port    = 14555

[UdpEndpoint SegMAV_RX_14601]
Mode    = Normal
Address = 127.0.0.1
Port    = 14601

[UdpEndpoint GCS_14550]
Mode    = Server
Address = 0.0.0.0
Port    = 14550
```
> Use `--device=udp:127.0.0.1:14555` in the script. Ensure the serial **Device** and **Baud** match your actual Cube link.

---

## 11) How to run (every drive)
1) **Power on**: Rover (Cube + servo/ESC) → Jetson → Transmitter.
2) **Mission Planner**: connect, GPS OK, set **AUTO**, don’t arm yet.
3) **Jetson**:
```bash
cd ~/segmav_auto
source venv/bin/activate
./run_segmav.sh
```
4) You should see the **video** window and mask preview (top‑right).
5) **Arm** the rover and start the mission (WP #1 starts).
6) Flip **CH9 HIGH** → “OVERRIDE CH1: ####us” appears; nudges are active.
7) Flip back to **MID/LOW** to stop nudges. **CH8** is **E‑Stop** if set.

**Tuning quickies (in `run_segmav.sh`)**
- `--kp=4.0`: increase for stronger nudges, decrease if it fights AUTO.
- `--pwm-max-delta=120`: bump size limit (±µs). 80–160 typical.
- `--deadband-deg=2.0`: raise to reduce jitter.

---

## 12) TFmini (Benewake) setup — UART & I²C
**Models differ** (TFmini, TFmini‑S, TFmini‑Plus). **Addresses, baud, and hex commands vary** by model/firmware. Treat the hex examples below as typical patterns — **check your datasheet** to confirm.

### UART mode (most common)
- **Wiring**: TFmini **TX→ Cube RX**, **RX→ Cube TX**, **5 V**, **GND** (on SERIAL4 or whichever port you use).  
- **Parameters** (see §3): `SERIALx_PROTOCOL=9`, `SERIALx_BAUD=115`, `RNGFND1_TYPE=20`.

**Typical serial settings** (factory): **115200 8N1**.  
**Common commands** (example frames; may differ by model):
- **Enter/exit config** (example placeholder): `5A 04 11 6F` (enter), `5A 04 11 6F` (exit).  
- **Set output frame rate** (e.g., 100 Hz): model‑specific (commonly a `5A 06 ..` pattern).  
- **Save config**: model‑specific (often another `5A 04 ..` pattern).  

> **ArduPilot usually needs no hex** — it just reads distances once UART is set. Only use hex if you must switch modes/rates in the sensor.

### I²C mode (if supported on your unit)
Some TFmini variants can switch to I²C with a vendor command; others are I²C‑only models.

- **Wiring**: **SCL/SDA + 3.3/5 V + GND** to the Cube’s I²C.
- **Default address** (many Benewake units): **0x10 (16 decimal)** — check yours.
- **Parameters**: `RNGFND1_TYPE=25`, `RNGFND1_ADDR=16` (decimal for 0x10).

**Typical I²C switching**:
- **UART→I²C**: send vendor command over UART, power‑cycle, then use I²C pins.
- **I²C→UART**: write vendor command via I²C, power‑cycle to return to UART.
- **Exact hex varies** — provide your model and I’ll insert the verified frames.

**Validation:** In MP → “Status”, watch `rangefinder_distance` while moving a target.

---

## 13) Validation checklist
- **Mission Planner → Status**: `RCIN9` ≈ 2000 when SWC is HIGH; `SERVO1` moves near 1500 ± nudges.
- **Video overlay**: shows `angle=±X deg` and **OVERRIDE** line when nudges are ON.
- **MAVLink**: `nc -u -l 14555` on Jetson shows traffic when router runs.
- **GPS/EKF**: No “AHRS/GPS unhealthy” messages before arming.
- **Power**: Servo rail has a solid 6 V source (UBEC), shared ground to Cube & Jetson.

---

## 14) Troubleshooting
- **No motion in AUTO** → not armed, EKF unhappy, or mission not started. Fix GPS/compass; wait for 3D fix; arm again.
- **No nudges** → CH9 not HIGH; or script not connected to MAVLink. Check `--device` and router.
- **Fights AUTO** → lower `--kp` and/or `--pwm-max-delta` (e.g., 2.5 and 80).
- **Jittery steering** → raise `--deadband-deg` (try 3–4).
- **Servo brownouts** → stronger UBEC, ensure grounds are shared.
- **Camera blank** → correct path (`/dev/video0` vs `csi://0`), add `--input-flip=rotate-180` if upside‑down, ensure an active desktop for `display://0`.
- **Serial device keeps changing** (`/dev/ttyUSB0` → `/dev/ttyUSB1`) → make a **udev rule** for a stable symlink (e.g., `/dev/cube`) and use that in `main.conf`.

---

## 15) Quick tailoring knobs
If you want me to hard‑code these, say the word:
- **RC switch channel**: `--rc` (default 9)
- **Steering channel**: `--steer-ch` (default 1)
- **Neutral PWM**: `--neutral` (default 1500)
- **MAVLink device**: `--device=udp:127.0.0.1:14555` (or `/dev/ttyUSB0,57600`)
- **Camera source**: `v4l2:///dev/video0` or `csi://0`
- **Router ports**: `14555` (Jetson apps), `14550` (GCS)

---

## 16) Appendix — udev example for stable Cube port
Find IDs:
```bash
udevadm info -a -n /dev/ttyUSB0 | egrep -i "idVendor|idProduct" -m2
```
Create rule (edit IDs to match):
```bash
echo 'SUBSYSTEM=="tty", ATTRS{{idVendor}}=="1a86", ATTRS{{idProduct}}=="7523", SYMLINK+="cube"' | sudo tee /etc/udev/rules.d/99-cube.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```
Then set in `/etc/mavlink-router/main.conf`:
```
Device=/dev/cube
```

---

**You’re set.** Plan → AUTO → ARM → run `run_segmav.sh` → flip **CH9 HIGH** → enjoy sidewalk‑steady waypoints.
