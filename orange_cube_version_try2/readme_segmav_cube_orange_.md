
# SegMAV + Cube Orange (Rover) — Fresh Start Guide
**End‑to‑end setup to run Steven Dade’s SegMAV-style vision with an AUTO mission, steering “nudges,” forward LiDAR (BendyRuler), emergency stop, and clean wiring for the Cube Orange + Jetson Orin Nano.**

> This is a **consolidated guide** distilled from our whole conversation. It’s written to be **copy/paste friendly** and field‑ready.

---

## 0) What you’re building (mental model)

- **Autopilot (ArduPilot Rover)** runs your **AUTO mission** (waypoints, speeds, OA, failsafes).
- **Jetson (SegMAV)** looks at the camera, finds the sidewalk/drivable region, and **gently biases steering** so you stay on the path.
- **3‑position RC switch (CH9)** controls SegMAV behavior (OFF / record / nudge).
- **Emergency stop (CH8)** hard‑kills the motors if needed.
- **Forward LiDAR** feeds **BendyRuler** obstacle avoidance on the Cube.
- The on‑screen overlay looks like Steven Dade’s: **purple mask → red polygon → two white centroids → white trajectory line**; angle is smoothed and used for nudging.

---

## 1) Upstream reference (SegMAV by Steven Dade)

- Repo: `stephendade/segmav` (semantic segmentation; finds largest “road” contour; splits; centroids; angle; 3‑point moving avg; publishes MAVLink **SET_POSITION_TARGET_LOCAL_NED** with body‑frame **forward velocity + yaw**).  
- Typical run (Steven’s original style, sending vel+yaw in GUIDED):
  ```bash
  python3 mavsegmav.py     --rc=9 --pwmlow=1000 --pwmmid=1500 --pwmhigh=2000     --vel=2.0     --device=udpout:127.0.0.1:14555     v4l2:///dev/video0     rtp://<GCS-IP>:5400
  ```

> In this guide we additionally support **AUTO + RC steering override “nudges”** so ArduPilot remains in charge of the mission while vision gently biases steering.

---

## 2) Hardware & Ports (Cube Orange + ADS‑B carrier v6.4)

### Devices you’re using
- **Jetson Orin Nano** (camera + SegMAV process)
- **Cube Orange** (ArduPilot Rover)
- **FS‑i6X + FS‑iA6B** (Mode‑2 radio, iBUS to RC IN)
- **Mateksys M10Q‑5883** (GNSS+compass) → **GPS1** (UART + I²C on same 10‑pin)
- **SiK telemetry radio** → **TELEM1** (MAVLink to GCS)
- **Jetson ↔ Cube** via **FTDI USB‑TTL** → **TELEM2** (MAVLink)
- **Benewake TFmini / TFmini‑Plus** (forward LiDAR) → **GPS2** (UART or I²C)
- **INJS035 steering servo** (30+ kg·cm) → **MAIN OUT 1** (powered by **UBEC**)

### Port map (summary)

| Device                 | Port on Cube (ADS‑B board) | Bus / Notes                                       |
|------------------------|-----------------------------|---------------------------------------------------|
| Matek M10Q‑5883 GNSS   | **GPS1 (10‑pin)**           | UART (GPS) + **I²C (compass)**                    |
| SiK Telemetry radio    | **TELEM1 (6‑pin)**          | MAVLink (GCS)                                     |
| Jetson FTDI (USB‑TTL)  | **TELEM2 (6‑pin)**          | MAVLink (companion)                               |
| Benewake TFmini/+      | **GPS2 (6‑pin)**            | UART **or** I²C (selectable; see §6)              |
| Steering servo         | **MAIN OUT 1**              | PWM signal from Cube; **power from UBEC**         |
| Motor ESC              | **MAIN OUT 3**              | PWM (default Rover throttle output)               |

> **Avoid TELEM3** on the ADS‑B carrier—often reserved by the onboard ADS‑B module.  
> **6‑pin “USB” header** = breakout of the Cube’s micro‑USB lines (not a UART/host). Don’t use it for Jetson MAVLink.

---

## 3) Radio channels & switches (FS‑i6X, Mode‑2)

Recommended mapping:

| RC channel | Purpose                                          |
|------------|---------------------------------------------------|
| **CH1**    | Steering (right stick **X**)                      |
| **CH2**    | Throttle (right stick **Y**, self‑centering)      |
| **CH5**    | Flight mode (optional)                            |
| **CH8**    | **Motor Emergency Stop** (`RC8_OPTION = 31`)      |
| **CH9**    | **SegMAV 3‑pos**: LOW=stop, MID=record, HIGH=nudge |

- **Set Mode‑2**: *Long‑press OK → Functions Setup → Stick Mode → Mode 2*  
- **Aux Channels**: map **CH9 → SWC (3‑pos)**, **CH8 → SWB (2‑pos)**  
- **Receiver**: **iBUS** from FS‑iA6B → **RC IN** on Cube

---

## 4) ArduPilot parameters (Rover)

In **Mission Planner → Config/Tuning → Full Parameter Tree**:

```text
# Outputs
SERVO1_FUNCTION = 26        # Steering on MAIN OUT 1
SERVO3_FUNCTION = 70        # Throttle on MAIN OUT 3 (default Rover)

# Emergency stop (recommended, on CH8)
RC8_OPTION      = 31        # Motor E-Stop

# Serial ports
SERIAL1_PROTOCOL = 2        # TELEM1 → MAVLink (SiK)
SERIAL1_BAUD     = 57       # 57600 (typical for SiK)
SERIAL2_PROTOCOL = 2        # TELEM2 → MAVLink (Jetson)
SERIAL2_BAUD     = 57

# GPS/Compass (Matek on GPS1)
SERIAL3_PROTOCOL = 5        # GPS
GPS_TYPE         = 1        # Ublox

# LiDAR (if UART on GPS2 → SERIAL4)
# SERIAL4_PROTOCOL = 9      # Lidar
# SERIAL4_BAUD     = 115    # 115200 (TFmini default)

# LiDAR (rangefinder) common
# UART Benewake:
# RNGFND1_TYPE   = 20
# I²C Benewake:
# RNGFND1_TYPE   = 25
# RNGFND1_ADDR   = 16       # 0x10 default for TFmini I²C
RNGFND1_MIN_CM = 30
RNGFND1_MAX_CM = 600
RNGFND1_ORIENT = 0          # forward

# Obstacle Avoidance (BendyRuler)
OA_TYPE = 1                 # BendyRuler
# OA_BR_LOOKAHEAD ~ 5, OA_MARGIN_MAX ~ 2 (start points; tune)
```

**Right‑stick self‑centering throttle (optional):**  
If you want ArduPilot to use **CH2** for throttle, set `SERVO2_FUNCTION = 70` **and** plug ESC into **MAIN OUT 2**.  
Most users keep the default **ESC on OUT3** (`SERVO3_FUNCTION = 70`) and simply calibrate RC so **CH3** is throttle; either works.

**Calibrate RC:** *Setup → Mandatory Hardware → Radio Calibration* (center ≈ **1500**, max ≈ **2000**, min ≈ **1000**).

---

## 5) Wiring details & power

### Steering servo (INJS035, 29–35 kg·cm)
- Needs **4.8–6.0 V**, **3–4 A** peak → **use a UBEC** (5–10 A) at **6 V**.  
- Wire UBEC **+6 V** and **GND** to the **MAIN OUT power rail**; servo **signal** to **MAIN OUT 1 (S)**.  
- Do **not** power from the Cube’s internal 5 V alone.

### LiDAR (TFmini / TFmini‑Plus)
- **UART mode**: TFmini **TX → RX**, **RX → TX**, **5V**, **GND** on **GPS2** (or any spare UART).  
  Set `SERIAL4_PROTOCOL=9`, `SERIAL4_BAUD=115`, `RNGFND1_TYPE=20`.
- **I²C mode**: SDA/SCL/5V/GND on **GPS1/2 I²C pins**.  
  Set `RNGFND1_TYPE=25`, `RNGFND1_ADDR=16`. See §6 to switch the sensor to I²C.

### Jetson ↔ Cube (MAVLink)
- **FTDI USB‑TTL** to **TELEM2**: **FTDI TX → Cube RX**, **FTDI RX → Cube TX**, **GND ↔ GND**.  
  On Jetson this appears as **`/dev/ttyUSB0`**.

### Telemetry radio (SiK)
- **TELEM1** with standard 6‑pin JST‑GH cable. Set `SERIAL1_PROTOCOL=2`, `SERIAL1_BAUD=57`.

### GPS + Compass (Matek M10Q‑5883)
- **GPS1 (10‑pin)** carries **UART (GPS)** + **I²C (compass)**; plug straight in.

---

## 6) Benewake TFmini/TFmini‑Plus — switch to I²C (optional)

1. Connect TFmini to a **USB‑TTL** (TX↔RX, GND, 5V).  
2. Use Benewake tool **or** raw commands:  
   - **I²C mode:** `5A 05 0A 01 6A`  
   - **Save:** `5A 04 11 6F`  
   - Power‑cycle the sensor.  
3. Wire **SDA/SCL/5V/GND** to Cube **GPS2 (or GPS1) I²C**.  
4. Params:
   ```text
   RNGFND1_TYPE   = 25   # Benewake I²C
   RNGFND1_ADDR   = 16   # 0x10 default
   RNGFND1_MIN_CM = 30
   RNGFND1_MAX_CM = 600
   RNGFND1_ORIENT = 0
   ```

---

## 7) AUTO + Vision “Nudges” (script)

This script keeps Steven’s **trajectory overlay** but, instead of taking over in GUIDED, it **biases steering** via **RC_CHANNELS_OVERRIDE** while the rover runs **AUTO**.  
- **CH9 HIGH** → apply nudges  
- **CH9 MID/LOW** → release (no override)  
- Output video goes to **display** or **file** on the Jetson.

> If you prefer **GUIDED with velocity+yaw** like upstream SegMAV, keep using Steven’s `mavsegmav.py` as‑is. You can run either mode depending on your test plan.

### Install (Jetson)
```bash
sudo apt-get update
sudo apt-get install -y python3-venv python3-opencv python3-pip

mkdir -p ~/segmav_auto && cd ~/segmav_auto
python3 -m venv venv && source venv/bin/activate
pip install --upgrade pip wheel numpy
pip install pymavlink opencv-python

# (Optional) YOLOP backend (recommended):
pip install torch torchvision --extra-index-url https://download.pytorch.org/whl/cu118
```

### `yolop_backend.py` (optional; improved segmentation)
```python
import cv2, torch, numpy as np

class YOLOPBackend:
    """YOLOP drivable-area backend. Returns uint8 mask (255=navigable)."""
    def __init__(self, device=None, half=True):
        self.device = torch.device(device or ("cuda" if torch.cuda.is_available() else "cpu"))
        self.model = torch.hub.load("hustvl/YOLOP", "yolop", pretrained=True).to(self.device).eval()
        self.half = half and self.device.type == "cuda"
        if self.half:
            self.model.half()
        self.inp_w, self.inp_h = 640, 384

    @torch.inference_mode()
    def infer_mask(self, bgr_img: np.ndarray) -> np.ndarray:
        h0, w0 = bgr_img.shape[:2]
        img = cv2.resize(bgr_img, (self.inp_w, self.inp_h))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        t = torch.from_numpy(img).permute(2,0,1).unsqueeze(0).to(self.device)
        if self.half: t = t.half()
        _, da_seg_out, _ = self.model(t)  # (det, drivable, lane)
        da = torch.sigmoid(da_seg_out).squeeze().float().cpu().numpy()
        mask = (da > 0.5).astype(np.uint8) * 255
        return cv2.resize(mask, (w0, h0), interpolation=cv2.INTER_NEAREST)
```

### `mavsegmav_auto_nudge.py`
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
    p.add_argument("--rc", type=int, default=9, help="RC channel for 3-pos switch")
    p.add_argument("--steer-ch", type=int, default=1, help="Steering PWM channel (CH1)")
    p.add_argument("--neutral", type=int, default=1500, help="Steering neutral PWM")
    p.add_argument("--pwm-max-delta", type=int, default=120, help="Max steering delta (µs)")
    p.add_argument("--kp", type=float, default=4.0, help="µs per degree of angle error")
    p.add_argument("--deadband-deg", type=float, default=2.0, help="ignore small errors")
    p.add_argument("--send-hz", type=float, default=15.0, help="RC override rate")
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
        base, ext = os.path.splitext(path); 
        path2 = base + ts + (ext if ext else ".mp4")
        vw = cv2.VideoWriter(path2, fourcc, fps, (w,h))
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
    chs = [0]*8
    for ch, pwm in ch_to_pwm.items():
        if 1 <= ch <= 8 and pwm is not None:
            chs[ch-1] = int(pwm)
    m.mav.rc_channels_override_send(m.target_system, m.target_component, *chs)

def release_override(m):
    rc_override(m, {})

def flip_image(img, mode: str):
    if mode == "rotate-180":   return cv2.rotate(img, cv2.ROTATE_180)
    if mode == "flip-horizontal": return cv2.flip(img, 1)
    if mode == "flip-vertical": return cv2.flip(img, 0)
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
    h, w = mask.shape[:2]
    cnt = largest_contour(mask)
    if cnt is None or cv2.contourArea(cnt) < 500:
        return 0.0, vis
    cv2.drawContours(vis, [cnt], -1, (0,0,255), 2)     # red polygon
    midy = h//2
    top, bot = mask[:midy,:], mask[midy:,:]
    def centroid(m): 
        M = cv2.moments(m, binaryImage=True)
        if M["m00"] <= 0: return None
        return (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"]))
    c1, c2 = centroid(top), centroid(bot)
    if c1 is None or c2 is None: return 0.0, vis
    cv2.circle(vis, (c1[0], c1[1]), 4, (255,255,255), -1)
    cv2.circle(vis, (c2[0], c2[1]+midy), 4, (255,255,255), -1)
    p1, p2 = (c1[0], c1[1]), (c2[0], c2[1]+midy)
    cv2.line(vis, p1, p2, (255,255,255), 2)
    dx = (p2[0] - p1[0]); dy = (p1[1] - p2[1])
    angle = math.degrees(math.atan2(dx, dy))           # +right, -left
    return angle, vis

def segnet_stub(bgr):
    return np.zeros((bgr.shape[0], bgr.shape[1]), dtype=np.uint8)

def main():
    args = parse_args()
    cap = open_input(args.input)
    ok, frame = cap.read()
    if not ok: raise RuntimeError("No frames from input")
    H, W = frame.shape[:2]; fps_guess = 20.0
    out_mode, writer = open_output(args.output, W, H, fps_guess)
    backend = None
    if args.backend == "yolop":
        if not YOLOP_AVAILABLE: raise RuntimeError("YOLOP backend selected but yolop_backend not available")
        backend = YOLOPBackend()
    m = open_mav(args.device)
    print("[mavlink] connected:", args.device)
    smoother = AngleSmoother3()
    send_period = 1.0 / args.send_hz; last_send = 0.0
    try:
        while True:
            ok, frame = cap.read()
            if not ok: break
            frame = flip_image(frame, args.input_flip)
            vis = frame.copy()
            road_mask = backend.infer_mask(frame) if backend else segnet_stub(frame)
            raw_angle, vis = compute_angle_deg(road_mask, vis)
            angle_deg = smoother.add(raw_angle)
            cv2.putText(vis, f"angle={angle_deg:+.1f} deg", (10, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
            from_rc = get_rc_channels(m, timeout=0.005)
            sw = from_rc.get(args.rc, 1500)
            now = time.time()
            if sw >= 1800 and (now - last_send) >= send_period:
                if abs(angle_deg) < args.deadband_deg: steer_pwm = args.neutral
                else:
                    delta = max(-args.pwm_max_delta, min(args.pwm_max_delta, args.kp * angle_deg))
                    steer_pwm = int(args.neutral + delta)
                rc_override(m, {args.steer_ch: steer_pwm})
                last_send = now
                cv2.putText(vis, f"OVERRIDE CH{args.steer_ch}: {steer_pwm}us",
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
        if writer is not None: writer.release()
        cap.release()
        if out_mode == "display": cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
```

### Run (on Jetson HDMI display)
```bash
cd ~/segmav_auto
source venv/bin/activate

python3 mavsegmav_auto_nudge.py   --backend=yolop   --rc=9 --steer-ch=1 --neutral=1500   --kp=4.0 --pwm-max-delta=120 --deadband-deg=2.0   --device=udp:127.0.0.1:14555   v4l2:///dev/video0 display://0
```
- **AUTO**: upload mission → set **AUTO** → **ARM** → start mission.
- Flip **CH9 = HIGH** once; the Jetson biases steering the whole mission.
- **MID/LOW** releases override at once; mission continues.
- **CH8** is your **E‑Stop** (if set).

**If you want to view on the GCS instead**: use `rtp://<GCS-IP>:5400` as the output in Steven’s original script or add RTP to yours.

---

## 8) Validating GUIDED nudges (if you stick with upstream SegMAV behavior)

If you keep **GUIDED + SET_POSITION_TARGET_LOCAL_NED** (Steven’s stock approach):
- Must be **GUIDED**, **ARMED**, **EKF healthy**; otherwise Rover ignores setpoints.
- In Mission Planner → **MAVLink Inspector**: watch **SET_POSITION_TARGET_LOCAL_NED** arriving at 5–20 Hz.
- Ensure **type_mask** enables **velocity** + **yaw** (not position). Avoid conflicting yaw & yaw_rate bits.

> Advanced: `GUID_OPTIONS` bit 6 (=64) makes Guided use waypoint‑style navigation for position targets; usually not needed for pure velocity targets.

---

## 9) Quick troubleshooting

- **AHRS/GPS bad** → fix first (cabling, compass orientation, mag interference, GPS HDOP). Guided/AUTO may reject commands when EKF is unhappy.
- **No steering movement** (AUTO + nudges) → confirm `RCIN9 ~ 2000` for HIGH; reduce `--kp` / `--pwm-max-delta` if fighting; check Mission Planner “SERVO Output” live.
- **LiDAR no data** → verify `RNGFND1_TYPE`, port protocol/baud or I²C address; check `rngfnd1_distance` in Status tab.
- **Servo brownouts** → use stronger UBEC; ensure shared **GND**.

---

## 10) Field checklist

1. **Wiring**: GPS1 (Matek), TELEM1 (SiK), TELEM2 (Jetson), GPS2 (TFmini), UBEC to servo rail, servo MAIN OUT 1, ESC MAIN OUT 3.  
2. **Params**: outputs, serials, RNGFND, OA, E‑Stop.  
3. **Radio**: Mode‑2; **CH9 → SWC**, **CH8 → SWB**; calibrate RC.  
4. **Mission**: Plan → Write; set **AUTO**; **ARM**.  
5. **Run script**; flip **CH9 HIGH** for vision nudges.  
6. Tune `--kp`, `--pwm-max-delta`, `--deadband-deg` for smooth lane‑keeping.  

---

### Notes & options
- **SegNet vs YOLOP**: Keep Steven’s SegNet or switch to YOLOP backend for stronger masks. Either way, downstream overlay + nudge logic is identical.  
- **Depth cameras**: Not required; SegMAV uses RGB. You can fuse LiDAR data via ArduPilot BendyRuler.  
- **RTP vs local display**: Use `display://0` on Jetson for on‑board viewing; `rtp://<GCS-IP>:5400` to watch at the GCS.  

---

**You’re set.** This README contains the wiring, parameters, LiDAR setup, emergency stop, and the full **AUTO + vision nudge** code with run commands. Drop it into your project as `readme_segmav_cube_orange_.md` and iterate.
