
# 📡 Radiolink GPS + 800MHz Telemetry Setup for Jetson Nano + CrossFlight Robot

This guide explains how to wire and configure the **Radiolink M10N TS100 GPS module** and an **800 MHz telemetry radio** for your robot using the **Radiolink CrossFlight** flight controller and Jetson Nano running MAVROS.

---

## ✅ Recommended GPS Module

**Module:** Radiolink M10N TS100  
**Features:**
- GNSS: GPS + GLONASS + Galileo + BDS
- Accuracy: <1.5 m
- Compass: Built-in (QMC5883L)
- Refresh Rate: Up to 10 Hz
- Power: 5V from CrossFlight GPS/I2C Port
- Connector: JST-GH (plug-and-play)

📦 Example:  
[Radiolink M10N TS100 GPS on Amazon](https://www.amazon.com/dp/B0DXV1KGH6)

---

## 🛠️ Wiring GPS Module

| GPS Wire | Connects To CrossFlight GPS/I2C Port |
|----------|--------------------------------------|
| TX       | RX (Serial3)                         |
| RX       | TX                                   |
| SDA/SCL  | I2C compass connection               |
| GND      | GND                                  |
| 5V       | 5V                                   |

> 💡 **Note:** GPS and compass data are sent through the same 4-pin JST-GH plug. No additional compass wiring is needed.

---

## 🔧 ArduPilot Parameters (via Mission Planner)

1. Plug in GPS to CrossFlight GPS/I2C port.
2. In **Mission Planner > Config > Full Parameter List**, set:

```
SERIAL3_PROTOCOL = 5      # GPS Protocol
SERIAL3_BAUD     = 38400  # Default for M10N
COMPASS_USE      = 1      # Enable Compass
COMPASS_EXTERNAL = 1      # Use external GPS compass
```

3. Save and reboot the flight controller.
4. Perform **Compass + Accelerometer calibration** in Mission Planner.

---

## 📶 Telemetry Radio (800 MHz)

**Use Case:** Connect Jetson Nano to Mission Planner on PC

**Example Radios:** Radiolink TSP 800, SiK Radio, RFD900x

### 🛠️ Wiring Telemetry to CrossFlight

| Radio Pin     | Connects To TELEM2 on CrossFlight |
|---------------|------------------------------------|
| TX (Radio)    | RX (TELEM2)                        |
| RX (Radio)    | TX (TELEM2)                        |
| GND           | GND                                |
| 5V            | 5V (from CrossFlight or BEC)       |

---

## 🔧 TELEM2 Parameters for Radio

In **Mission Planner**, set:

```
SERIAL2_PROTOCOL = 2       # MAVLink
SERIAL2_BAUD     = 57600   # Match with radio
```

📍 Use **Mission Planner on your PC** to connect over COM port at 57600 baud.

---

## 📈 Typical Port Assignments

| Port      | Function                        |
|-----------|----------------------------------|
| TELEM1    | Jetson Nano (MAVROS ↔ MAVLink) |
| TELEM2    | 800 MHz telemetry radio to PC  |
| GPS/I2C   | M10N GPS + Compass             |

---

## 📦 Mission Planner Connection

- Select the COM port from your radio (shows up via USB)
- Set **baud rate = 57600**
- Click **Connect**
- Monitor GPS, compass, MAVLink messages, and camera feedback

---

## 🧠 Notes

- GPS and telemetry radio both use **UART**, so avoid port conflicts
- The **M10N TS100** GPS works out-of-the-box with CrossFlight
- Jetson ↔ Pixhawk MAVLink bridge uses `/dev/ttyTHS1` (via TELEM1)
- Compass orientation: Arrow forward, flat mounting

---

## 🏁 Summary

| Feature              | Recommendation                 |
|----------------------|---------------------------------|
| GPS Module           | Radiolink M10N TS100            |
| GPS Port             | GPS/I2C (Serial3)               |
| Telemetry Radio Port | TELEM2                          |
| MAVROS Port          | TELEM1                          |
| Baud Rate            | 57600 (GPS, Radio, Jetson)      |
| Compass              | Use built-in QMC5883L           |
| Calibration          | Required (Compass + Accelerometer) |

---
1. The Mission Planner for parameters setup that CrossFlight supports:

The CrossFlight can set parameters by Radiolink Mission Planner, ArduPilot Mission Planner, and QGC Mission Planner.

2. The Mission Planner for firmware update that CrossFlight support:

① Connect the CrossFlight to the Radiolink Mission Planner, there is a (d5a4c778) default that can upgrade the firmware by both the Radiolink Mission Planner and ArduPilot Mission Planner.
---
link https://www.radiolink.com.cn/crossflight_missionplanner
---