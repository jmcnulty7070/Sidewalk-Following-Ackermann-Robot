# 🔌 Jetson Nano to Pixhawk Crossflight Wiring Guide

This README provides safe and tested wiring instructions for connecting a **Jetson Nano** to a **Pixhawk Crossflight** using a logic level converter.

---

## 📍 Port Usage on Pixhawk Crossflight

| Purpose        | Pixhawk Port |
|----------------|--------------|
| MAVROS/MAVLink | TELEM1       |
| GPS Module     | TELEM2       |

### Configure in Mission Planner

| Parameter          | Value   |
|--------------------|---------|
| `SERIAL1_PROTOCOL` | `2`     |
| `SERIAL1_BAUD`     | `57600` |

---

## ✅ Summary: Best Practice Wiring

| Feature           | Recommendation                               |
|-------------------|-----------------------------------------------|
| Signal Level      | 3.3V ↔ 5V via logic level converter            |
| Port on Jetson    | UART1 (`/dev/ttyTHS1`, GPIO TX=8, RX=10)       |
| Port on Pixhawk   | TELEM1                                        |
| Safest Connection | Via logic level converter                     |
| Baud Rate         | 57600 (standard for MAVLink)                  |

---

## ✅ Logic Level Converter Wiring Summary

| Jetson Nano Pin | Logic Converter (Low Side / 3.3V) | Logic Converter (High Side / 5V) | Pixhawk TELEM1 Pin     |
|------------------|-----------------------------------|------------------------------------|-------------------------|
| TX (Pin 8)        | LV-TX (output from Jetson)        | HV-RX (input to Pixhawk)           | RX                      |
| RX (Pin 10)       | LV-RX (input to Jetson)           | HV-TX (output from Pixhawk)        | TX                      |
| GND (Pin 6)       | GND                               | GND                                | GND                     |
| 3.3V (Pin 1)      | LV (reference voltage)            | —                                  | —                       |
| 5V (from Pixhawk) | —                                 | HV (reference voltage)             | 5V                      |

---

## 🔌 Explanation

- **3.3V (Pin 1 from Jetson Nano)** connects to the LV (Low Voltage) reference pin on the logic level converter.  
  This tells the converter that the Jetson side is using 3.3V.

- **5V (from Pixhawk TELEM port)** connects to the HV (High Voltage) reference on the converter.  
  This tells the converter that the Pixhawk side is using 5V.

- **TX/RX are crossed (TX → RX, RX ← TX)** through the converter:
  - Jetson's TX (Pin 8) → LV-TX → HV-RX → Pixhawk RX
  - Jetson's RX (Pin 10) ← LV-RX ← HV-TX ← Pixhawk TX

- **GND must be shared across all devices** to complete the circuit.

---

This setup ensures reliable and safe serial communication between your Jetson Nano and Pixhawk flight controller.