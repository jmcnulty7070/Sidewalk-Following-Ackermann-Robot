# 🚗 ArduPilot Rover QuickTune for Ackermann Steering

This guide shows how to use the **QuickTune Lua script** with an **Ackermann-style rover** (front steering vehicle), such as one using a **Radiolink CrossFlight** flight controller.

---

## 🎥 Video Tutorial

Watch this demo of QuickTune in action:
👉 https://www.youtube.com/watch?v=yGB9uLD4dkM&t=183s

---

## ✅ Why It Works for Ackermann

- ArduPilot Rover supports Ackermann steering (set `FRAME_TYPE = 2`)
- QuickTune auto-tunes:
  - Steering controller
  - Speed/throttle controller
- Works even with Ackermann-style vehicles (no skid or differential turning required)

---

## ⚙️ Required Parameter Settings

| Parameter        | Value        | Description                                      |
|------------------|--------------|--------------------------------------------------|
| `FRAME_CLASS`    | `1`          | Rover                                             |
| `FRAME_TYPE`     | `2`          | Ackermann steering                               |
| `ATC_STR_TYPE`   | `1`          | Angular position steering                        |
| `SCR_ENABLE`     | `1`          | Enable Lua scripting                             |
| `QUIK_ENABLE`    | `1`          | Enable QuickTune script                          |
| `RC9_OPTION`     | `300`        | RC switch to trigger QuickTune                   |
| `CIRCLE_RADIUS`  | `5` (or more)| Circle size used during tuning                   |

---

## 🔧 Installation Steps

1. Flash **Rover firmware** to your CrossFlight via Mission Planner.
2. Enable scripting:
   - `SCR_ENABLE = 1`
3. Copy the `rover-quicktune.lua` script to your SD card:
   ```
   /APM/scripts/rover-quicktune.lua
   ```
4. Reboot your autopilot.
5. In Mission Planner:
   - Set `RC9_OPTION = 300` (or any unused RC channel)
6. Ensure vehicle can drive in a smooth circle in **CIRCLE mode**.

---

## ▶️ Running QuickTune

1. Set flight mode to `CIRCLE`
2. Drive in a smooth circle manually.
3. Flip the RC switch:
   - **Middle** = Do nothing
   - **High** = Start tuning
   - **Low** = Cancel tuning
4. After a few rotations, the script:
   - Tunes **steering controller**
   - Tunes **speed controller**
5. Flip the switch to HIGH again to **save the gains**

---

## 📌 Tips

- Make sure your **steering and throttle respond smoothly** before tuning.
- Keep the **circle radius reasonably wide** for Ackermann.
- You can later fine-tune gains:
  - `ATC_STR_RAT_P`, `ATC_STR_RAT_I`, etc.
- You can plot results in Mission Planner logs or view them live in the **tuning window**.

---

## 📁 Optional: Include in GitHub Project

- Add this guide as `README_QuikTune.md`
- Include the Lua script in your repo `/scripts`

---

## 💬 Questions?

Visit [https://ardupilot.org/rover/](https://ardupilot.org/rover/) for full Rover documentation.

