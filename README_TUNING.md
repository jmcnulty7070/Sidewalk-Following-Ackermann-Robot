# 🚗 Ackermann Rover Tuning Guide

This guide explains how to tune your ArduPilot parameters so your Ackermann-style rover (car-like steering) makes **smooth turns** around waypoints instead of jerky pivots.

---

## 1. Key Parameter: `WP_RADIUS`

- **What it does**: Defines the radius (in meters) around a waypoint where the rover considers the waypoint "reached" and begins turning toward the next one.  
- **Default**: ~2.0 m for rovers.  
- **Effect**:  
  - Too small → rover tries to “hit” the waypoint exactly → sharp pivot.  
  - Larger value → rover arcs smoothly through the waypoint.  

👉 **Set `WP_RADIUS = 2.0–4.0 m`** depending on your driveway/sidewalk width.

---

## 2. Supporting Parameters for Ackermann Cars

### `NAVL1_PERIOD`
- Controls **lookahead distance** for the L1 navigation controller.  
- **Bigger values** = smoother, wider turns.  
- **Typical range**: 15–25.  
- Start at `20` if turns are too sharp.

### `TURN_MAX_G`  
(or sometimes `ATC_STR_RAT_MAX` depending on firmware)  
- Limits maximum steering rate.  
- Helps prevent jerky steering inputs on tight turns.  
- Example: lower this gradually until steering feels natural.

### `WP_OVERSHOOT`
- Allows the rover to **pass slightly beyond** the waypoint before turning.  
- Smooths pathing and prevents early cut-ins.  
- Start with **0.5–1.0 m**.

---

## 3. Recommended Starting Values

| Parameter       | Value        | Effect                                |
|-----------------|--------------|---------------------------------------|
| `WP_RADIUS`     | 2.0–3.0 m    | Larger radius for smoother turns      |
| `NAVL1_PERIOD`  | 20           | Increases lookahead distance          |
| `TURN_MAX_G`    | Adjust down  | Prevents aggressive steering          |
| `WP_OVERSHOOT`  | 0.5–1.0 m    | Lets car flow past waypoint naturally |

---

## 4. How to Apply

1. Connect to your rover in **Mission Planner**.  
2. Go to **Config → Full Parameter List**.  
3. Update the values above.  
4. Write parameters and reboot the flight controller.  
5. Run a test mission down your driveway:
   - Watch if it pivots or arcs too sharply.  
   - Increase `WP_RADIUS` or `NAVL1_PERIOD` until it flows smoothly.  

---

## 5. Quick Tips

- 🚦 For **tight driveways**, use `WP_RADIUS` = 2 m.  
- 🛣️ For **sidewalk navigation**, you may need 3–4 m.  
- 🔄 Always test in **Guided mode** first before running a full AUTO mission.  

✅ **In short**:  
- Start with `WP_RADIUS = 2.5 m`  
- Bump `NAVL1_PERIOD` to ~20  
- Add `WP_OVERSHOOT` if turns still feel jerky  

---
