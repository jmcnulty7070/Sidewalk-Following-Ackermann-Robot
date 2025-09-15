
# SegMAV Communication Flow (Text-Only Diagram)
**Date:** 2025-09-15

This README documents how each component talks to the others during a **GUIDED mission** with **SegMAV nudges** and **LiDAR OBSTACLE_DISTANCE** avoidance.

---

## High-Level Topology

```text
                   (Your PC)
        ┌───────────────────────────┐
        │ Mission Planner (GCS)     │
        │ - Builds & WRITES mission │
        │ - Can ARM/DISARM          │
        └─────────────┬─────────────┘
                      │ SiK Radio (MAVLink)
                  bi  │
                      ▼
        ┌───────────────────────────┐
        │      Flight Controller    │
        │ (ArduPilot Rover, sys=1)  │
        │ - Executes GUIDED motion  │
        │ - BendyRuler OA w/        │
        │   OBSTACLE_DISTANCE       │
        └─────────────┬─────────────┘
                      │ UART /dev/telem1 @115200 (MAVLink2)
                  bi  │
                      ▼
        ┌───────────────────────────┐
        │       Jetson (Router)     │
        │   mavlink-routerd hubs:   │
        │   • TCP :5760  (Runner)   │
        │   • UDP :14553 (LocalHub) │
        │   • UDP :14555 (SegMAV TX)│
        │   • UDP →14601 (SegMAV RX)│
        └─┬───────────┬───────────┬─┘
          │           │           │
          │           │           │
          │           │           │
          │           │           │
   TCP bi │           │ UDP out   │ UDP in
          ▼           ▼           ▲
┌────────────────┐  ┌──────────────────────┐     ┌──────────────────────┐
│ GUIDED Runner  │  │ LiDAR→MAVLink Bridge │     │      SegMAV          │
│ (pymavlink)    │  │ scan_to_obstacle_... │     │ mavsegmav_merged.py  │
│ --hub tcp:5760 │  │ --hub udpout:14553   │     │ TX→:14555 / RX←14601 │
│ Sends:         │  │ Sends:               │     │ Sends:               │
│  • set GUIDED  │  │  • OBSTACLE_DISTANCE │     │  • velocity setpoints│
│  • ARM (opt.)  │  │     (sectors, cm)    │     │     (SET_POSITION_*) │
│  • DO_REPOSITION│ └──────────────────────┘     │ Listens to FC stream │
│  • mission pull│                                └──────────────────────┘
│ Reads:         │
│  • HEARTBEAT   │
│  • GLOBAL_POS  │
└────────────────┘
```

---

## Message & Port Cheat-Sheet

```text
FC ↔ Router (/dev/telem1, 115200)   : MAVLink2 (all traffic)
Runner ↔ Router (tcp:127.0.0.1:5760): HEARTBEAT, MISSION_*, COMMAND_LONG (DO_REPOSITION), GLOBAL_POSITION_INT
Bridge → Router (udpout:127.0.0.1:14553): OBSTACLE_DISTANCE (uint16 cm array, increment/offset)
SegMAV TX → Router (127.0.0.1:14555): SET_POSITION_TARGET_* (velocity “nudges”)
Router → SegMAV RX (127.0.0.1:14601): FC stream (HEARTBEAT, etc.)
MP ↔ FC (SiK link)                   : Mission write/arm/telemetry (parallel to Jetson path)
```

---

## “During Mission” Communication Flow (Happy Path)

```text
1) MP → FC (SiK): WRITE mission.
2) Runner → Router (TCP 5760): HEARTBEAT; wait for FC HEARTBEAT (via router).
3) Runner → FC: set GUIDED; (optional) ARM; pull mission; send DO_REPOSITION to WP#1.
4) FC → Runner: GLOBAL_POSITION_INT at ~10 Hz; Runner checks distance-to-WP.
5) LiDAR driver → Bridge: /scan (ROS).
6) Bridge → FC (via Router UDP 14553): OBSTACLE_DISTANCE at ~10 Hz.
7) SegMAV → FC (via Router 14555): velocity nudges when needed.
8) FC: BendyRuler uses OBSTACLE_DISTANCE to avoid; mixes with GUIDED target & SegMAV nudges.
9) Runner: when distance ≤ acceptance, HOLD if needed, then DO_REPOSITION to next WP.
10) Repeat until last WP → Runner prints “Mission complete”.
```

---

## Failure Isolation (Mini Flow)

```text
No HEARTBEAT in runner?
  → Router shows “Opened TCP Server … :5760”?
  → sudo ss -ltnp | grep :5760
  → If OK, test FC serial: stop router; open /dev/telem1 @115200; wait_heartbeat().

No Proximity ring?
  → Bridge running on udpout:127.0.0.1:14553?
  → MP Ctrl-F → Mavlink Inspector: OBSTACLE_DISTANCE rate > 0?

SegMAV nudges not applied?
  → Router listening on 14555? Pushing on 14601?
  → Vehicle in GUIDED? (nudges ignored in MANUAL/AUTO).
```

---

## Notes
- TCP for the runner avoids UDP “peer learning” issues. LiDAR + SegMAV stay on UDP as configured.
- FC once-only params: `PRX_TYPE=5`, `AVOID_ENABLE=1`, `OA_TYPE=1` (BendyRuler), optional `AVOID_MARGIN=1.0`.
- If you later consolidate hubs to UDP `:14550`, change the bridge `--hub` accordingly. The runner can stay on TCP `:5760`.

*End of README.*
