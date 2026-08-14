#!/usr/bin/env python3
import os
import sys
import time
from pathlib import Path

# 一定要用 MAVLink2 + ardupilotmega
os.environ["MAVLINK20"] = "1"
os.environ["MAVLINK_DIALECT"] = "ardupilotmega"

HOME = Path.home()
ARDUPILOT = HOME / "ardupilot"

sys.path.insert(0, str(ARDUPILOT / "modules" / "mavlink"))
sys.path.insert(0, str(ARDUPILOT / "modules" / "mavlink" / "pymavlink"))

from pymavlink import mavutil

AI_SYSID = 42
AI_COMPID = 211

def now_ms():
    return int(time.monotonic() * 1000) & 0xFFFFFFFF

print("[CONNECT] tcp:127.0.0.1:5762")

m = mavutil.mavlink_connection(
    "tcp:127.0.0.1:5762",
    source_system=AI_SYSID,
    source_component=AI_COMPID,
    dialect="ardupilotmega",
)

print("[WAIT] heartbeat...")
m.wait_heartbeat(timeout=15)
print("[OK] heartbeat received")

print("[CHECK] send functions:")
print("  ai_landing_correction_send =", hasattr(m.mav, "ai_landing_correction_send"))
print("  ai_landing_status_send     =", hasattr(m.mav, "ai_landing_status_send"))

if not hasattr(m.mav, "ai_landing_correction_send"):
    print("[FAIL] 找不到 ai_landing_correction_send，代表 pymavlink dialect 沒載入成功")
    sys.exit(1)

print("[SEND] start sending AI_LANDING_CORRECTION=52100 and AI_LANDING_STATUS=52102")

while True:
    # AI heartbeat
    m.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        mavutil.mavlink.MAV_STATE_ACTIVE,
    )

    # 52100: AI_LANDING_CORRECTION
    # 這裡故意送非 0，等等 MAVProxy 應該要看到 raw=(9,18,27)
    m.mav.ai_landing_correction_send(
        now_ms(),
        0.0,     # roll_err
        -0.02,   # pitch_err
        0.0,     # yaw_err
        9.0,     # x_err
        18.0,    # y_err
        27.0,    # z_err
        5.0,     # distance
        0.95,    # confidence
        200,     # frame = CAMERA_FRD
        0b11,    # flags: yaw_valid + distance_valid
    )

    # 52102: AI_LANDING_STATUS
    m.mav.ai_landing_status_send(
        now_ms(),
        0.95,    # visual_confidence
        0,       # target_lost
        0.01,    # reproj_error
        0.01,    # covariance
    )

    print("[SEND] CORR 52100 x=9 y=18 z=27 conf=0.95 / STAT 52102 conf=0.95 lost=0")
    time.sleep(0.2)
