#!/usr/bin/env python3
import os
import sys
import time
import math
import random
import argparse
from pathlib import Path

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

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=int, default=43143)
    parser.add_argument("--seconds", type=float, default=30)
    parser.add_argument("--hz", type=float, default=2)
    parser.add_argument("--master", default="tcp:127.0.0.1:5762")
    args = parser.parse_args()

    rng = random.Random(args.seed)

    print(f"[CONNECT] {args.master}")
    m = mavutil.mavlink_connection(
        args.master,
        source_system=AI_SYSID,
        source_component=AI_COMPID,
        dialect="ardupilotmega",
    )

    print("[WAIT] heartbeat...")
    m.wait_heartbeat(timeout=15)
    print("[OK] heartbeat received")

    print("[CHECK]")
    print("  ai_landing_correction_send =", hasattr(m.mav, "ai_landing_correction_send"))
    print("  ai_landing_status_send     =", hasattr(m.mav, "ai_landing_status_send"))

    if not hasattr(m.mav, "ai_landing_correction_send"):
        print("[FAIL] pymavlink dialect 沒有 ai_landing_correction_send")
        sys.exit(1)

    end_t = time.time() + args.seconds
    dt = 1.0 / args.hz
    seq = 0

    print("[SEND] random raw xyz + random pitch")
    print("[NOTE] MAVProxy 應該看到 raw 隨機、corr = raw + AIL_CAM_X/Y/Z、pitch 也會變")

    while time.time() < end_t:
        seq += 1

        x = rng.uniform(-10.0, 10.0)
        y = rng.uniform(-10.0, 10.0)
        z = rng.uniform(1.0, 30.0)

        pitch = rng.uniform(-0.08, 0.08)   # rad，大約 -4.6° 到 +4.6°
        roll = rng.uniform(-0.03, 0.03)
        yaw = rng.uniform(-0.05, 0.05)

        distance = math.sqrt(x*x + y*y + z*z)
        conf = 0.95

        m.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            mavutil.mavlink.MAV_STATE_ACTIVE,
        )

        # 52100 AI_LANDING_CORRECTION
        m.mav.ai_landing_correction_send(
            now_ms(),
            roll,
            pitch,
            yaw,
            x,
            y,
            z,
            distance,
            conf,
            200,    # CAMERA_FRD
            0b11,   # yaw_valid + distance_valid
        )

        # 52102 AI_LANDING_STATUS
        m.mav.ai_landing_status_send(
            now_ms(),
            0.95,
            0,
            0.01,
            0.01,
        )

        print(
            f"[SEND {seq:03d}] 52100 raw=({x:.2f},{y:.2f},{z:.2f}) "
            f"pitch={pitch:.4f}rad({math.degrees(pitch):.2f}deg) "
            f"dist={distance:.2f} conf={conf:.2f}"
        )

        time.sleep(dt)

    print("[DONE]")

if __name__ == "__main__":
    main()
