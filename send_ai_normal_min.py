#!/usr/bin/env python3
import sys
import os
import time

sys.path.insert(0, os.path.expanduser("~/ardupilot/modules/mavlink"))

os.environ["MAVLINK20"] = "1"
os.environ["MAVLINK_DIALECT"] = "ardupilotmega"

from pymavlink import mavutil

AI_SYSID = 42
AI_COMPID = 211
TARGET = "tcp:127.0.0.1:5762"

FRAME_CAMERA_FRD = 200
FLAG_YAW_VALID = 1 << 0
FLAG_DISTANCE_VALID = 1 << 1


def main():
    print(f"Connecting to {TARGET} ...")
    m = mavutil.mavlink_connection(
        TARGET,
        source_system=AI_SYSID,
        source_component=AI_COMPID,
        dialect="ardupilotmega"
    )

    print("Waiting heartbeat...")
    hb = m.wait_heartbeat(timeout=10)
    if hb is None:
        raise RuntimeError("No heartbeat from FC")
    print(f"Heartbeat OK: target sys={m.target_system} comp={m.target_component}")

    print("Sending warm-up heartbeats...")
    for i in range(5):
        m.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0
        )
        print(f"  heartbeat {i+1}/5")
        time.sleep(0.2)

    t0 = time.time()
    print("Start streaming normal AI landing data... Ctrl+C to stop")

    while True:
        now_ms = int((time.time() - t0) * 1000)

        m.mav.ai_landing_status_send(
            now_ms,
            0.90,
            0,
            0.05,
            0.02
        )

        m.mav.ai_landing_correction_send(
            now_ms,
            0.00,
            0.00,
            0.02,
            1.00,
            0.50,
            -2.00,
            2.50,
            0.90,
            FRAME_CAMERA_FRD,
            FLAG_YAW_VALID | FLAG_DISTANCE_VALID
        )

        time.sleep(0.1)


if __name__ == "__main__":
    main()