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

    # 8 秒、2Hz，共 16 筆
    hz = 2.0
    dt = 1.0 / hz
    burst_count = 16

    print("Start L2 burst (8 sec)...")
    for i in range(burst_count):
        now_ms = int((time.time() - t0) * 1000)

        m.mav.ai_landing_status_send(
            now_ms,
            0.40,   # conf
            1,      # lost
            0.40,   # reproj
            0.30    # cov
        )

        m.mav.ai_landing_correction_send(
            now_ms,
            0.00,   # roll
            -0.03,  # pitch
            0.00,   # yaw
            0.20,   # x
            0.10,   # y
            -1.00,  # z
            1.20,   # dist
            0.40,   # conf
            FRAME_CAMERA_FRD,
            FLAG_YAW_VALID | FLAG_DISTANCE_VALID
        )

        print(f"  sent L2 sample {i+1}/{burst_count}")
        time.sleep(dt)

    print("L2 burst finished.")
    print("Sender exits now.")


if __name__ == "__main__":
    main()