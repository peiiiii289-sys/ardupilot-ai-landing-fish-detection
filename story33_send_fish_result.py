#!/usr/bin/env python3
import argparse
import os
import time
from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink

os.environ["MAVLINK20"] = "1"
os.environ["MAVLINK_DIALECT"] = "ardupilotmega"

MASTER = "tcp:127.0.0.1:5762"

def now_ms():
    return int(time.monotonic() * 1000) & 0xFFFFFFFF

def build_fish_msg():
    cls = mavlink.MAVLink_ai_fish_detection_result_message
    fields = list(cls.fieldnames)

    values = {
        "time_boot_ms": now_ms(),
        "fish_coverage_pct": 45.0,
        "fish_count": 32,
        "tuna_similarity_pct": 88.5,
        "bird_count": 3,
        "inference_fps": 12.5,
        "image_width": 640,
        "image_height": 480,
    }

    args = [values.get(name, 0) for name in fields]

    print("[FIELDS]", fields)
    print("[VALUES]", {name: values.get(name, 0) for name in fields})

    return cls(*args)

def send_heartbeat(m):
    for i in range(5):
        m.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            mavutil.mavlink.MAV_STATE_ACTIVE
        )
        print(f"[AI_HEARTBEAT] {i+1}/5")
        time.sleep(0.2)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--compid", type=int, default=211,
                        help="source component id. Use 211 for valid AI, 190 for invalid source test.")
    parser.add_argument("--count", type=int, default=5)
    parser.add_argument("--rate", type=float, default=2.0)
    args = parser.parse_args()

    print(f"[CONNECT] {MASTER} as sys=1 comp={args.compid}")

    m = mavutil.mavlink_connection(
        MASTER,
        source_system=1,
        source_component=args.compid,
        autoreconnect=True,
        dialect="ardupilotmega",
    )

    print("[WAIT] heartbeat from FC...")
    m.wait_heartbeat()
    print(f"[OK] heartbeat from sys={m.target_system} comp={m.target_component}")

    send_heartbeat(m)

    delay = 1.0 / args.rate

    for i in range(args.count):
        msg = build_fish_msg()
        m.mav.send(msg)
        print(f"[SEND] AI_FISH_DETECTION_RESULT {i+1}/{args.count} compid={args.compid}")
        time.sleep(delay)

    print("[DONE] fish result send completed")

if __name__ == "__main__":
    main()
