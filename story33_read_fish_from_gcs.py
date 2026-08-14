#!/usr/bin/env python3
import os
import time
from pymavlink import mavutil

os.environ["MAVLINK20"] = "1"
os.environ["MAVLINK_DIALECT"] = "ardupilotmega"

MASTER = "udpin:127.0.0.1:14560"

def main():
    print(f"[GCS_CONNECT] {MASTER}")
    m = mavutil.mavlink_connection(
        MASTER,
        source_system=250,
        source_component=190,
        dialect="ardupilotmega",
    )

    print("[GCS_WAIT] AI_FISH_DETECTION_RESULT for 15 seconds...")
    deadline = time.time() + 15
    got = 0

    while time.time() < deadline:
        msg = m.recv_match(type="AI_FISH_DETECTION_RESULT", blocking=True, timeout=1)
        if msg is None:
            continue

        got += 1
        d = msg.to_dict()
        print("[GCS_RX] AI_FISH_DETECTION_RESULT", d)

    print(f"[GCS_DONE] received={got}")

if __name__ == "__main__":
    main()
