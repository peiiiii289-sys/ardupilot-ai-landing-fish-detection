import os
import sys
import time

sys.path.insert(0, "/home/lucia/ardupilot/modules/mavlink")
os.environ["MAVLINK20"] = "1"
os.environ["MAVLINK_DIALECT"] = "ardupilotmega"

from pymavlink import mavutil

m = mavutil.mavlink_connection("tcp:127.0.0.1:5762")
m.wait_heartbeat()

print("Connected")

start_time = time.monotonic()

def now_ms():
    return int((time.monotonic() - start_time) * 1000)

def send_status(conf, lost, reproj, cov):
    m.mav.ai_landing_status_send(
        now_ms(),
        conf,
        lost,
        reproj,
        cov
    )

while True:

    # =========================
    print("=== NORMAL ===")
    for _ in range(20):
        send_status(0.9, 0, 0.05, 0.02)
        time.sleep(0.1)

    # =========================
    print("=== L1 ===")
    for _ in range(30):
        send_status(0.6, 0, 0.12, 0.02)
        time.sleep(0.1)

    # =========================
    print("=== L2 ===")
    for _ in range(30):
        send_status(0.4, 1, 0.4, 0.3)
        time.sleep(0.1)

    # =========================
    print("=== L3 (repeat L2) ===")
    for _ in range(60):
        send_status(0.4, 1, 0.4, 0.3)
        time.sleep(0.1)