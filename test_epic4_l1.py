import os
import sys
import time

sys.path.insert(0, "/home/lucia/ardupilot/modules/mavlink")
os.environ["MAVLINK20"] = "1"
os.environ["MAVLINK_DIALECT"] = "ardupilotmega"

from pymavlink import mavutil

m = mavutil.mavlink_connection("tcp:127.0.0.1:5762")
m.wait_heartbeat()
print("Connected: L1 test")

start_time = time.monotonic()

def now_ms():
    return int((time.monotonic() - start_time) * 1000)

for i in range(40):
    m.mav.ai_landing_status_send(
        now_ms(),
        0.60,   # conf < 0.7 -> L1
        0,      # target_lost
        0.12,   # reproj > 0.1 -> L1
        0.02    # cov normal
    )
    time.sleep(0.1)

print("L1 test done")
