import os
import sys
import time

sys.path.insert(0, "/home/lucia/ardupilot/modules/mavlink")
os.environ["MAVLINK20"] = "1"
os.environ["MAVLINK_DIALECT"] = "ardupilotmega"

from pymavlink import mavutil

m = mavutil.mavlink_connection("tcp:127.0.0.1:5762")
m.wait_heartbeat()
print("Connected: L2 test")

start_time = time.monotonic()

def now_ms():
    return int((time.monotonic() - start_time) * 1000)

print("Sending only L2 vector: conf=0.40 lost=1 reproj=0.40 cov=0.30")

for i in range(40):
    print(f"send #{i+1}")
    m.mav.ai_landing_status_send(
        now_ms(),
        0.40,   # conf < 0.5 -> L2
        1,      # target_lost = 1 -> L2
        0.40,   # reproj > 0.3 -> L2
        0.30    # cov > 0.2 -> L2
    )
    time.sleep(0.1)

print("L2 test done")