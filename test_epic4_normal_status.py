import os
import sys
import time

sys.path.insert(0, "/home/lucia/ardupilot/modules/mavlink")
os.environ["MAVLINK20"] = "1"
os.environ["MAVLINK_DIALECT"] = "ardupilotmega"

from pymavlink import mavutil

m = mavutil.mavlink_connection("tcp:127.0.0.1:5762")
m.wait_heartbeat()
print("Connected: normal status sender")

start_time = time.monotonic()

def now_ms():
    return int((time.monotonic() - start_time) * 1000)

for i in range(20):
    print(f"normal send #{i+1}")
    m.mav.ai_landing_status_send(
        now_ms(),
        0.90,
        0,
        0.05,
        0.02
    )
    time.sleep(0.1)

print("Normal status done")