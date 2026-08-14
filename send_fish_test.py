import os
import sys
import time

sys.path.insert(0, os.path.expanduser("~/ardupilot/modules/mavlink"))
sys.path.insert(0, os.path.expanduser("~/ardupilot/modules/mavlink/pymavlink"))

os.environ["MAVLINK20"] = "1"
os.environ["MAVLINK_DIALECT"] = "ardupilotmega"

from pymavlink import mavutil

m = mavutil.mavlink_connection(
    "tcp:127.0.0.1:5762",
    source_system=42,
    source_component=211
)

print("Waiting heartbeat...")
m.wait_heartbeat()
print("Connected")

print("HAS_FISH_CLASS =", hasattr(mavutil.mavlink, "MAVLink_ai_fish_detection_result_message"))
print("HAS_FISH_SEND  =", hasattr(mavutil.mavlink.MAVLink, "ai_fish_detection_result_send"))

start_ms = int(time.monotonic() * 1000)

while True:
    now_ms = int(time.monotonic() * 1000) - start_ms

    m.mav.ai_fish_detection_result_send(
        now_ms,   # uint32 time_boot_ms
        640,
        480,
        45.0,
        32,
        78.0,
        2,
        18.5
    )
    time.sleep(0.2)