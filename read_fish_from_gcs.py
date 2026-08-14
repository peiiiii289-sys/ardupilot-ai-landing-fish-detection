import os
import sys

sys.path.insert(0, os.path.expanduser("~/ardupilot/modules/mavlink"))
sys.path.insert(0, os.path.expanduser("~/ardupilot/modules/mavlink/pymavlink"))

os.environ["MAVLINK20"] = "1"
os.environ["MAVLINK_DIALECT"] = "ardupilotmega"

from pymavlink import mavutil

m = mavutil.mavlink_connection("udpin:127.0.0.1:14560")

print("Waiting heartbeat...")
m.wait_heartbeat()
print("Connected")

while True:
    msg = m.recv_match(type="AI_FISH_DETECTION_RESULT", blocking=True, timeout=5)
    if msg:
        print(msg)