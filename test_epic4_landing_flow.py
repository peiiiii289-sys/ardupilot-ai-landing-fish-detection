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

while True:
    now_ms = int((time.monotonic() - start_time) * 1000)

    m.mav.ai_landing_status_send(
        now_ms,
        0.9,    # visual_confidence
        0,      # target_lost
        0.05,   # reproj_error
        0.02    # covariance
    )

    m.mav.ai_landing_correction_send(
        now_ms,
        0.0, 0.0, 0.05,   # roll_err, pitch_err, yaw_err
        1.0, 0.5, -2.0,   # x_err, y_err, z_err
        2.5,              # distance
        0.9,              # confidence
        200,              # frame
        3                 # flags
    )

    time.sleep(0.1)