import os
import sys

# 強制使用專案內的 pymavlink，而不是 venv/site-packages 舊版本
sys.path.insert(0, "/home/lucia/ardupilot/modules/mavlink")

os.environ["MAVLINK20"] = "1"
os.environ["MAVLINK_DIALECT"] = "ardupilotmega"

import time
from pymavlink import mavutil

# === 連線 ===
m = mavutil.mavlink_connection(
    'tcp:127.0.0.1:5762',
    source_system=42,
    source_component=211
)

print("Waiting heartbeat...")
m.wait_heartbeat()
print("Heartbeat OK")
print("HAS ai_landing_correction_send:", hasattr(m.mav, "ai_landing_correction_send"))

# === 註冊 routing（很重要）===
for i in range(5):
    m.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0, 0, 0
    )
    print(f"AI heartbeat {i+1}/5 sent")
    time.sleep(0.2)

# === 固定測試數據 ===
x_err = 10.0
y_err = 20.0
z_err = 30.0

print("\nStart sending correction...")

for i in range(30):
    now_ms = int(time.time() * 1000) & 0xFFFFFFFF

    m.mav.ai_landing_correction_send(
        now_ms,
        0.0,     # roll_err
        0.0,     # pitch_err
        0.1,     # yaw_err
        x_err,
        y_err,
        z_err,
        5.0,     # distance
        0.9,     # confidence
        200,     # frame
        3        # flags: yaw_valid + distance_valid
    )

    print(f"Sent {i+1}/30 | raw=({x_err},{y_err},{z_err})")
    time.sleep(0.1)

print("Done")