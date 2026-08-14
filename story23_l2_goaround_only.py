#!/usr/bin/env python3
import os
import time
from pymavlink import mavutil

os.environ["MAVLINK20"] = "1"
os.environ["MAVLINK_DIALECT"] = "ardupilotmega"

MASTER = "tcp:127.0.0.1:5762"

def now_ms():
    return int(time.monotonic() * 1000) & 0xFFFFFFFF

def connect():
    print(f"[CONNECT] {MASTER}")
    m = mavutil.mavlink_connection(
        MASTER,
        source_system=1,
        source_component=211,
        autoreconnect=True,
        dialect="ardupilotmega",
    )

    print("[WAIT] heartbeat...")
    m.wait_heartbeat()
    print(f"[OK] heartbeat from sys={m.target_system} comp={m.target_component}")

    print("[ROUTE] sending AI heartbeat...")
    for i in range(5):
        m.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            mavutil.mavlink.MAV_STATE_ACTIVE
        )
        print(f"  AI heartbeat {i+1}/5 sent")
        time.sleep(0.2)

    return m

def send_l2_status(m):
    m.mav.ai_landing_status_send(
        now_ms(),
        0.40,   # conf：低信心
        1,      # lost：目標遺失
        0.35,   # reproj：重投影誤差大
        0.25    # cov：不確定性高
    )

def send_l2_corr(m):
    m.mav.ai_landing_correction_send(
        now_ms(),
        0.0,      # roll_err
        -0.02,    # pitch_err
        0.0,      # yaw_err
        1.0,      # x_err
        2.0,      # y_err
        3.0,      # z_err
        2.50,     # distance
        0.40,     # confidence
        mavutil.mavlink.MAV_FRAME_BODY_FRD,
        3         # flags 有效
    )

def main():
    m = connect()

    print("[CASE] STORY_2_3_L2_GO_AROUND_ONLY")
    print("[SEND] L2 status + correction for 10 seconds")

    end = time.time() + 10.0
    while time.time() < end:
        send_l2_status(m)
        send_l2_corr(m)
        time.sleep(0.1)

    print("[DONE] L2 go-around injection finished")
    print("[NEXT] grep story23_l2_goaround.log for AI_LANDING_ABORT_L2 / AI_GO_AROUND")

if __name__ == "__main__":
    main()
