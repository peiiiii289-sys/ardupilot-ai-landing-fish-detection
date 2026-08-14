#!/usr/bin/env python3
import os
import time
from pymavlink import mavutil

os.environ["MAVLINK20"] = "1"
os.environ["MAVLINK_DIALECT"] = "ardupilotmega"

MASTER = "tcp:127.0.0.1:5762"

def now_ms():
    return int(time.monotonic() * 1000) & 0xFFFFFFFF

def send_ai_heartbeat(m):
    m.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        mavutil.mavlink.MAV_STATE_ACTIVE
    )

def send_status(m, conf, lost, reproj, cov):
    m.mav.ai_landing_status_send(
        now_ms(),
        float(conf),
        int(lost),
        float(reproj),
        float(cov)
    )

def send_corr(m, conf):
    m.mav.ai_landing_correction_send(
        now_ms(),
        0.0,          # roll_err
        -0.02,        # pitch_err
        0.0,          # yaw_err
        1.0,          # x_err
        2.0,          # y_err
        3.0,          # z_err
        2.50,         # distance
        float(conf),  # confidence
        mavutil.mavlink.MAV_FRAME_BODY_FRD,
        0             # flags
    )

def stream_case(m, name, conf, lost, reproj, cov, seconds):
    print(f"\n[CASE] {name}")
    print(f"  conf={conf} lost={lost} reproj={reproj} cov={cov} duration={seconds}s")
    end = time.time() + seconds
    while time.time() < end:
        send_status(m, conf, lost, reproj, cov)
        send_corr(m, conf)
        time.sleep(0.1)

def main():
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
        send_ai_heartbeat(m)
        print(f"  AI heartbeat {i+1}/5 sent")
        time.sleep(0.3)

    # 先送正常狀態，確認 L1/L2/L3 都是 0
    stream_case(m, "NORMAL_BASELINE", 0.90, 0, 0.05, 0.02, 4)

    # L1：低信心，但沒有 target_lost
    stream_case(m, "L1_LOW_CONF", 0.69, 0, 0.05, 0.02, 5)

    # 恢復正常，讓 L1 收回
    stream_case(m, "RECOVER_AFTER_L1", 0.90, 0, 0.05, 0.02, 4)

    # L2：target_lost + conf/reproj/cov 都壞
    stream_case(m, "L2_TARGET_LOST_FIRST", 0.40, 1, 0.35, 0.25, 5)

    # 恢復正常，製造第二次新的 L2 事件
    stream_case(m, "RECOVER_AFTER_L2", 0.90, 0, 0.05, 0.02, 5)

    # 第二次 L2：用來觸發 repeated-L2 → L3
    stream_case(m, "L2_TARGET_LOST_SECOND_FOR_L3", 0.40, 1, 0.35, 0.25, 7)

    print("\n[DONE] Story 4.2 L1/L2/L3 injection completed")

if __name__ == "__main__":
    main()
