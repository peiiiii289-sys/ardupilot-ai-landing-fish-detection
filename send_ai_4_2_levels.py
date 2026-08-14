import sys
import os
import time
import math
import argparse

sys.path.insert(0, os.path.expanduser("~/ardupilot/modules/mavlink"))

os.environ["MAVLINK20"] = "1"
os.environ["MAVLINK_DIALECT"] = "ardupilotmega"

from pymavlink import mavutil

AI_SYSID = 42
AI_COMPID = 211   # MAV_COMP_ID_UNICO_AI_COMPUTER
TARGET_CONN = "tcp:127.0.0.1:5762"

# correction frame:
# 依你目前專案脈絡，實作端用的是自訂 landing frame；你現有測試主線以 200 使用較一致
FRAME_CAMERA_FRD = 200

# flags
FLAG_YAW_VALID = 1 << 0
FLAG_DISTANCE_VALID = 1 << 1


def connect():
    print(f"Connecting to {TARGET_CONN} as AI mock...")
    m = mavutil.mavlink_connection(
        TARGET_CONN,
        source_system=AI_SYSID,
        source_component=AI_COMPID,
        dialect="ardupilotmega"
    )
    print("Waiting heartbeat...")
    hb = m.wait_heartbeat(timeout=10)
    if hb is None:
        raise RuntimeError("No heartbeat from FC")
    print(f"Heartbeat OK: target sys={m.target_system} comp={m.target_component}")

    # 先送幾個 AI heartbeat，讓 routing / source 穩定
    print("Sending AI heartbeat warm-up...")
    for i in range(5):
        m.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0
        )
        print(f"  AI heartbeat {i+1}/5 sent")
        time.sleep(0.2)
    return m


def now_ms(t0):
    return int((time.time() - t0) * 1000)


def send_status(m, t0, conf, lost, reproj, cov):
    m.mav.ai_landing_status_send(
        now_ms(t0),
        conf,
        int(lost),
        reproj,
        cov
    )


def send_corr(m, t0, roll, pitch, yaw, x, y, z, dist, conf, flags):
    m.mav.ai_landing_correction_send(
        now_ms(t0),
        roll,
        pitch,
        yaw,
        x,
        y,
        z,
        dist,
        conf,
        FRAME_CAMERA_FRD,
        flags
    )


def stream_case(m, mode):
    t0 = time.time()
    hz = 10.0
    dt = 1.0 / hz

    print(f"Streaming mode={mode}")

    if mode == "normal":
        # 4.1 核心 landing flow 驗證
        while True:
            send_status(m, t0, conf=0.90, lost=False, reproj=0.05, cov=0.02)
            send_corr(
                m, t0,
                roll=0.00, pitch=0.00, yaw=0.02,
                x=1.00, y=0.50, z=-2.00,
                dist=2.50, conf=0.90,
                flags=FLAG_YAW_VALID | FLAG_DISTANCE_VALID
            )
            time.sleep(dt)

    elif mode == "l1":
        # conf < 0.7 or reproj > 0.1 or cov > 0.05 -> L1
        while True:
            send_status(m, t0, conf=0.60, lost=False, reproj=0.12, cov=0.02)
            send_corr(
                m, t0,
                roll=0.00, pitch=-0.02, yaw=0.01,
                x=0.50, y=0.20, z=-2.00,
                dist=2.20, conf=0.60,
                flags=FLAG_YAW_VALID | FLAG_DISTANCE_VALID
            )
            time.sleep(dt)

    elif mode == "l2":
        # lost=1 / conf<0.5 / reproj>0.3 / cov>0.2 -> L2
        while True:
            send_status(m, t0, conf=0.40, lost=True, reproj=0.40, cov=0.30)
            send_corr(
                m, t0,
                roll=0.00, pitch=-0.03, yaw=0.00,
                x=0.20, y=0.10, z=-1.00,
                dist=1.20, conf=0.40,
                flags=FLAG_YAW_VALID | FLAG_DISTANCE_VALID
            )
            time.sleep(dt)

    elif mode == "l3":
        # repeated L2 within 1 minute
        # 第一段 L2 burst -> normal -> 第二段 L2 burst
        print("Phase 1: L2 burst")
        for _ in range(20):
            send_status(m, t0, conf=0.40, lost=True, reproj=0.40, cov=0.30)
            send_corr(
                m, t0,
                roll=0.00, pitch=-0.03, yaw=0.00,
                x=0.20, y=0.10, z=-1.00,
                dist=1.20, conf=0.40,
                flags=FLAG_YAW_VALID | FLAG_DISTANCE_VALID
            )
            time.sleep(dt)

        print("Phase 2: normal stream")
        for _ in range(20):
            send_status(m, t0, conf=0.90, lost=False, reproj=0.05, cov=0.02)
            send_corr(
                m, t0,
                roll=0.00, pitch=0.00, yaw=0.02,
                x=0.50, y=0.20, z=-2.00,
                dist=2.20, conf=0.90,
                flags=FLAG_YAW_VALID | FLAG_DISTANCE_VALID
            )
            time.sleep(dt)

        print("Phase 3: keep normal stream alive; wait user re-enter QLAND")
        # 這段很重要：先不要讓 link timeout，等你手動再進 QLAND
        while True:
            send_status(m, t0, conf=0.90, lost=False, reproj=0.05, cov=0.02)
            send_corr(
                m, t0,
                roll=0.00, pitch=0.00, yaw=0.02,
                x=0.50, y=0.20, z=-2.00,
                dist=2.20, conf=0.90,
                flags=FLAG_YAW_VALID | FLAG_DISTANCE_VALID
            )
            time.sleep(dt)

    else:
        raise ValueError(f"Unknown mode: {mode}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", required=True, choices=["normal", "l1", "l2", "l3"])
    args = ap.parse_args()

    m = connect()
    try:
        stream_case(m, args.mode)
    except KeyboardInterrupt:
        print("Stopped by user")


if __name__ == "__main__":
    main()