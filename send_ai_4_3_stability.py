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
AI_COMPID = 211
TARGET_CONN = "tcp:127.0.0.1:5762"
FRAME_CAMERA_FRD = 200

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

    for _ in range(5):
        m.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0
        )
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


def send_corr(m, t0, pitch, conf=0.90, yaw=0.01, x=0.3, y=0.2, z=-2.0, dist=2.2):
    m.mav.ai_landing_correction_send(
        now_ms(t0),
        0.0,
        pitch,
        yaw,
        x,
        y,
        z,
        dist,
        conf,
        FRAME_CAMERA_FRD,
        FLAG_YAW_VALID | FLAG_DISTANCE_VALID
    )


def case_step(m):
    # Case A: step input
    # 先小、再突增、再維持、再回小
    t0 = time.time()
    hz = 10.0
    dt = 1.0 / hz

    while True:
        t = time.time() - t0

        if t < 3.0:
            pitch = 0.00
        elif t < 6.0:
            pitch = -0.10   # 大 step，剛好打到 mode_qland constrain 上限附近
        elif t < 9.0:
            pitch = -0.05
        else:
            pitch = 0.00

        send_status(m, t0, conf=0.90, lost=False, reproj=0.05, cov=0.02)
        send_corr(m, t0, pitch=pitch, conf=0.90)
        time.sleep(dt)


def case_boundary(m):
    # Case C: boundary / protection
    # 在邊界附近切換，觀察 L1 / L2 保護是否穩定觸發
    t0 = time.time()
    hz = 10.0
    dt = 1.0 / hz

    while True:
        t = time.time() - t0

        if t < 4.0:
            # 接近 L1 邊界
            conf = 0.69
            lost = False
            reproj = 0.11
            cov = 0.04
        elif t < 8.0:
            # 接近 L2 邊界
            conf = 0.49
            lost = False
            reproj = 0.31
            cov = 0.10
        else:
            # 再拉回 normal
            conf = 0.90
            lost = False
            reproj = 0.05
            cov = 0.02

        send_status(m, t0, conf=conf, lost=lost, reproj=reproj, cov=cov)
        send_corr(m, t0, pitch=-0.03, conf=conf)
        time.sleep(dt)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--case", required=True, choices=["step", "boundary"])
    args = ap.parse_args()

    m = connect()

    try:
        if args.case == "step":
            case_step(m)
        else:
            case_boundary(m)
    except KeyboardInterrupt:
        print("Stopped by user")


if __name__ == "__main__":
    main()