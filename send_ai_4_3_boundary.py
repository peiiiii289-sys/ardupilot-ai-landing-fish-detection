#!/usr/bin/env python3
import sys
import os
import time

sys.path.insert(0, os.path.expanduser("~/ardupilot/modules/mavlink"))

os.environ["MAVLINK20"] = "1"
os.environ["MAVLINK_DIALECT"] = "ardupilotmega"

from pymavlink import mavutil

AI_SYSID = 42
AI_COMPID = 211
TARGET = "tcp:127.0.0.1:5762"

FRAME_CAMERA_FRD = 200
FLAG_YAW_VALID = 1 << 0
FLAG_DISTANCE_VALID = 1 << 1


def send_heartbeat(m, count=5, dt=0.2):
    print("Sending warm-up heartbeats...")
    for i in range(count):
        m.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0
        )
        print(f"  heartbeat {i+1}/{count}")
        time.sleep(dt)


def send_status(m, now_ms, conf, lost, reproj, cov):
    m.mav.ai_landing_status_send(
        now_ms,
        conf,
        int(lost),
        reproj,
        cov
    )


def send_corr(m, now_ms, roll, pitch, yaw, x, y, z, dist, conf):
    m.mav.ai_landing_correction_send(
        now_ms,
        roll,
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


def stream_phase(m, t0, name, hz, duration_sec,
                 conf, lost, reproj, cov,
                 roll, pitch, yaw, x, y, z, dist):
    dt = 1.0 / hz
    count = int(duration_sec * hz)

    print(f"\n===== {name} START =====")
    print(f"duration={duration_sec}s hz={hz} count={count}")
    print(f"status: conf={conf} lost={int(lost)} reproj={reproj} cov={cov}")
    print(f"corr  : roll={roll:.3f} pitch={pitch:.3f} yaw={yaw:.3f} x={x:.2f} y={y:.2f} z={z:.2f} dist={dist:.2f}")

    for i in range(count):
        now_ms = int((time.time() - t0) * 1000)

        send_status(m, now_ms, conf, lost, reproj, cov)
        send_corr(m, now_ms, roll, pitch, yaw, x, y, z, dist, conf)

        print(f"  [{name}] sample {i+1}/{count}")
        time.sleep(dt)

    print(f"===== {name} END =====\n")


def main():
    print(f"Connecting to {TARGET} ...")
    m = mavutil.mavlink_connection(
        TARGET,
        source_system=AI_SYSID,
        source_component=AI_COMPID,
        dialect="ardupilotmega"
    )

    print("Waiting heartbeat...")
    hb = m.wait_heartbeat(timeout=10)
    if hb is None:
        raise RuntimeError("No heartbeat from FC")
    print(f"Heartbeat OK: target sys={m.target_system} comp={m.target_component}")

    send_heartbeat(m)
    t0 = time.time()

    print(">>> 目標：驗證 4.3 Boundary / Protection")
    print(">>> 你會先看到 normal，再看到 L1 邊界，再看到 L2 邊界")
    print(">>> 如果在 L2 段成功，理想上應看到：AI_LANDING_ABORT_L2 / AI_GO_AROUND")

    # Normal reference
    stream_phase(
        m, t0,
        name="PHASE0_NORMAL_REFERENCE",
        hz=10.0,
        duration_sec=5,
        conf=0.90, lost=False, reproj=0.05, cov=0.02,
        roll=0.01, pitch=-0.02, yaw=0.03,
        x=0.80, y=0.30, z=-2.00, dist=2.20
    )

    # Just above L1 threshold: should remain normal
    stream_phase(
        m, t0,
        name="PHASE1_ABOVE_L1_THRESHOLD",
        hz=10.0,
        duration_sec=5,
        conf=0.71, lost=False, reproj=0.09, cov=0.049,
        roll=0.01, pitch=-0.02, yaw=0.03,
        x=0.80, y=0.30, z=-2.00, dist=2.20
    )

    # Just below L1 threshold: should trigger L1
    stream_phase(
        m, t0,
        name="PHASE2_BELOW_L1_THRESHOLD",
        hz=10.0,
        duration_sec=6,
        conf=0.69, lost=False, reproj=0.11, cov=0.06,
        roll=0.01, pitch=-0.02, yaw=0.03,
        x=0.80, y=0.30, z=-2.00, dist=2.20
    )

    # Recover to normal
    stream_phase(
        m, t0,
        name="PHASE3_RECOVER_TO_NORMAL",
        hz=10.0,
        duration_sec=5,
        conf=0.90, lost=False, reproj=0.05, cov=0.02,
        roll=0.01, pitch=-0.02, yaw=0.03,
        x=0.80, y=0.30, z=-2.00, dist=2.20
    )

    # Just above L2 threshold: ideally no L2 yet
    stream_phase(
        m, t0,
        name="PHASE4_ABOVE_L2_THRESHOLD",
        hz=10.0,
        duration_sec=5,
        conf=0.51, lost=False, reproj=0.29, cov=0.19,
        roll=0.01, pitch=-0.02, yaw=0.03,
        x=0.80, y=0.30, z=-2.00, dist=2.20
    )

    # Just below L2 threshold: should trigger L2 / go-around
    stream_phase(
        m, t0,
        name="PHASE5_BELOW_L2_THRESHOLD",
        hz=10.0,
        duration_sec=8,
        conf=0.49, lost=True, reproj=0.31, cov=0.21,
        roll=0.01, pitch=-0.02, yaw=0.03,
        x=0.80, y=0.30, z=-2.00, dist=2.20
    )

    # Hold for observation
    stream_phase(
        m, t0,
        name="PHASE6_POST_HOLD",
        hz=10.0,
        duration_sec=6,
        conf=0.49, lost=True, reproj=0.31, cov=0.21,
        roll=0.01, pitch=-0.02, yaw=0.03,
        x=0.80, y=0.30, z=-2.00, dist=2.20
    )

    print("All phases complete. Sender exits now.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped by user.")