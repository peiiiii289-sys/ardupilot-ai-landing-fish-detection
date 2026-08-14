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

    print(">>> 目標：驗證 4.3 Transition continuity")
    print(">>> 這支刻意維持 yaw / pos correction 全程非零")
    print(">>> 你要做的是：")
    print(">>> 1. 先進 QLAND")
    print(">>> 2. 讓飛機自然往 landing phase 往後跑")
    print(">>> 3. 觀察 phase 切換前後的 QLAND AI / AI_CTRL_USE 是否連續")
    print(">>> 如果你有加 state log，例如 QPOS state changed，那更好抓證據")

    # longer warmup before entering QLAND
    stream_phase(
        m, t0,
        name="PHASE0_NONZERO_WARMUP",
        hz=10.0,
        duration_sec=8,
        conf=0.90, lost=False, reproj=0.05, cov=0.02,
        roll=0.03, pitch=-0.04, yaw=0.08,
        x=1.50, y=-1.00, z=-2.50, dist=3.10
    )

    # main transition observation window
    stream_phase(
        m, t0,
        name="PHASE1_TRANSITION_OBSERVE",
        hz=10.0,
        duration_sec=25,
        conf=0.90, lost=False, reproj=0.05, cov=0.02,
        roll=0.03, pitch=-0.04, yaw=0.08,
        x=1.50, y=-1.00, z=-2.50, dist=3.10
    )

    # slightly changed but still nonzero, to confirm continuity is not accidental
    stream_phase(
        m, t0,
        name="PHASE2_TRANSITION_OBSERVE_VARIANT",
        hz=10.0,
        duration_sec=12,
        conf=0.90, lost=False, reproj=0.05, cov=0.02,
        roll=0.02, pitch=-0.03, yaw=0.06,
        x=1.20, y=-0.80, z=-2.20, dist=2.80
    )

    # final hold
    stream_phase(
        m, t0,
        name="PHASE3_FINAL_HOLD",
        hz=10.0,
        duration_sec=5,
        conf=0.90, lost=False, reproj=0.05, cov=0.02,
        roll=0.02, pitch=-0.03, yaw=0.06,
        x=1.20, y=-0.80, z=-2.20, dist=2.80
    )

    print("All phases complete. Sender exits now.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped by user.")