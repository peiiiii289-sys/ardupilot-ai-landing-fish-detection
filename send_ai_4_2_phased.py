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
    print(f"corr  : roll={roll} pitch={pitch} yaw={yaw} x={x} y={y} z={z} dist={dist}")

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

    print("Connected (skip heartbeat wait)")
    print(f"Heartbeat OK: target sys={m.target_system} comp={m.target_component}")

    send_heartbeat(m)

    t0 = time.time()

    # Phase 0: normal warmup
    stream_phase(
        m, t0,
        name="PHASE0_NORMAL_WARMUP",
        hz=2.0,
        duration_sec=5,
        conf=0.90, lost=False, reproj=0.05, cov=0.02,
        roll=0.00, pitch=0.00, yaw=0.02,
        x=1.00, y=0.50, z=-2.00, dist=2.50
    )

    # Phase 1: first L2 burst
    print(">>> 目標：第一次 L2，理想上應看到 L2=1 L3=0 R=2")
    stream_phase(
        m, t0,
        name="PHASE1_FIRST_L2",
        hz=2.0,
        duration_sec=6,
        conf=0.40, lost=True, reproj=0.40, cov=0.30,
        roll=0.00, pitch=-0.03, yaw=0.00,
        x=0.20, y=0.10, z=-1.00, dist=1.20
    )

    # Phase 2: longer normal gap for re-enter
    print(">>> 現在回 normal，請把飛機整理回 QHOVER，準備二次進場")
    print(">>> 建議這一段內在 MAVProxy 做：")
    print(">>> mode QHOVER")
    print(">>> arm throttle force")
    print(">>> rc 3 1600")
    print(">>> 然後在這段中間偏前時，手動打一次：mode QLAND")
    print(">>> 建議在 sample 8/40 到 sample 20/40 之間打")
    stream_phase(
        m, t0,
        name="PHASE2_NORMAL_GAP_FOR_REENTER",
        hz=2.0,
        duration_sec=20,
        conf=0.90, lost=False, reproj=0.05, cov=0.02,
        roll=0.00, pitch=0.00, yaw=0.02,
        x=0.50, y=0.20, z=-2.00, dist=2.20
    )

    # Phase 3: second L2 burst, longer
    print(">>> 目標：第二次 L2，理想上應看到 L2=1 L3=1 R=5")
    stream_phase(
        m, t0,
        name="PHASE3_SECOND_L2",
        hz=2.0,
        duration_sec=12,
        conf=0.40, lost=True, reproj=0.40, cov=0.30,
        roll=0.00, pitch=-0.03, yaw=0.00,
        x=0.20, y=0.10, z=-1.00, dist=1.20
    )

    # Phase 4: short hold for observation
    print(">>> 最後保留 short hold，方便你看 log")
    stream_phase(
        m, t0,
        name="PHASE4_POST_HOLD",
        hz=2.0,
        duration_sec=6,
        conf=0.40, lost=True, reproj=0.40, cov=0.30,
        roll=0.00, pitch=-0.03, yaw=0.00,
        x=0.20, y=0.10, z=-1.00, dist=1.20
    )

    print("All phases complete. Sender exits now.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped by user.")