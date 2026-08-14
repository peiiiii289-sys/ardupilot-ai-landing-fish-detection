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

def send_status(m, conf=0.90, lost=0, reproj=0.05, cov=0.02):
    m.mav.ai_landing_status_send(
        now_ms(),
        float(conf),
        int(lost),
        float(reproj),
        float(cov)
    )

def send_corr(m, conf=0.90, flags=3, x=1.0, y=2.0, z=3.0, distance=2.50, pitch=-0.02):
    m.mav.ai_landing_correction_send(
        now_ms(),
        0.0,                         # roll_err
        float(pitch),                # pitch_err
        0.0,                         # yaw_err
        float(x),                    # x_err
        float(y),                    # y_err
        float(z),                    # z_err
        float(distance),             # distance
        float(conf),                 # confidence
        mavutil.mavlink.MAV_FRAME_BODY_FRD,
        int(flags)
    )

def stream_case(m, title, seconds, status_kwargs, corr_kwargs, interval=0.1):
    print(f"\n[CASE] {title}: {seconds}s")
    print(f"[STATUS] {status_kwargs}")
    print(f"[CORR]   {corr_kwargs}")

    end = time.time() + seconds
    while time.time() < end:
        send_status(m, **status_kwargs)
        send_corr(m, **corr_kwargs)
        time.sleep(interval)

def pause_no_ai(seconds):
    print(f"\n[CASE] PAUSE_NO_AI_TIMEOUT_BOUNDARY: stop all AI messages for {seconds}s")
    time.sleep(seconds)

def main():
    m = connect()

    # ------------------------------------------------------------
    # CASE 1：Baseline normal
    # 目的：建立正常控制基準，預期 QLAND AI ok=1、AI_CTRL_USE 正常。
    # ------------------------------------------------------------
    stream_case(
        m,
        "4.3_BASELINE_NORMAL",
        3.0,
        status_kwargs=dict(conf=0.90, lost=0, reproj=0.05, cov=0.02),
        corr_kwargs=dict(conf=0.90, flags=3, x=1.0, y=2.0, z=3.0, distance=2.50, pitch=-0.02),
    )

    # ------------------------------------------------------------
    # CASE 2：Step input
    # 目的：突然把 correction 從小值跳到大值。
    # 預期：系統不崩潰，有 AI_CORR / QLAND AI / AI_CTRL_USE。
    # ------------------------------------------------------------
    stream_case(
        m,
        "4.3_STEP_SMALL_ERROR",
        2.0,
        status_kwargs=dict(conf=0.95, lost=0, reproj=0.03, cov=0.01),
        corr_kwargs=dict(conf=0.95, flags=3, x=0.2, y=0.3, z=0.5, distance=1.20, pitch=-0.01),
    )

    stream_case(
        m,
        "4.3_STEP_LARGE_ERROR",
        3.0,
        status_kwargs=dict(conf=0.95, lost=0, reproj=0.03, cov=0.01),
        corr_kwargs=dict(conf=0.95, flags=3, x=8.0, y=-8.0, z=6.0, distance=9.00, pitch=-0.20),
    )

    # ------------------------------------------------------------
    # CASE 3：Transition valid -> invalid flags -> valid
    # 目的：驗證 invalid correction flags 會被偵測，且後續 valid 可恢復。
    # 預期：AI_LANDING_CORR_INVALID，之後恢復 AI_CTRL_USE / QLAND AI ok=1。
    # ------------------------------------------------------------
    stream_case(
        m,
        "4.3_TRANSITION_VALID_BEFORE_INVALID",
        2.0,
        status_kwargs=dict(conf=0.90, lost=0, reproj=0.05, cov=0.02),
        corr_kwargs=dict(conf=0.90, flags=3, x=1.0, y=2.0, z=3.0, distance=2.50, pitch=-0.02),
    )

    stream_case(
        m,
        "4.3_TRANSITION_INVALID_FLAGS",
        3.0,
        status_kwargs=dict(conf=0.90, lost=0, reproj=0.05, cov=0.02),
        corr_kwargs=dict(conf=0.90, flags=0, x=1.0, y=2.0, z=3.0, distance=2.50, pitch=-0.02),
    )

    stream_case(
        m,
        "4.3_TRANSITION_RECOVER_VALID",
        3.0,
        status_kwargs=dict(conf=0.90, lost=0, reproj=0.05, cov=0.02),
        corr_kwargs=dict(conf=0.90, flags=3, x=1.5, y=2.5, z=3.5, distance=2.80, pitch=-0.02),
    )

    # ------------------------------------------------------------
    # CASE 4：Boundary low confidence / lost / high reproj / high covariance
    # 目的：驗證極端 status 會觸發 L2 / fallback 類流程，不會崩潰。
    # 注意：這可能會觸發 L2 Go-Around，屬於合理結果。
    # ------------------------------------------------------------
    stream_case(
        m,
        "4.3_BOUNDARY_LOW_CONFIDENCE",
        2.0,
        status_kwargs=dict(conf=0.10, lost=0, reproj=0.05, cov=0.02),
        corr_kwargs=dict(conf=0.10, flags=3, x=1.0, y=2.0, z=3.0, distance=2.50, pitch=-0.02),
    )

    stream_case(
        m,
        "4.3_BOUNDARY_LOST_HIGH_REPROJ_COV",
        4.0,
        status_kwargs=dict(conf=0.40, lost=1, reproj=0.35, cov=0.25),
        corr_kwargs=dict(conf=0.40, flags=3, x=2.0, y=3.0, z=4.0, distance=3.50, pitch=-0.05),
    )

    # ------------------------------------------------------------
    # CASE 5：No AI message timeout
    # 目的：完全停止 AI 訊息，驗證 timeout / fallback。
    # ------------------------------------------------------------
    pause_no_ai(1.8)

    # ------------------------------------------------------------
    # CASE 6：Recovery after boundary / timeout
    # 目的：恢復正常資料，驗證系統仍可接收正常資料。
    # ------------------------------------------------------------
    stream_case(
        m,
        "4.3_RECOVER_AFTER_BOUNDARY_TIMEOUT",
        4.0,
        status_kwargs=dict(conf=0.90, lost=0, reproj=0.05, cov=0.02),
        corr_kwargs=dict(conf=0.90, flags=3, x=1.0, y=2.0, z=3.0, distance=2.50, pitch=-0.02),
    )

    print("\n[DONE] Story 4.3 boundary/stability injection completed.")
    print("[NEXT] grep story43_ai_landing_boundary_stability.log for proof lines.")

if __name__ == "__main__":
    main()
