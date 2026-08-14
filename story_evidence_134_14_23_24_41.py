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
        time.sleep(0.3)
    return m

def send_status(m, conf=0.90, lost=0, reproj=0.05, cov=0.02):
    m.mav.ai_landing_status_send(
        now_ms(),
        float(conf),
        int(lost),
        float(reproj),
        float(cov)
    )

def send_corr(m, conf=0.90, flags=3, x=1.0, y=2.0, z=3.0, distance=2.50):
    # flags=3：預期代表 yaw / distance 有效。
    # flags=0：用來測 AI_LANDING_CORR_INVALID。
    m.mav.ai_landing_correction_send(
        now_ms(),
        0.0,          # roll_err
        -0.02,        # pitch_err
        0.0,          # yaw_err
        float(x),     # x_err
        float(y),     # y_err
        float(z),     # z_err
        float(distance),
        float(conf),
        mavutil.mavlink.MAV_FRAME_BODY_FRD,
        int(flags)
    )

def stream_valid(m, name, seconds=3.0, conf=0.90):
    print(f"\n[CASE] {name}: valid status + valid correction, {seconds}s")
    end = time.time() + seconds
    while time.time() < end:
        send_status(m, conf=conf, lost=0, reproj=0.05, cov=0.02)
        send_corr(m, conf=conf, flags=3)
        time.sleep(0.1)

def stream_invalid_corr_flags(m, seconds=2.0):
    print(f"\n[CASE] INVALID_CORR_FLAGS: flags=0, {seconds}s")
    end = time.time() + seconds
    while time.time() < end:
        send_status(m, conf=0.90, lost=0, reproj=0.05, cov=0.02)
        send_corr(m, conf=0.90, flags=0)
        time.sleep(0.1)

def pause_no_ai(seconds):
    print(f"\n[CASE] PAUSE_NO_AI: stop all AI messages for {seconds}s")
    time.sleep(seconds)

def stream_l2_go_around(m, seconds=4.0):
    print(f"\n[CASE] L2_GO_AROUND: conf=0.40 lost=1 reproj=0.35 cov=0.25, {seconds}s")
    end = time.time() + seconds
    while time.time() < end:
        send_status(m, conf=0.40, lost=1, reproj=0.35, cov=0.25)
        send_corr(m, conf=0.40, flags=3)
        time.sleep(0.1)

def main():
    m = connect()

    # Story 4.1 + Story 1.3 + Story 1.4 baseline：
    # SITL 注入 correction/status，飛控應該解析並更新 AI_STAT / AI_CORR / QLAND AI / AI_CTRL_USE。
    stream_valid(m, "S4.1_BASELINE_VALID_INJECTION", seconds=4)

    # Story 1.3 flags 無效：
    # correction flags=0，飛控應該輸出 AI_LANDING_CORR_INVALID。
    stream_invalid_corr_flags(m, seconds=3)

    # 恢復正常：
    # 驗證 invalid 後系統可恢復正常資料流。
    stream_valid(m, "RECOVER_AFTER_INVALID_FLAGS", seconds=3)

    # Story 2.3：
    # L2 條件必須在 QLAND 流程仍有效時先測。
    # 若先做 timeout，模式可能已切離 QLAND，導致只能看到 L2=1，
    # 但看不到 AI_LANDING_ABORT_L2 / AI_GO_AROUND。
    stream_l2_go_around(m, seconds=5)

    # Story 1.4 / 2.4 timeout：
    # 停止所有 AI_LANDING_STATUS / CORRECTION 超過 1 秒。
    # 期待 AI_LINK_TIMEOUT / AI_CORR_FALLBACK / Data unhealthy / lost。
    pause_no_ai(1.8)

    # Story 2.4 recovery：
    # timeout 後重新送正常資料，期待 lost=0 err=0 或 QLAND AI ok=1 恢復。
    stream_valid(m, "RECOVER_AFTER_TIMEOUT", seconds=4)

    print("\n[DONE] Evidence injection completed.")
    print("[NEXT] grep evidence_134_14_23_24_41.log for proof lines.")

if __name__ == "__main__":
    main()
