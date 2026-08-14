#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import threading
from pymavlink import mavutil

HOME = os.path.expanduser("~")
LOCAL_MAVLINK = os.path.join(HOME, "ardupilot", "modules", "mavlink")
if os.path.isdir(LOCAL_MAVLINK) and LOCAL_MAVLINK not in sys.path:
    sys.path.insert(0, LOCAL_MAVLINK)

os.environ.setdefault("MAVLINK20", "1")
os.environ.setdefault("MAVLINK_DIALECT", "ardupilotmega")

AI_SYS_ID = 42
AI_COMP_ID = 211
FRAME_CAMERA = 200

texts = []
stop_reader = False


def now_ms():
    return int(time.time() * 1000) & 0xFFFFFFFF


def send_heartbeat(conn):
    conn.mav.heartbeat_send(
        getattr(mavutil.mavlink, "MAV_TYPE_ONBOARD_CONTROLLER", 18),
        getattr(mavutil.mavlink, "MAV_AUTOPILOT_INVALID", 8),
        0,
        0,
        0,
    )


def reader(conn):
    global stop_reader
    last_hb = 0

    while not stop_reader:
        if time.time() - last_hb > 1:
            try:
                send_heartbeat(conn)
            except Exception:
                pass
            last_hb = time.time()

        msg = conn.recv_match(blocking=True, timeout=0.2)
        if msg is None:
            continue

        if msg.get_type() == "STATUSTEXT":
            text = msg.text
            if isinstance(text, bytes):
                text = text.decode(errors="ignore")
            text = str(text).strip("\x00")
            texts.append(text)
            print("[STATUSTEXT]", text, flush=True)

        elif msg.get_type() == "COMMAND_LONG":
            cmd = int(getattr(msg, "command", -1))
            print("[COMMAND_LONG]", cmd, flush=True)

            if cmd in (31010, 31011):
                try:
                    conn.mav.command_ack_send(cmd, 0, 0, 0, 1, 1)
                except TypeError:
                    conn.mav.command_ack_send(cmd, 0)


def send_status(conn, conf, lost, reproj, cov):
    conn.mav.ai_landing_status_send(
        now_ms(),
        float(conf),
        int(lost),
        float(reproj),
        float(cov),
    )
    print(f"[SEND] STATUS conf={conf:.2f} lost={lost} reproj={reproj:.2f} cov={cov:.2f}", flush=True)


def send_corr(conn, conf, pitch=-0.04):
    conn.mav.ai_landing_correction_send(
        now_ms(),
        0.0,
        float(pitch),
        0.01,
        0.0,
        0.0,
        1.2,
        1.2,
        float(conf),
        int(FRAME_CAMERA),
        int(3),
    )
    print(f"[SEND] CORR conf={conf:.2f} pitch={pitch:.3f}", flush=True)


def send_l2_burst(conn, seconds, conf=0.40):
    end = time.time() + seconds
    while time.time() < end:
        send_status(conn, conf=conf, lost=1, reproj=0.40, cov=0.30)
        send_corr(conn, conf=conf, pitch=-0.04)
        time.sleep(0.2)


def seen_l3():
    all_text = "\n".join(texts)
    return (
        "AI_LANDING_TAKEOVER" in all_text
        or "L3=1" in all_text
        or "TAKEOVER" in all_text
    )


def main():
    global stop_reader

    print("[CONNECT] tcp:127.0.0.1:5762")
    conn = mavutil.mavlink_connection(
        "tcp:127.0.0.1:5762",
        source_system=AI_SYS_ID,
        source_component=AI_COMP_ID,
        autoreconnect=True,
        robust_parsing=True,
    )

    send_heartbeat(conn)

    t = threading.Thread(target=reader, args=(conn,), daemon=True)
    t.start()

    print()
    print("=" * 72)
    print("請確認 MAVProxy 已經在 QLAND。")
    print("如果還沒，請在 MAVProxy 輸入：")
    print("  mode QHOVER")
    print("  arm throttle force")
    print("  rc 3 1700")
    print("  等 2~3 秒")
    print("  mode QLAND")
    print("=" * 72)
    input("確認 QLAND 後按 Enter 開始測 L3...")

    print()
    print("[PHASE 0] normal baseline 2 秒")
    for _ in range(10):
        send_status(conn, conf=0.90, lost=0, reproj=0.05, cov=0.02)
        send_corr(conn, conf=0.90, pitch=-0.02)
        time.sleep(0.2)

    print()
    print("[PHASE 1] 第一次 L2：送 4 秒")
    send_l2_burst(conn, seconds=4, conf=0.40)

    print()
    print("=" * 72)
    print("第一次 L2 後，飛控可能已切到 QLOITER。")
    print("請看 MAVProxy prompt。")
    print("如果不是 QLAND，請重新輸入：")
    print("  mode QHOVER")
    print("  arm throttle force")
    print("  rc 3 1700")
    print("  等 2 秒")
    print("  mode QLAND")
    print("=" * 72)
    input("重新進 QLAND 後按 Enter，繼續累積 L3...")

    print()
    print("[PHASE 2] 重複 L2，最多 30 秒，直到看到 L3=1 或 TAKEOVER")

    start = time.time()
    round_no = 1

    while time.time() - start < 30:
        print()
        print(f"[L2 ROUND {round_no}] 送 L2 3 秒")
        send_l2_burst(conn, seconds=3, conf=0.40)

        if seen_l3():
            print()
            print("=" * 72)
            print("[PASS] L3 成功：看到 AI_LANDING_TAKEOVER 或 L3=1")
            print("=" * 72)
            stop_reader = True
            return

        print("[INFO] 尚未看到 L3。若已切出 QLAND，請在 MAVProxy 重新進 QLAND。")
        print("      mode QHOVER -> arm throttle force -> rc 3 1700 -> mode QLAND")
        round_no += 1
        time.sleep(1)

    print()
    print("=" * 72)
    print("[WARN] 30 秒內沒有看到 L3")
    print("請檢查 MAVProxy 是否出現：")
    print("  AI_STAT ... L3=1")
    print("  AI_LANDING_TAKEOVER reason=5")
    print("如果一直只有 L2=1 L3=0，代表 R 還沒到門檻，或每次 L2 後模式切走導致沒有累積。")
    print("=" * 72)

    stop_reader = True


if __name__ == "__main__":
    main()
