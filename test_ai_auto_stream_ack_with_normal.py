#!/usr/bin/env python3
import argparse
import time
import threading
from pymavlink import mavutil

START_CMD = 31010
STOP_CMD = 31011

def send_ai_heartbeat(m):
    m.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        mavutil.mavlink.MAV_STATE_ACTIVE
    )

def now_ms():
    return int(time.monotonic() * 1000) & 0xFFFFFFFF

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--master", default="tcp:127.0.0.1:5762")
    parser.add_argument("--hz", type=float, default=10.0)
    args = parser.parse_args()

    print(f"[CONNECT] {args.master}")
    mav = mavutil.mavlink_connection(
        args.master,
        source_system=1,
        source_component=211,
        autoreconnect=True,
        dialect="ardupilotmega",
    )

    print("[WAIT] heartbeat...")
    mav.wait_heartbeat(timeout=30)
    print(f"[OK] heartbeat from sys={mav.target_system} comp={mav.target_component}")

    print("[ROUTE] sending AI heartbeat to register routing...")
    for i in range(5):
        send_ai_heartbeat(mav)
        print(f"  AI heartbeat {i+1}/5 sent")
        time.sleep(0.5)

    counts = {"START": 0, "STOP": 0}
    running = True

    def send_normal_loop():
        period = 1.0 / args.hz
        while running:
            try:
                t = now_ms()

                mav.mav.ai_landing_status_send(
                    t,
                    0.90,
                    0,
                    0.05,
                    0.02
                )

                mav.mav.ai_landing_correction_send(
                    t,
                    0.0,
                    -0.02,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    2.5,
                    0.90,
                    12,
                    3
                )
            except Exception as e:
                print(f"[STREAM_ERR] {e}")
            time.sleep(period)

    th = threading.Thread(target=send_normal_loop, daemon=True)
    th.start()

    print("[RUN] sending normal AI stream and waiting START/STOP commands...")
    print("[GOAL] need START=1 and STOP=1")

    try:
        while True:
            msg = mav.recv_match(blocking=True, timeout=1)
            if msg is None:
                continue

            if msg.get_type() != "COMMAND_LONG":
                continue

            cmd = int(msg.command)
            if cmd not in (START_CMD, STOP_CMD):
                continue

            name = "START" if cmd == START_CMD else "STOP"
            counts[name] += 1

            print(f"\n[COMMAND_LONG] {name} command={cmd}")
            print(
                f"  from sys={msg.get_srcSystem()} comp={msg.get_srcComponent()} "
                f"target_system={msg.target_system} target_component={msg.target_component}"
            )
            print(
                f"  p1={msg.param1} p2={msg.param2} p3={msg.param3} "
                f"p4={msg.param4} p5={msg.param5} p6={msg.param6} p7={msg.param7}"
            )

            mav.mav.command_ack_send(
                cmd,
                mavutil.mavlink.MAV_RESULT_ACCEPTED,
                0,
                0,
                msg.get_srcSystem(),
                msg.get_srcComponent()
            )

            print("  -> ACK sent: ACCEPTED")
            print(f"  -> Counters: START={counts['START']}, STOP={counts['STOP']}")

            if counts["START"] >= 1 and counts["STOP"] >= 1:
                print("\nPASS: Story 2.5 START/STOP ACK completed")
                break
    finally:
        running = False
        time.sleep(0.2)

if __name__ == "__main__":
    main()
