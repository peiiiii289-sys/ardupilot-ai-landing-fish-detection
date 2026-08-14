#!/usr/bin/env python3
import argparse
import os
import time
from pymavlink import mavutil

os.environ["MAVLINK20"] = "1"
os.environ["MAVLINK_DIALECT"] = "ardupilotmega"

MASTER = "tcp:127.0.0.1:5762"

# AI companion component id
AI_COMPID = 211

# 3.2 使用 common.xml 既有 USER command
CMD_START = 31012  # MAV_CMD_USER_3
CMD_STOP  = 31013  # MAV_CMD_USER_4

def send_ai_heartbeat(m):
    for i in range(5):
        m.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            mavutil.mavlink.MAV_STATE_ACTIVE
        )
        print(f"[AI_HEARTBEAT] {i+1}/5")
        time.sleep(0.2)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--result", type=int, default=0,
                        help="COMMAND_ACK result, 0=MAV_RESULT_ACCEPTED")
    parser.add_argument("--param2", type=int, default=0,
                        help="COMMAND_ACK result_param2")
    parser.add_argument("--no-ack", action="store_true",
                        help="Receive START/STOP but intentionally do not reply ACK")
    parser.add_argument("--count", type=int, default=1,
                        help="How many START/STOP commands to handle")
    args = parser.parse_args()

    print(f"[CONNECT] {MASTER} as sys=1 comp={AI_COMPID}")

    m = mavutil.mavlink_connection(
        MASTER,
        source_system=1,
        source_component=AI_COMPID,
        autoreconnect=True,
        dialect="ardupilotmega",
    )

    print("[WAIT] heartbeat from flight controller...")
    m.wait_heartbeat()
    print(f"[OK] heartbeat from sys={m.target_system} comp={m.target_component}")

    print("[ROUTE] send AI heartbeat so FC knows component 211 exists")
    send_ai_heartbeat(m)

    print(f"[WAIT] COMMAND_LONG 31012/31013, count={args.count}, no_ack={args.no_ack}")

    handled = 0
    deadline = time.time() + 20.0

    while handled < args.count and time.time() < deadline:
        msg = m.recv_match(type="COMMAND_LONG", blocking=True, timeout=1)

        if msg is None:
            continue

        cmd = int(msg.command)

        print(
            f"[RX] COMMAND_LONG command={cmd} "
            f"target_system={msg.target_system} target_component={msg.target_component} "
            f"param1={msg.param1} param2={msg.param2} param3={msg.param3}"
        )

        if cmd not in (CMD_START, CMD_STOP):
            print(f"[SKIP] Not AI vision START/STOP command: {cmd}")
            continue

        handled += 1

        if args.no_ack:
            print(f"[NO_ACK] command={cmd}, intentionally not replying")
            continue

        print(f"[SEND_ACK] command={cmd} result={args.result} param2={args.param2}")

        # COMMAND_ACK fields:
        # command, result, progress, result_param2, target_system, target_component
        m.mav.command_ack_send(
            cmd,
            args.result,
            0,
            args.param2,
            1,
            1
        )

    print(f"[DONE] handled={handled}")

if __name__ == "__main__":
    main()
