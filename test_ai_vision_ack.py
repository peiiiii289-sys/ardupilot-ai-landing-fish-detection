import argparse
import os
import sys
import time

sys.path.insert(0, os.path.expanduser("~/ardupilot/modules/mavlink"))
os.environ["MAVLINK20"] = "1"
os.environ["MAVLINK_DIALECT"] = "ardupilotmega"

from pymavlink import mavutil

AI_SYS_ID = 42
AI_COMP_ID = 211

parser = argparse.ArgumentParser()
parser.add_argument("--result", type=int, default=0, help="MAV_RESULT value, default=0 (ACCEPTED)")
parser.add_argument("--param2", type=int, default=0, help="COMMAND_ACK result_param2")
parser.add_argument("--no-ack", action="store_true", help="Do not send ACK")
args = parser.parse_args()

m = mavutil.mavlink_connection(
    "tcp:127.0.0.1:5762",
    source_system=AI_SYS_ID,
    source_component=AI_COMP_ID
)

print("Connecting AI mock on tcp:127.0.0.1:5762 ...")
print("Waiting heartbeat from FC...")
m.wait_heartbeat()
print("Heartbeat OK: FC discovered")

print("Sending AI heartbeats to register routing...")
for i in range(5):
    m.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0, 0, 0
    )
    print(f"AI heartbeat {i+1}/5 sent")
    time.sleep(0.5)

last_hb = time.time()

while True:
    now = time.time()
    if now - last_hb >= 1.0:
        m.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0
        )
        last_hb = now

    msg = m.recv_match(blocking=False)
    if msg is None:
        time.sleep(0.05)
        continue

    if msg.get_type() != "COMMAND_LONG":
        continue

    print(f"RX COMMAND_LONG cmd={msg.command} p1={msg.param1} p2={msg.param2} p3={msg.param3}")

    if msg.command not in (31012, 31013):
        continue

    if args.no_ack:
        print(f"NO_ACK mode: skip ACK for cmd={msg.command}")
        continue

    m.mav.command_ack_send(
        msg.command,
        args.result,
        0,
        args.param2,
        msg.get_srcSystem(),
        msg.get_srcComponent()
    )
    print(f"ACK sent cmd={msg.command} result={args.result} param2={args.param2}")