#!/usr/bin/env python3
import os
import sys
import time
from pymavlink import mavutil

os.environ["MAVLINK20"] = "1"
os.environ["MAVLINK_DIALECT"] = "ardupilotmega"

AI_SYSID = 1
AI_COMPID = 211

CMD_START = 31010
CMD_STOP = 31011

EXPECTED_START_STREAM = 1.0
EXPECTED_START_HZ = 10.0
EXPECTED_START_FRAME = 12.0   # MAV_FRAME_BODY_FRD

def send_ai_heartbeat(m):
    m.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        mavutil.mavlink.MAV_STATE_ACTIVE
    )

def approx_equal(a, b, eps=1e-4):
    return abs(float(a) - float(b)) <= eps

def validate_start(msg):
    ok = True
    errors = []

    if int(msg.target_component) != AI_COMPID:
        ok = False
        errors.append(f"target_component expected {AI_COMPID}, got {msg.target_component}")

    if int(msg.command) != CMD_START:
        ok = False
        errors.append(f"command expected {CMD_START}, got {msg.command}")

    if not approx_equal(msg.param1, EXPECTED_START_STREAM):
        ok = False
        errors.append(f"param1(stream) expected {EXPECTED_START_STREAM}, got {msg.param1}")

    if not approx_equal(msg.param2, EXPECTED_START_HZ):
        ok = False
        errors.append(f"param2(freq_hz) expected {EXPECTED_START_HZ}, got {msg.param2}")

    if not approx_equal(msg.param3, EXPECTED_START_FRAME):
        ok = False
        errors.append(f"param3(frame) expected {EXPECTED_START_FRAME}, got {msg.param3}")

    return ok, errors

def validate_stop(msg):
    ok = True
    errors = []

    if int(msg.target_component) != AI_COMPID:
        ok = False
        errors.append(f"target_component expected {AI_COMPID}, got {msg.target_component}")

    if int(msg.command) != CMD_STOP:
        ok = False
        errors.append(f"command expected {CMD_STOP}, got {msg.command}")

    return ok, errors

def ack(m, msg, result, result_param2=0):
    m.mav.command_ack_send(
        int(msg.command),
        result,
        0,
        result_param2,
        msg.get_srcSystem(),
        msg.get_srcComponent()
    )

def print_msg(msg):
    ts = time.strftime("%H:%M:%S")
    print(f"\n[{ts}] COMMAND_LONG received")
    print(f"  from sys={msg.get_srcSystem()} comp={msg.get_srcComponent()}")
    print(f"  target_system={msg.target_system} target_component={msg.target_component}")
    print(f"  command={msg.command}")
    print(f"  p1={msg.param1} p2={msg.param2} p3={msg.param3} p4={msg.param4} p5={msg.param5} p6={msg.param6} p7={msg.param7}")

def main():
    print("Connecting as AI mock on tcp:127.0.0.1:5762 ...")
    m = mavutil.mavlink_connection(
        "tcp:127.0.0.1:5762",
        source_system=AI_SYSID,
        source_component=AI_COMPID,
        dialect="ardupilotmega",
    )

    print("Waiting heartbeat...")
    hb = m.wait_heartbeat(timeout=30)
    if hb is None:
        raise RuntimeError("No heartbeat received from FC")

    print(f"Heartbeat OK: target sys={m.target_system} comp={m.target_component}")

    print("Sending AI heartbeat to register routing...")
    for i in range(5):
        send_ai_heartbeat(m)
        print(f"  AI heartbeat {i+1}/5 sent")
        time.sleep(0.5)

    print("AI mock strict validator ready. Waiting COMMAND_LONG ...")

    last_hb = time.time()
    start_count = 0
    stop_count = 0

    while True:
        now = time.time()
        if now - last_hb >= 1.0:
            send_ai_heartbeat(m)
            last_hb = now

        msg = m.recv_match(type=["COMMAND_LONG"], blocking=True, timeout=1)
        if msg is None:
            continue

        print_msg(msg)

        if int(msg.target_component) != AI_COMPID:
            print("  -> ignored: not for component 211")
            continue

        cmd = int(msg.command)

        if cmd == CMD_START:
            start_count += 1
            ok, errors = validate_start(msg)

            if ok:
                print("  -> START VALIDATION PASSED")
                ack(m, msg, mavutil.mavlink.MAV_RESULT_ACCEPTED, 0)
                print("  -> ACK sent: ACCEPTED")
            else:
                print("  -> START VALIDATION FAILED")
                for err in errors:
                    print(f"     - {err}")
                ack(m, msg, mavutil.mavlink.MAV_RESULT_DENIED, 9001)
                print("  -> ACK sent: DENIED result_param2=9001")

        elif cmd == CMD_STOP:
            stop_count += 1
            ok, errors = validate_stop(msg)

            if ok:
                print("  -> STOP VALIDATION PASSED")
                ack(m, msg, mavutil.mavlink.MAV_RESULT_ACCEPTED, 0)
                print("  -> ACK sent: ACCEPTED")
            else:
                print("  -> STOP VALIDATION FAILED")
                for err in errors:
                    print(f"     - {err}")
                ack(m, msg, mavutil.mavlink.MAV_RESULT_DENIED, 9002)
                print("  -> ACK sent: DENIED result_param2=9002")

        else:
            print("  -> UNKNOWN COMMAND for AI landing strict validator")
            ack(m, msg, mavutil.mavlink.MAV_RESULT_UNSUPPORTED, 9003)
            print("  -> ACK sent: UNSUPPORTED result_param2=9003")

        print(f"  -> Counters: START={start_count}, STOP={stop_count}")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped by user.")
        sys.exit(0)