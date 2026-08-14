#!/usr/bin/env python3
import argparse
import os
import random
import time

from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink

os.environ["MAVLINK20"] = "1"
os.environ["MAVLINK_DIALECT"] = "ardupilotmega"

MASTER = "tcp:127.0.0.1:5762"

AI_SYSID = 1
AI_COMPID = 211

CMD_START = 31012  # MAV_CMD_USER_3
CMD_STOP  = 31013  # MAV_CMD_USER_4

def now_ms():
    return int(time.monotonic() * 1000) & 0xFFFFFFFF

def connect():
    print(f"[CONNECT] {MASTER} as sys={AI_SYSID} comp={AI_COMPID}")

    m = mavutil.mavlink_connection(
        MASTER,
        source_system=AI_SYSID,
        source_component=AI_COMPID,
        autoreconnect=True,
        dialect="ardupilotmega",
    )

    print("[WAIT] heartbeat from flight controller...")
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
        0.0,
        float(pitch),
        0.0,
        float(x),
        float(y),
        float(z),
        float(distance),
        float(conf),
        mavutil.mavlink.MAV_FRAME_BODY_FRD,
        int(flags)
    )

def send_fish_result(m, compid_label="211-valid"):
    cls = mavlink.MAVLink_ai_fish_detection_result_message
    fields = list(cls.fieldnames)

    values = {
        "time_boot_ms": now_ms(),
        "image_width": 640,
        "image_height": 480,
        "fish_coverage_pct": 45.0,
        "fish_count": 32,
        "tuna_similarity_pct": 88.5,
        "bird_count": 3,
        "inference_fps": 12.5,
    }

    args = [values.get(name, 0) for name in fields]
    msg = cls(*args)
    m.mav.send(msg)

    print(
        "[SEND_FISH] AI_FISH_DETECTION_RESULT "
        f"src={compid_label} fish=32 cov=45.0 tuna=88.5 bird=3 fps=12.5 size=640x480"
    )

def handle_command_long_ack_random(m, rng, no_ack_chance=0.20, fail_ack_chance=0.20):
    """
    3.2 路徑：
    如果飛控轉送 COMMAND_LONG 31012/31013 給 AI companion，
    此函式會隨機回 ACCEPTED、FAILED+param2，或故意不回 ACK。
    """
    handled = 0

    while True:
        msg = m.recv_match(type="COMMAND_LONG", blocking=False)
        if msg is None:
            break

        cmd = int(msg.command)
        print(
            f"[RX_CMD] COMMAND_LONG command={cmd} "
            f"target_system={msg.target_system} target_component={msg.target_component} "
            f"param1={msg.param1} param2={msg.param2} param3={msg.param3}"
        )

        if cmd not in (CMD_START, CMD_STOP):
            print(f"[RX_CMD_SKIP] command={cmd}")
            continue

        handled += 1

        r = rng.random()

        if r < no_ack_chance:
            print(f"[ACK_RANDOM_NO_ACK] command={cmd}")
            continue

        if r < no_ack_chance + fail_ack_chance:
            result = 4
            param2 = 1002
            print(f"[ACK_RANDOM_FAIL] command={cmd} result={result} param2={param2}")
        else:
            result = 0
            param2 = 0
            print(f"[ACK_RANDOM_ACCEPTED] command={cmd} result={result} param2={param2}")

        m.mav.command_ack_send(
            cmd,
            result,
            0,
            param2,
            1,
            1
        )

    return handled

def event_valid_normal(m):
    print("[EVENT] VALID_NORMAL")
    for _ in range(10):
        send_status(m, conf=0.90, lost=0, reproj=0.05, cov=0.02)
        send_corr(m, conf=0.90, flags=3, x=1.0, y=2.0, z=3.0, distance=2.50, pitch=-0.02)
        time.sleep(0.1)

def event_step_small(m):
    print("[EVENT] STEP_SMALL")
    for _ in range(10):
        send_status(m, conf=0.95, lost=0, reproj=0.03, cov=0.01)
        send_corr(m, conf=0.95, flags=3, x=0.2, y=0.3, z=0.5, distance=1.20, pitch=-0.01)
        time.sleep(0.1)

def event_step_large(m):
    print("[EVENT] STEP_LARGE")
    for _ in range(10):
        send_status(m, conf=0.95, lost=0, reproj=0.03, cov=0.01)
        send_corr(m, conf=0.95, flags=3, x=8.0, y=-8.0, z=6.0, distance=9.00, pitch=-0.20)
        time.sleep(0.1)

def event_invalid_flags(m):
    print("[EVENT] INVALID_FLAGS")
    for _ in range(10):
        send_status(m, conf=0.90, lost=0, reproj=0.05, cov=0.02)
        send_corr(m, conf=0.90, flags=0, x=1.0, y=2.0, z=3.0, distance=2.50, pitch=-0.02)
        time.sleep(0.1)

def event_l1_mild_conf(m):
    print("[EVENT] L1_MILD_CONF")
    # Known L1 trigger from story42_l123.log:
    # conf=0.69 lost=0 reproj=0.05 cov=0.02 => L1=1 L2=0 L3=0 reason=1
    for _ in range(10):
        send_status(m, conf=0.69, lost=0, reproj=0.05, cov=0.02)
        send_corr(m, conf=0.69, flags=3, x=1.0, y=2.0, z=3.0, distance=2.50, pitch=-0.02)
        time.sleep(0.1)

def event_l2_low_conf(m):
    print("[EVENT] BOUNDARY_LOW_CONF_L2")
    for _ in range(10):
        send_status(m, conf=0.10, lost=0, reproj=0.05, cov=0.02)
        send_corr(m, conf=0.10, flags=3, x=1.0, y=2.0, z=3.0, distance=2.50, pitch=-0.02)
        time.sleep(0.1)

def event_l2_lost_high(m):
    print("[EVENT] BOUNDARY_LOST_HIGH_REPROJ_COV")
    for _ in range(10):
        send_status(m, conf=0.40, lost=1, reproj=0.35, cov=0.25)
        send_corr(m, conf=0.40, flags=3, x=2.0, y=3.0, z=4.0, distance=3.50, pitch=-0.05)
        time.sleep(0.1)

def event_timeout_pause():
    print("[EVENT] PAUSE_NO_AI_TIMEOUT")
    time.sleep(1.8)

def event_recover_valid(m):
    print("[EVENT] RECOVER_VALID")
    for _ in range(12):
        send_status(m, conf=0.90, lost=0, reproj=0.05, cov=0.02)
        send_corr(m, conf=0.90, flags=3, x=1.5, y=2.5, z=3.5, distance=2.80, pitch=-0.02)
        time.sleep(0.1)

def event_fish_valid(m):
    print("[EVENT] FISH_VALID_COMP211")
    for _ in range(3):
        send_fish_result(m, compid_label="211-valid")
        time.sleep(0.2)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--events", type=int, default=30)
    parser.add_argument("--include-timeout", action="store_true")
    args = parser.parse_args()

    if args.seed is None:
        rng = random.Random()
        seed_label = "system-random"
    else:
        rng = random.Random(args.seed)
        seed_label = str(args.seed)

    print(f"[CONFIG] seed={seed_label} events={args.events} include_timeout={args.include_timeout}")

    m = connect()

    events = [
        event_valid_normal,
        event_step_small,
        event_step_large,
        event_invalid_flags,
        event_l1_mild_conf,
        event_l2_low_conf,
        event_l2_lost_high,
        event_recover_valid,
        event_fish_valid,
    ]

    if args.include_timeout:
        events.append(lambda _m: event_timeout_pause())

    # 先建立 baseline，避免一開始就極端狀態
    event_valid_normal(m)

    for i in range(args.events):
        handle_command_long_ack_random(m, rng)

        ev = rng.choice(events)
        print(f"\n[RANDOM_EVENT] {i+1}/{args.events} name={ev.__name__ if hasattr(ev, '__name__') else 'timeout_lambda'}")
        ev(m)

        handle_command_long_ack_random(m, rng)

    # 最後恢復正常資料，方便證明 recovery
    event_recover_valid(m)

    # 最後再收一次可能的 31012 / 31013
    end = time.time() + 2.0
    while time.time() < end:
        handle_command_long_ack_random(m, rng)
        time.sleep(0.1)

    print("\n[DONE] all-in-one random integration stress test completed.")
    print("[NEXT] grep story_all_random_integration_stress.log for proof lines.")

if __name__ == "__main__":
    main()
