#!/usr/bin/env python3
import argparse
import os
import sys
import time

os.environ["MAVLINK20"] = "1"
os.environ["MAVLINK_DIALECT"] = "ardupilotmega"

from pymavlink import mavutil


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Story 2.4 unified AI timeout/recovery test sender")
    p.add_argument("--master", default="tcp:127.0.0.1:5762")
    p.add_argument("--hz", type=float, default=10.0)
    p.add_argument("--drop", choices=["none", "corr", "status", "both"], default="none")
    p.add_argument("--drop-after", type=float, default=-1.0)
    p.add_argument("--recover-after", type=float, default=-1.0)
    p.add_argument("--duration", type=float, default=30.0)
    p.add_argument("--corr-conf", type=float, default=0.85)
    p.add_argument("--status-conf", type=float, default=0.85)
    p.add_argument("--pitch", type=float, default=-0.02)
    p.add_argument("--roll", type=float, default=0.0)
    p.add_argument("--yaw", type=float, default=0.0)
    p.add_argument("--x", type=float, default=0.0)
    p.add_argument("--y", type=float, default=0.0)
    p.add_argument("--z", type=float, default=0.0)
    p.add_argument("--distance", type=float, default=1.23)
    p.add_argument("--target-lost", type=int, choices=[0, 1], default=0)
    p.add_argument("--reproj", type=float, default=0.12)
    p.add_argument("--cov", type=float, default=0.03)
    p.add_argument("--frame", type=int, default=20)
    p.add_argument("--flags", type=int, default=3)
    return p.parse_args()


def should_send(kind: str, elapsed: float, drop: str, drop_after: float, recover_after: float) -> bool:
    dropped = False

    if drop_after >= 0 and elapsed >= drop_after:
        if drop == "both":
            dropped = True
        elif drop == kind:
            dropped = True

    if dropped and recover_after >= 0 and elapsed >= recover_after:
        dropped = False

    return not dropped


def main() -> int:
    args = parse_args()

    if args.hz <= 0:
        print("hz must be > 0", file=sys.stderr)
        return 1

    period = 1.0 / args.hz

    print(f"Connecting to {args.master}")
    mav = mavutil.mavlink_connection(
        args.master,
        source_system=1,
        source_component=211,
    )

    start_wall = time.monotonic()
    seq = 0

    print(
        f"START duration={args.duration}s hz={args.hz} "
        f"drop={args.drop} drop_after={args.drop_after} recover_after={args.recover_after}"
    )

    try:
        while True:
            now_wall = time.monotonic()
            elapsed = now_wall - start_wall

            if elapsed > args.duration:
                print("DONE")
                break

            time_boot_ms = int(elapsed * 1000.0)

            send_corr = should_send("corr", elapsed, args.drop, args.drop_after, args.recover_after)
            send_status = should_send("status", elapsed, args.drop, args.drop_after, args.recover_after)

            if send_corr:
                corr_msg = mavutil.mavlink.MAVLink_ai_landing_correction_message(
                    time_boot_ms,
                    float(args.roll),
                    float(args.pitch),
                    float(args.yaw),
                    float(args.x),
                    float(args.y),
                    float(args.z),
                    float(args.distance),
                    float(args.corr_conf),
                    int(args.frame),
                    int(args.flags),
                )
                mav.mav.send(corr_msg)

            if send_status:
                status_msg = mavutil.mavlink.MAVLink_ai_landing_status_message(
                    time_boot_ms,
                    float(args.status_conf),
                    int(args.target_lost),
                    float(args.reproj),
                    float(args.cov),
                )
                mav.mav.send(status_msg)

            seq += 1
            print(
                f"t={elapsed:6.2f}s seq={seq:04d} "
                f"corr={'Y' if send_corr else 'N'} "
                f"status={'Y' if send_status else 'N'} "
                f"time_boot_ms={time_boot_ms}"
            )

            time.sleep(period)

    except KeyboardInterrupt:
        print("STOP by user")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())