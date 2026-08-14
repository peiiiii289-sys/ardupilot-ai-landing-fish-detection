#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
flightstack_story_fulltest.py

穩定版順序測試：
先驗證最重要的資料流：
1. 自訂 MAVLink dialect 存在
2. AI_LANDING_STATUS 可送進飛控
3. AI_LANDING_CORRECTION 可送進飛控
4. Camera offset / xyz 可看到非 0
5. Fish Detection 可送出

執行：
python3 flightstack_story_fulltest.py --master tcp:127.0.0.1:5762
"""

import argparse
import os
import sys
import time
import threading

HOME = os.path.expanduser("~")
LOCAL_MAVLINK = os.path.join(HOME, "ardupilot", "modules", "mavlink")
if os.path.isdir(LOCAL_MAVLINK) and LOCAL_MAVLINK not in sys.path:
    sys.path.insert(0, LOCAL_MAVLINK)

os.environ.setdefault("MAVLINK20", "1")
os.environ.setdefault("MAVLINK_DIALECT", "ardupilotmega")

from pymavlink import mavutil


AI_SYS_ID = 42
AI_COMP_ID = 211
FRAME_CAMERA = 20


class FlightstackStoryFullTest:
    def __init__(self, master, frame):
        self.master = master
        self.frame = frame
        self.conn = None
        self.mav = mavutil.mavlink
        self.stop_reader = False
        self.texts = []

    def now_ms(self):
        return int(time.monotonic() * 1000) & 0xFFFFFFFF

    def connect(self):
        print("=" * 72)
        print(f"[CONNECT] {self.master}")
        print(f"[AI] sysid={AI_SYS_ID}, compid={AI_COMP_ID}")
        print("=" * 72)

        self.conn = mavutil.mavlink_connection(
            self.master,
            source_system=AI_SYS_ID,
            source_component=AI_COMP_ID,
            autoreconnect=True,
            robust_parsing=True,
        )

        self.send_heartbeat()

        th = threading.Thread(target=self.reader_loop, daemon=True)
        th.start()

        time.sleep(1)
        print("[OK] connected")

    def send_heartbeat(self):
        if self.conn is None:
            return

        self.conn.mav.heartbeat_send(
            getattr(self.mav, "MAV_TYPE_ONBOARD_CONTROLLER", 18),
            getattr(self.mav, "MAV_AUTOPILOT_INVALID", 8),
            0,
            0,
            0
        )

    def reader_loop(self):
        last_hb = 0

        while not self.stop_reader:
            now = time.time()

            if now - last_hb > 1:
                try:
                    self.send_heartbeat()
                except Exception:
                    pass
                last_hb = now

            try:
                msg = self.conn.recv_match(blocking=True, timeout=0.2)
            except Exception:
                continue

            if msg is None:
                continue

            msg_type = msg.get_type()

            if msg_type == "STATUSTEXT":
                text = getattr(msg, "text", "")
                if isinstance(text, bytes):
                    text = text.decode(errors="ignore")
                text = str(text).strip("\x00")
                self.texts.append((time.time(), text))
                print(f"[STATUSTEXT] {text}")

            elif msg_type == "COMMAND_LONG":
                cmd = int(getattr(msg, "command", -1))
                print(f"[COMMAND_LONG] cmd={cmd}")

                if cmd in (31010, 31011, 31012, 31013):
                    self.send_ack(cmd)

            elif msg_type == "COMMAND_ACK":
                cmd = int(getattr(msg, "command", -1))
                result = int(getattr(msg, "result", -1))
                print(f"[COMMAND_ACK] cmd={cmd} result={result}")

    def send_ack(self, command, result=0, result_param2=0):
        print(f"[SEND] ACK command={command} result={result} param2={result_param2}")
        try:
            self.conn.mav.command_ack_send(
                int(command),
                int(result),
                0,
                int(result_param2),
                1,
                1
            )
        except TypeError:
            self.conn.mav.command_ack_send(int(command), int(result))

    def check_dialect(self):
        print("\n" + "-" * 72)
        print("[TEST 1] 檢查自訂 MAVLink dialect")

        required = [
            "MAVLink_ai_landing_status_message",
            "MAVLink_ai_landing_correction_message",
            "MAVLink_ai_fish_detection_result_message",
        ]

        ok = True
        for name in required:
            exists = hasattr(self.mav, name)
            print(f"{name}: {exists}")
            if not exists:
                ok = False

        print("[PASS]" if ok else "[FAIL]", "custom dialect check")
        return ok

    def send_status(self, confidence=0.90, target_lost=0, reproj_error=0.05, covariance=0.02):
        """
        注意：
        這裡一定要照 ai_landing_status_send 的參數順序：
        time_boot_ms, visual_confidence, target_lost, reproj_error, covariance
        """
        self.conn.mav.ai_landing_status_send(
            int(self.now_ms()),
            float(confidence),
            int(target_lost),
            float(reproj_error),
            float(covariance),
        )

        print(
            f"[SEND] STATUS conf={confidence:.2f} lost={target_lost} "
            f"reproj={reproj_error:.2f} cov={covariance:.2f}"
        )

    def send_correction(
        self,
        roll=0.0,
        pitch=-0.02,
        yaw=0.01,
        x=0.0,
        y=0.0,
        z=0.0,
        distance=2.5,
        confidence=0.90,
        flags=3,
    ):
        """
        參數順序：
        time_boot_ms, roll_err, pitch_err, yaw_err,
        x_err, y_err, z_err, distance, confidence, frame, flags
        """
        self.conn.mav.ai_landing_correction_send(
            int(self.now_ms()),
            float(roll),
            float(pitch),
            float(yaw),
            float(x),
            float(y),
            float(z),
            float(distance),
            float(confidence),
            int(self.frame),
            int(flags),
        )

        print(
            f"[SEND] CORR pitch={pitch:.3f} xyz=({x:.2f},{y:.2f},{z:.2f}) "
            f"dist={distance:.2f} conf={confidence:.2f} flags={flags}"
        )

    def send_fish(
        self,
        width=640,
        height=480,
        coverage=45.0,
        fish_count=32,
        tuna=78.0,
        bird_count=1,
        fps=5.0,
    ):
        """
        你本機 Fish ordered_fieldnames 是：
        time_boot_ms, fish_coverage_pct, tuna_similarity_pct, inference_fps,
        image_width, image_height, fish_count, bird_count
        """
        self.conn.mav.ai_fish_detection_result_send(
            int(self.now_ms()),
            float(coverage),
            float(tuna),
            float(fps),
            int(width),
            int(height),
            int(fish_count),
            int(bird_count),
        )

        print(
            f"[SEND] FISH cov={coverage:.1f} fish={fish_count} tuna={tuna:.1f} "
            f"bird={bird_count} fps={fps:.1f} size={width}x{height}"
        )

    def param_set(self, name, value):
        self.conn.mav.param_set_send(
            1,
            1,
            name.encode("ascii"),
            float(value),
            getattr(self.mav, "MAV_PARAM_TYPE_REAL32", 9),
        )
        print(f"[PARAM_SET] {name}={value}")

    def wait_text(self, keyword, timeout=5):
        start = time.time()
        while time.time() - start < timeout:
            for ts, text in self.texts:
                if ts >= start and keyword in text:
                    return text
            time.sleep(0.2)
        return None

    def stream_normal(self, seconds=5):
        end = time.time() + seconds
        while time.time() < end:
            self.send_status(0.90, 0, 0.05, 0.02)
            self.send_correction(pitch=-0.02, x=0, y=0, z=0, confidence=0.90, flags=3)
            time.sleep(0.2)

    def test_normal_flow(self):
        print("\n" + "-" * 72)
        print("[TEST 2] 正常 Landing AI 資料流")
        print("接下來會送 5 秒 status + correction。")
        print("你應該在 MAVProxy / MissionPlanner 看到：")
        print("  AI_STAT conf=0.90")
        print("  AI_CORR conf=0.90")
        print("  QLAND AI ... conf=0.90 ... ok=1")
        print("  或至少不再一直是 conf=0.00")
        self.stream_normal(5)

    def test_xyz(self):
        print("\n" + "-" * 72)
        print("[TEST 3] 測 xyz 非 0")
        print("接下來會送 xyz=(10,20,30)。")
        for _ in range(10):
            self.send_status(0.90, 0, 0.05, 0.02)
            self.send_correction(x=10, y=20, z=30, confidence=0.90)
            time.sleep(0.2)

        print("預期看到：AI_CAM_OFF raw=(10.00,20.00,30.00)")

    def test_offset(self):
        print("\n" + "-" * 72)
        print("[TEST 4] 測 Camera Offset")
        print("設定 AIL_CAM_X/Y/Z = 1/2/3，再送 xyz=(10,20,30)。")
        self.param_set("AIL_CAM_X", 1)
        self.param_set("AIL_CAM_Y", 2)
        self.param_set("AIL_CAM_Z", 3)
        time.sleep(1)

        for _ in range(10):
            self.send_status(0.90, 0, 0.05, 0.02)
            self.send_correction(x=10, y=20, z=30, confidence=0.90)
            time.sleep(0.2)

        print("預期看到：AI_CAM_OFF raw=(10,20,30) corr=(11,22,33)")

    def test_invalid_flags(self):
        print("\n" + "-" * 72)
        print("[TEST 5] 測 invalid flags")
        for _ in range(5):
            self.send_status(0.90, 0, 0.05, 0.02)
            self.send_correction(flags=0, confidence=0.90)
            time.sleep(0.2)

        found = self.wait_text("AI_LANDING_CORR_INVALID", 6)
        if found:
            print(f"[PASS] invalid flags warning: {found}")
        else:
            print("[WARN] 沒看到 AI_LANDING_CORR_INVALID，請確認飛控端是否有印這個 STATUSTEXT")

    def test_fish(self):
        print("\n" + "-" * 72)
        print("[TEST 6] 測 Fish Detection")
        for _ in range(15):
            self.send_fish()
            time.sleep(0.2)

        print("預期看到：AI_FISH cov=45.0 fish=32 tuna=78.0 bird=1 fps=5.0 size=640x480")

    def run(self):
        self.connect()
        try:
            if not self.check_dialect():
                return

            print("\n" + "=" * 72)
            print("請先到 Terminal 2 MAVProxy 輸入：")
            print("  module load message")
            print("  mode QHOVER")
            print("  arm throttle force")
            print("  rc 3 1600")
            print("  等 2~3 秒")
            print("  mode QLAND")
            print()
            print("注意：先不要 rc 3 1500，避免太快 Land complete。")
            print("=" * 72)
            input("完成後按 Enter 開始送 AI 資料...")

            self.test_normal_flow()
            self.test_xyz()
            self.test_offset()
            self.test_invalid_flags()
            self.test_fish()

            print("\n" + "=" * 72)
            print("測試完成。")
            print("如果 MAVProxy / MissionPlanner 看到 conf=0.90、xyz 非 0、AI_FISH，代表資料流已經通。")
            print("=" * 72)

        finally:
            self.stop_reader = True
            time.sleep(0.5)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--master", default="tcp:127.0.0.1:5762")
    parser.add_argument("--frame", type=int, default=FRAME_CAMERA)
    args = parser.parse_args()

    app = FlightstackStoryFullTest(args.master, args.frame)
    app.run()


if __name__ == "__main__":
    main()
