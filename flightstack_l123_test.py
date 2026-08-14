#!/usr/bin/env python3
# -*- coding: utf-8 -*-

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


class L123Tester:
    def __init__(self, master, frame):
        self.master = master
        self.frame = frame
        self.mav = mavutil.mavlink
        self.conn = None
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
        t = threading.Thread(target=self.reader_loop, daemon=True)
        t.start()
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
            0,
        )

    def reader_loop(self):
        last_hb = 0.0

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
                if cmd in (31010, 31011):
                    self.send_ack(cmd)

            elif msg_type == "COMMAND_ACK":
                cmd = int(getattr(msg, "command", -1))
                result = int(getattr(msg, "result", -1))
                print(f"[COMMAND_ACK] cmd={cmd} result={result}")

    def send_ack(self, command, result=0, param2=0):
        print(f"[SEND] ACK cmd={command} result={result} param2={param2}")
        try:
            self.conn.mav.command_ack_send(
                int(command),
                int(result),
                0,
                int(param2),
                1,
                1,
            )
        except TypeError:
            self.conn.mav.command_ack_send(int(command), int(result))

    def send_status(self, confidence, target_lost, reproj_error, covariance):
        self.conn.mav.ai_landing_status_send(
            int(self.now_ms()),
            float(confidence),
            int(target_lost),
            float(reproj_error),
            float(covariance),
        )

        print(
            f"[SEND] STATUS conf={confidence:.2f} "
            f"lost={target_lost} reproj={reproj_error:.2f} cov={covariance:.2f}"
        )

    def send_correction(
        self,
        pitch=-0.02,
        x=0.0,
        y=0.0,
        z=0.0,
        confidence=0.90,
        flags=3,
        distance=2.5,
    ):
        self.conn.mav.ai_landing_correction_send(
            int(self.now_ms()),
            float(0.0),
            float(pitch),
            float(0.01),
            float(x),
            float(y),
            float(z),
            float(distance),
            float(confidence),
            int(self.frame),
            int(flags),
        )

        print(
            f"[SEND] CORR pitch={pitch:.3f} "
            f"xyz=({x:.2f},{y:.2f},{z:.2f}) "
            f"conf={confidence:.2f} flags={flags}"
        )

    def send_vector(
        self,
        seconds,
        confidence,
        target_lost,
        reproj_error,
        covariance,
        pitch=-0.02,
        hz=5,
    ):
        end = time.time() + seconds
        dt = 1.0 / hz

        while time.time() < end:
            self.send_status(confidence, target_lost, reproj_error, covariance)
            self.send_correction(
                pitch=pitch,
                confidence=confidence,
                flags=3,
            )
            time.sleep(dt)

    def wait_text(self, keywords, since, timeout):
        if isinstance(keywords, str):
            keywords = [keywords]

        end = time.time() + timeout
        while time.time() < end:
            for ts, text in self.texts:
                if ts >= since:
                    for kw in keywords:
                        if kw in text:
                            return text
            time.sleep(0.2)

        return None

    def pause(self, text):
        print("\n" + "=" * 72)
        print(text)
        print("=" * 72)
        input("完成後按 Enter 繼續...")

    def check_dialect(self):
        print("\n[CHECK] MAVLink dialect")
        required = [
            "MAVLink_ai_landing_status_message",
            "MAVLink_ai_landing_correction_message",
        ]

        ok = True
        for name in required:
            exists = hasattr(self.mav, name)
            print(f"{name}: {exists}")
            if not exists:
                ok = False

        if not ok:
            raise RuntimeError("找不到 AI Landing 自訂 MAVLink message")

    def test_normal(self):
        print("\n" + "-" * 72)
        print("[TEST 0] Normal baseline")
        print("條件：conf=0.90, lost=0, reproj=0.05, cov=0.02")
        print("預期：L1=0, L2=0, L3=0, QLAND AI ok=1")

        self.send_vector(
            seconds=5,
            confidence=0.90,
            target_lost=0,
            reproj_error=0.05,
            covariance=0.02,
            pitch=-0.02,
        )

    def test_l1_confidence(self):
        print("\n" + "-" * 72)
        print("[TEST 1] L1 by low confidence")
        print("條件：conf=0.69, lost=0, reproj=0.05, cov=0.02，持續 5 秒")
        print("預期：AI_LANDING_L1_WARNING 或 AI_STAT ... L1=1")

        since = time.time()

        self.send_vector(
            seconds=5,
            confidence=0.69,
            target_lost=0,
            reproj_error=0.05,
            covariance=0.02,
            pitch=-0.02,
        )

        found = self.wait_text(["AI_LANDING_L1_WARNING", "L1=1"], since, 5)

        if found:
            print(f"[PASS] L1 detected: {found}")
        else:
            print("[WARN] 沒抓到 L1。請看 MAVProxy/MissionPlanner 是否有 AI_LANDING_L1_WARNING 或 L1=1")

    def test_l1_reproj(self):
        print("\n" + "-" * 72)
        print("[TEST 1B] L1 by reprojection error")
        print("條件：conf=0.90, lost=0, reproj=0.15, cov=0.02，持續 5 秒")

        since = time.time()

        self.send_vector(
            seconds=5,
            confidence=0.90,
            target_lost=0,
            reproj_error=0.15,
            covariance=0.02,
            pitch=-0.02,
        )

        found = self.wait_text(["AI_LANDING_L1_WARNING", "L1=1"], since, 5)

        if found:
            print(f"[PASS] L1 by reproj detected: {found}")
        else:
            print("[INFO] 沒看到 L1 by reproj，可能程式目前主要看 confidence。")

    def test_l1_covariance(self):
        print("\n" + "-" * 72)
        print("[TEST 1C] L1 by covariance")
        print("條件：conf=0.90, lost=0, reproj=0.05, cov=0.08，持續 5 秒")

        since = time.time()

        self.send_vector(
            seconds=5,
            confidence=0.90,
            target_lost=0,
            reproj_error=0.05,
            covariance=0.08,
            pitch=-0.02,
        )

        found = self.wait_text(["AI_LANDING_L1_WARNING", "L1=1"], since, 5)

        if found:
            print(f"[PASS] L1 by covariance detected: {found}")
        else:
            print("[INFO] 沒看到 L1 by covariance，可能程式目前主要看 confidence。")

    def test_l2(self):
        print("\n" + "-" * 72)
        print("[TEST 2] L2 by target lost")
        print("條件：conf=0.40, lost=1, reproj=0.35, cov=0.25，持續 5 秒")
        print("預期：AI_LANDING_ABORT_L2 / AI_GO_AROUND / L2=1")

        since = time.time()

        self.send_vector(
            seconds=5,
            confidence=0.40,
            target_lost=1,
            reproj_error=0.35,
            covariance=0.25,
            pitch=-0.04,
        )

        found_l2 = self.wait_text(["AI_LANDING_ABORT_L2", "L2=1"], since, 5)
        found_go = self.wait_text(["AI_GO_AROUND", "Mode QLOITER"], since, 5)

        if found_l2 or found_go:
            print(f"[PASS] L2/Go-Around detected: L2={found_l2}, GO={found_go}")
        else:
            print("[WARN] 沒抓到 L2/Go-Around。請確認目前真的在 QLAND。")

    def test_l3(self):
        print("\n" + "-" * 72)
        print("[TEST 3] L3 by repeated L2")
        print("條件：重複送 L2 狀態 12 秒")
        print("預期：AI_LANDING_TAKEOVER / L3=1")

        self.pause(
            "如果剛才 L2 已切到 QLOITER，請先重新進 QLAND：\n"
            "  mode QHOVER\n"
            "  arm throttle force\n"
            "  rc 3 1700\n"
            "  等 2 秒\n"
            "  mode QLAND\n\n"
            "完成後按 Enter。"
        )

        since = time.time()

        self.send_vector(
            seconds=12,
            confidence=0.35,
            target_lost=1,
            reproj_error=0.40,
            covariance=0.30,
            pitch=-0.04,
        )

        found_l3 = self.wait_text(["AI_LANDING_TAKEOVER", "L3=1"], since, 8)

        if found_l3:
            print(f"[PASS] L3 detected: {found_l3}")
        else:
            print("[WARN] 沒抓到 L3。可能條件是 1 分鐘內至少 2 次 L2，需要更長時間或分兩輪。")

    def run(self):
        self.connect()
        self.check_dialect()

        self.pause(
            "請先到 MAVProxy Terminal 輸入：\n"
            "  module load message\n"
            "  mode QHOVER\n"
            "  arm throttle force\n"
            "  rc 3 1700\n"
            "  等 3~5 秒\n"
            "  mode QLAND\n\n"
            "完成後回到這裡按 Enter，開始 L1/L2/L3 測試。"
        )

        try:
            self.test_normal()
            self.test_l1_confidence()
            self.test_l1_reproj()
            self.test_l1_covariance()
            self.test_l2()
            self.test_l3()

            print("\n" + "=" * 72)
            print("L1/L2/L3 測試結束")
            print("成功重點：")
            print("  L1：AI_LANDING_L1_WARNING 或 AI_STAT ... L1=1")
            print("  L2：AI_LANDING_ABORT_L2 / AI_GO_AROUND 或 AI_STAT ... L2=1")
            print("  L3：AI_LANDING_TAKEOVER 或 AI_STAT ... L3=1")
            print("=" * 72)

        finally:
            self.stop_reader = True
            time.sleep(0.5)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--master", default="tcp:127.0.0.1:5762")
    parser.add_argument("--frame", type=int, default=FRAME_CAMERA)
    args = parser.parse_args()

    app = L123Tester(args.master, args.frame)
    app.run()


if __name__ == "__main__":
    main()
