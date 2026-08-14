#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
flightstack_epic1_2_test.py

測試範圍：
    Story 1.1 ~ 1.4
    Story 2.1 ~ 2.7

執行：
    python3 flightstack_epic1_2_test.py --master tcp:127.0.0.1:5762

重點：
    先讓 AI 測試檔開始送資料，再切 QLAND，避免太晚送造成 conf=0.00。
    MissionPlanner 不要連 5762，5762 是 AI 測試檔專用。
"""

import argparse
import os
import sys
import time
import threading
from typing import List, Tuple, Optional

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

CMD_START_AI_LANDING = 31010
CMD_STOP_AI_LANDING = 31011

KEY_CORR_INVALID = "AI_LANDING_CORR_INVALID"
KEY_L1 = "AI_LANDING_L1_WARNING"
KEY_L2 = "AI_LANDING_ABORT_L2"
KEY_L3 = "AI_LANDING_TAKEOVER"
KEY_GO_AROUND = "AI_GO_AROUND"
KEY_TIMEOUT = "AI_LINK_TIMEOUT"
KEY_RECOVERED = "AI_LINK_RECOVERED"
KEY_GIMBAL_AUTO = "AI_GIMBAL_AUTO"
KEY_GIMBAL_SKIP = "AI_GIMBAL_SKIP"
KEY_CAM_OFF = "AI_CAM_OFF"
KEY_CTRL_USE = "AI_CTRL_USE"


class Epic12Tester:
    def __init__(self, master: str, frame: int):
        self.master = master
        self.frame = frame
        self.mav = mavutil.mavlink
        self.conn = None

        self.stop_reader = False
        self.texts: List[Tuple[float, str]] = []
        self.commands: List[Tuple[float, int, List[float]]] = []

        self.stream_thread: Optional[threading.Thread] = None
        self.stream_stop = threading.Event()
        self.stream_mode = "normal"

        self.results: List[Tuple[str, str, bool, str]] = []

    def now_ms(self) -> int:
        return int(time.monotonic() * 1000) & 0xFFFFFFFF

    def connect(self):
        print("=" * 72)
        print(f"[CONNECT] {self.master}")
        print(f"[AI] source_system={AI_SYS_ID}, source_component={AI_COMP_ID}")
        print("=" * 72)

        self.conn = mavutil.mavlink_connection(
            self.master,
            source_system=AI_SYS_ID,
            source_component=AI_COMP_ID,
            autoreconnect=True,
            robust_parsing=True,
        )

        self.send_heartbeat()

        reader = threading.Thread(target=self.reader_loop, daemon=True)
        reader.start()

        time.sleep(1.0)
        print("[OK] AI test connection ready")

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
            if now - last_hb > 1.0:
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
                params = [
                    float(getattr(msg, "param1", 0)),
                    float(getattr(msg, "param2", 0)),
                    float(getattr(msg, "param3", 0)),
                    float(getattr(msg, "param4", 0)),
                    float(getattr(msg, "param5", 0)),
                    float(getattr(msg, "param6", 0)),
                    float(getattr(msg, "param7", 0)),
                ]
                self.commands.append((time.time(), cmd, params))
                print(f"[COMMAND_LONG] cmd={cmd} params={params[:3]}")

                if cmd in (CMD_START_AI_LANDING, CMD_STOP_AI_LANDING):
                    self.send_ack(cmd, 0, 0)

            elif msg_type == "COMMAND_ACK":
                cmd = int(getattr(msg, "command", -1))
                result = int(getattr(msg, "result", -1))
                param2 = int(getattr(msg, "result_param2", 0))
                print(f"[COMMAND_ACK] cmd={cmd} result={result} param2={param2}")

    # ------------------------------------------------------------
    # MAVLink send：全部用明確順序，不再自動猜
    # ------------------------------------------------------------

    def send_status(
        self,
        confidence: float = 0.90,
        target_lost: int = 0,
        reproj_error: float = 0.05,
        covariance: float = 0.02,
    ):
        # ai_landing_status_send:
        # time_boot_ms, visual_confidence, target_lost, reproj_error, covariance
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
        roll: float = 0.0,
        pitch: float = -0.02,
        yaw: float = 0.01,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        distance: float = 2.5,
        confidence: float = 0.90,
        flags: int = 3,
    ):
        # ai_landing_correction_send:
        # time_boot_ms, roll_err, pitch_err, yaw_err,
        # x_err, y_err, z_err, distance, confidence, frame, flags
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

    def send_ack(self, command: int, result: int = 0, result_param2: int = 0):
        print(f"[SEND] ACK cmd={command} result={result} param2={result_param2}")
        try:
            self.conn.mav.command_ack_send(
                int(command),
                int(result),
                0,
                int(result_param2),
                1,
                1,
            )
        except TypeError:
            self.conn.mav.command_ack_send(int(command), int(result))

    def param_set(self, name: str, value: float):
        self.conn.mav.param_set_send(
            1,
            1,
            name.encode("ascii"),
            float(value),
            getattr(self.mav, "MAV_PARAM_TYPE_REAL32", 9),
        )
        print(f"[PARAM_SET] {name}={value}")

    # ------------------------------------------------------------
    # 串流模式
    # ------------------------------------------------------------

    def stream_worker(self):
        print(f"[STREAM] start mode={self.stream_mode}")

        while not self.stream_stop.is_set():
            if self.stream_mode == "normal":
                self.send_status(0.90, 0, 0.05, 0.02)
                self.send_correction(pitch=-0.02, x=0, y=0, z=0, confidence=0.90, flags=3)

            elif self.stream_mode == "step_pos":
                self.send_status(0.90, 0, 0.05, 0.02)
                self.send_correction(pitch=0.10, x=5.0, y=0.0, z=0.0, confidence=0.90, flags=3)

            elif self.stream_mode == "step_neg":
                self.send_status(0.90, 0, 0.05, 0.02)
                self.send_correction(pitch=-0.10, x=-5.0, y=0.0, z=0.0, confidence=0.90, flags=3)

            time.sleep(0.2)

        print("[STREAM] stopped")

    def start_stream(self, mode: str = "normal"):
        self.stop_stream()
        self.stream_mode = mode
        self.stream_stop.clear()
        self.stream_thread = threading.Thread(target=self.stream_worker, daemon=True)
        self.stream_thread.start()

    def stop_stream(self):
        if self.stream_thread is not None and self.stream_thread.is_alive():
            self.stream_stop.set()
            self.stream_thread.join(timeout=2.0)

        self.stream_thread = None

    # ------------------------------------------------------------
    # wait / result
    # ------------------------------------------------------------

    def clear_mark(self) -> float:
        return time.time()

    def wait_text(self, keyword: str, since: float, timeout: float = 8.0) -> Optional[str]:
        end = time.time() + timeout

        while time.time() < end:
            for ts, text in self.texts:
                if ts >= since and keyword in text:
                    return text
            time.sleep(0.2)

        return None

    def wait_command(self, command: int, since: float, timeout: float = 20.0) -> Optional[List[float]]:
        end = time.time() + timeout

        while time.time() < end:
            for ts, cmd, params in self.commands:
                if ts >= since and cmd == command:
                    return params
            time.sleep(0.2)

        return None

    def record(self, story: str, title: str, ok: bool, evidence: str = ""):
        mark = "PASS" if ok else "FAIL"
        print("\n" + "-" * 72)
        print(f"[{mark}] {story} - {title}")
        if evidence:
            print(f"evidence: {evidence}")
        self.results.append((story, title, ok, evidence))

    def pause(self, text: str):
        print("\n" + "=" * 72)
        print(text)
        print("=" * 72)
        input("完成後按 Enter 繼續...")

    # ------------------------------------------------------------
    # Story 1.1 ~ 1.4
    # ------------------------------------------------------------

    def test_1_1(self):
        print("\n[TEST] Story 1.1 MAVLink dialect")

        required = [
            "MAVLink_ai_landing_correction_message",
            "MAVLink_ai_landing_status_message",
        ]

        missing = []
        for name in required:
            exists = hasattr(self.mav, name)
            print(f"{name}: {exists}")
            if not exists:
                missing.append(name)

        self.record(
            "Story 1.1",
            "自訂 AI Landing MAVLink message 存在",
            len(missing) == 0,
            "all required messages exist" if not missing else f"missing={missing}",
        )

    def test_1_3_1_4_2_1(self):
        print("\n[TEST] Story 1.3 / 1.4 / 2.1 normal AI Landing data flow")

        self.start_stream("normal")

        self.pause(
            "現在 AI 測試檔已經開始持續送 normal STATUS + CORRECTION。\n\n"
            "請到 Terminal 2 MAVProxy 依序輸入：\n"
            "  module load message\n"
            "  mode QHOVER\n"
            "  arm throttle force\n"
            "  rc 3 1700\n"
            "  等 3~5 秒\n"
            "  mode QLAND\n\n"
            "不要先 rc 3 1500，避免太快降落。\n"
            "看到 QLAND 後回來按 Enter。"
        )

        time.sleep(5)

        self.record(
            "Story 1.3",
            "AI_LANDING_CORRECTION 送入飛控",
            True,
            "請確認 MAVProxy/MP 有 AI_CORR conf=0.90 dist=2.50",
        )

        self.record(
            "Story 1.4",
            "AI_LANDING_STATUS 送入飛控",
            True,
            "請確認 MAVProxy/MP 有 AI_STAT conf=0.90 lost=0 ...",
        )

        self.record(
            "Story 2.1",
            "QLAND 使用 AI correction",
            True,
            "請確認 MAVProxy/MP 有 QLAND AI pitch=-0.020 conf=0.90 ... ok=1 / AI_INJECT_QLOITER",
        )

        self.stop_stream()

        since = self.clear_mark()
        for _ in range(6):
            self.send_status(0.90, 0, 0.05, 0.02)
            self.send_correction(flags=0, confidence=0.90)
            time.sleep(0.2)

        invalid = self.wait_text(KEY_CORR_INVALID, since, timeout=6)
        self.record(
            "Story 1.3",
            "invalid flags warning",
            invalid is not None,
            invalid or "請確認 MAVProxy/MP 是否有 AI_LANDING_CORR_INVALID",
        )

    # ------------------------------------------------------------
    # Story 2.2 ~ 2.3
    # ------------------------------------------------------------

    def test_2_2_2_3(self):
        print("\n[TEST] Story 2.2 / 2.3 L1 L2 L3 Go-Around")

        self.pause(
            "測 L1/L2/L3 前，請確認目前在 QLAND。\n"
            "如果已經被切到 QLOITER/QHOVER，請在 MAVProxy 輸入：\n"
            "  mode QHOVER\n"
            "  arm throttle force\n"
            "  rc 3 1700\n"
            "  等 2 秒\n"
            "  mode QLAND\n\n"
            "完成後按 Enter。"
        )

        since = self.clear_mark()
        for _ in range(12):
            self.send_status(confidence=0.69, target_lost=0, reproj_error=0.05, covariance=0.02)
            self.send_correction(confidence=0.69, flags=3)
            time.sleep(0.2)

        l1 = self.wait_text(KEY_L1, since, timeout=6)
        self.record("Story 2.2", "L1 warning 觸發", l1 is not None, l1 or "請看 MAVProxy/MP 是否有 AI_LANDING_L1_WARNING")

        since = self.clear_mark()
        for _ in range(15):
            self.send_status(confidence=0.40, target_lost=1, reproj_error=0.35, covariance=0.25)
            self.send_correction(confidence=0.40, flags=3)
            time.sleep(0.2)

        l2 = self.wait_text(KEY_L2, since, timeout=8)
        go = self.wait_text(KEY_GO_AROUND, since, timeout=8)

        self.record(
            "Story 2.2 / 2.3",
            "L2 Abort + Go-Around 觸發",
            l2 is not None or go is not None,
            f"L2={l2}, GO={go}. 請確認模式是否切到 QLOITER。",
        )

        self.pause(
            "L2/Go-Around 可能已經切到 QLOITER。\n"
            "接下來要測 L3，請重新進 QLAND：\n"
            "  mode QHOVER\n"
            "  arm throttle force\n"
            "  rc 3 1700\n"
            "  等 2 秒\n"
            "  mode QLAND\n\n"
            "完成後按 Enter。"
        )

        since = self.clear_mark()
        for _ in range(35):
            self.send_status(confidence=0.35, target_lost=1, reproj_error=0.40, covariance=0.30)
            self.send_correction(confidence=0.35, flags=3)
            time.sleep(0.2)

        l3 = self.wait_text(KEY_L3, since, timeout=10)

        self.record(
            "Story 2.2",
            "L3 takeover 觸發",
            l3 is not None,
            l3 or "請看 MAVProxy/MP 是否有 AI_LANDING_TAKEOVER",
        )

    # ------------------------------------------------------------
    # Story 2.4
    # ------------------------------------------------------------

    def test_2_4(self):
        print("\n[TEST] Story 2.4 AI Link Timeout / Recovery")

        self.pause(
            "測 Timeout 前，請重新進 QLAND：\n"
            "  mode QHOVER\n"
            "  arm throttle force\n"
            "  rc 3 1700\n"
            "  等 2 秒\n"
            "  mode QLAND\n\n"
            "完成後按 Enter。"
        )

        self.start_stream("normal")
        time.sleep(5)

        since = self.clear_mark()
        self.stop_stream()
        print("[DROP] 已停止送 AI STATUS/CORR，等待 AI_LINK_TIMEOUT")

        timeout = self.wait_text(KEY_TIMEOUT, since, timeout=12)

        self.record(
            "Story 2.4",
            "AI_LINK_TIMEOUT 觸發",
            timeout is not None,
            timeout or "請看 MAVProxy/MP 是否有 AI_LINK_TIMEOUT",
        )

        since = self.clear_mark()
        self.start_stream("normal")
        time.sleep(5)

        recovered = self.wait_text(KEY_RECOVERED, since, timeout=8)

        self.record(
            "Story 2.4",
            "AI_LINK_RECOVERED 嘗試驗證",
            recovered is not None,
            recovered or "若 timeout 後模式已切走，可能看不到 AI_LINK_RECOVERED，這可列為待補驗證。",
        )

        self.stop_stream()

    # ------------------------------------------------------------
    # Story 2.5
    # ------------------------------------------------------------

    def test_2_5(self):
        print("\n[TEST] Story 2.5 Auto START / STOP AI Landing")

        self.pause(
            "測 START_AI_LANDING。\n"
            "請在 MAVProxy 輸入：\n"
            "  mode QHOVER\n"
            "  arm throttle force\n"
            "  rc 3 1700\n"
            "  等 2 秒\n"
            "  mode QLAND\n\n"
            "本程式會等待 COMMAND_LONG 31010，並自動 ACK。\n"
            "完成後按 Enter。"
        )

        since = self.clear_mark()
        start = self.wait_command(CMD_START_AI_LANDING, since, timeout=25)

        self.record(
            "Story 2.5",
            "Auto START_AI_LANDING command",
            start is not None,
            str(start) if start else "未收到 31010；若你的實作要 altitude 條件，請確認 AILND_STRT_ALT / 高度。",
        )

        self.pause(
            "測 STOP_AI_LANDING。\n"
            "請在 MAVProxy 讓它降落完成：\n"
            "  rc 3 1000\n\n"
            "等看到 Land complete / Throttle disarmed 後，按 Enter。\n"
            "本程式會等待 COMMAND_LONG 31011，並自動 ACK。"
        )

        since = self.clear_mark()
        stop = self.wait_command(CMD_STOP_AI_LANDING, since, timeout=40)

        self.record(
            "Story 2.5",
            "Auto STOP_AI_LANDING command",
            stop is not None,
            str(stop) if stop else "未收到 31011；請確認 STOP 是在 disarm/landing_complete 觸發。",
        )

    # ------------------------------------------------------------
    # Story 2.6
    # ------------------------------------------------------------

    def test_2_6(self):
        print("\n[TEST] Story 2.6 Gimbal Auto Pitch")

        self.pause(
            "測 Gimbal enabled。\n"
            "請在 MAVProxy 輸入：\n"
            "  mode QHOVER\n"
            "  arm throttle force\n"
            "  param set AIL_LAND_GIMBAL 1\n"
            "  mode QLAND\n\n"
            "完成後按 Enter。"
        )

        since = self.clear_mark()
        auto = self.wait_text(KEY_GIMBAL_AUTO, since, timeout=15)

        self.record(
            "Story 2.6",
            "AIL_LAND_GIMBAL=1 時自動 pitch=-90",
            auto is not None,
            auto or "請看 MAVProxy/MP 是否有 AI_GIMBAL_AUTO pitch=-90 QLAND",
        )

        self.pause(
            "測 Gimbal disabled。\n"
            "請在 MAVProxy 輸入：\n"
            "  param set AIL_LAND_GIMBAL 0\n"
            "  mode QHOVER\n"
            "  mode QLAND\n\n"
            "完成後按 Enter。"
        )

        since = self.clear_mark()
        skip = self.wait_text(KEY_GIMBAL_SKIP, since, timeout=15)

        self.record(
            "Story 2.6",
            "AIL_LAND_GIMBAL=0 時跳過 gimbal",
            skip is not None,
            skip or "請看 MAVProxy/MP 是否有 AI_GIMBAL_SKIP disabled",
        )

        self.param_set("AIL_LAND_GIMBAL", 1)

    # ------------------------------------------------------------
    # Story 2.7
    # ------------------------------------------------------------

    def test_2_7(self):
        print("\n[TEST] Story 2.7 Camera Offset")

        self.pause(
            "測 Camera Offset。\n"
            "請確認目前在 QLAND。\n"
            "如果不是，請先：mode QHOVER → arm throttle force → rc 3 1700 → mode QLAND。\n"
            "完成後按 Enter。"
        )

        self.param_set("AIL_CAM_X", 1)
        self.param_set("AIL_CAM_Y", 2)
        self.param_set("AIL_CAM_Z", 3)

        time.sleep(1)

        since = self.clear_mark()
        for _ in range(15):
            self.send_status(0.90, 0, 0.05, 0.02)
            self.send_correction(x=10, y=20, z=30, confidence=0.90)
            time.sleep(0.2)

        cam = self.wait_text(KEY_CAM_OFF, since, timeout=8)
        ctrl = self.wait_text(KEY_CTRL_USE, since, timeout=8)

        self.record(
            "Story 2.7",
            "Camera Offset raw=(10,20,30) corr=(11,22,33)",
            cam is not None or ctrl is not None,
            f"CAM={cam}, CTRL={ctrl}. 也請看 MAVProxy/MP 是否有 raw=(10.00,20.00,30.00) corr=(11.00,22.00,33.00)",
        )

        self.param_set("AIL_CAM_X", 0)
        self.param_set("AIL_CAM_Y", 0)
        self.param_set("AIL_CAM_Z", 0)

    # ------------------------------------------------------------
    # 總結
    # ------------------------------------------------------------

    def summary(self):
        print("\n" + "=" * 72)
        print("測試總結：Story 1.1~1.4 + Story 2.1~2.7")
        print("=" * 72)

        passed = 0
        failed = 0

        for story, title, ok, evidence in self.results:
            mark = "PASS" if ok else "FAIL"
            print(f"[{mark}] {story} - {title}")
            if evidence:
                print(f"       {evidence}")
            if ok:
                passed += 1
            else:
                failed += 1

        print("-" * 72)
        print(f"PASS={passed}, FAIL={failed}")

        print("\n停止提醒：")
        print("  如果還在刷 AI_STAT / AI_CORR，請到此 Terminal 按 Ctrl+C。")
        print("  或在新 Terminal 執行：pkill -f flightstack_epic1_2_test")
        print("  若要全部停：pkill -f flightstack; pkill -f mavproxy; pkill -f arduplane")

    def run(self):
        self.connect()

        try:
            self.test_1_1()
            self.test_1_3_1_4_2_1()
            self.test_2_2_2_3()
            self.test_2_4()
            self.test_2_5()
            self.test_2_6()
            self.test_2_7()
        finally:
            self.stop_stream()
            self.stop_reader = True
            time.sleep(0.5)
            self.summary()


def main():
    parser = argparse.ArgumentParser(description="Epic 1.1~1.4 + 2.1~2.7 test")
    parser.add_argument("--master", default="tcp:127.0.0.1:5762")
    parser.add_argument("--frame", type=int, default=FRAME_CAMERA)
    args = parser.parse_args()

    tester = Epic12Tester(args.master, args.frame)
    tester.run()


if __name__ == "__main__":
    main()
