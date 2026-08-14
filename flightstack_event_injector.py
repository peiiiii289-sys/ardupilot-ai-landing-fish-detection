#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
flightstack_event_injector.py

非順序式突發事件測試檔
用途：
    真實狀況不會照 Story 順序發生，所以這支讓你手動推送事件。

使用：
    python3 flightstack_event_injector.py --master tcp:127.0.0.1:5762

常用指令：
    normal 5
    l1 3
    l2 3
    l3
    drop 5
    fish 5
    corr_invalid
    step_pos 5
    step_neg 5
    offset 1 2 3
    xyz 10 20 30
"""

import argparse
import os
import sys
import time
import threading
import inspect
import shlex

HOME = os.path.expanduser("~")
LOCAL_MAVLINK = os.path.join(HOME, "ardupilot", "modules", "mavlink")
if LOCAL_MAVLINK not in sys.path:
    sys.path.insert(0, LOCAL_MAVLINK)

os.environ.setdefault("MAVLINK20", "1")
os.environ.setdefault("MAVLINK_DIALECT", "ardupilotmega")

from pymavlink import mavutil


AI_SYS_ID = 42
AI_COMP_ID = 211

CMD_START_AI_LANDING = 31010
CMD_STOP_AI_LANDING = 31011
CMD_START_AI_VISION = 31012
CMD_STOP_AI_VISION = 31013

FRAME_CAMERA = 20


class FlightstackEventInjector:
    def __init__(self, master, frame):
        self.master = master
        self.frame = frame
        self.conn = None
        self.mav = mavutil.mavlink
        self.stop_reader = False
        self.loop_thread = None
        self.loop_stop = threading.Event()
        self.auto_ack = True

    # ------------------------------------------------------------
    # 連線
    # ------------------------------------------------------------

    def connect(self):
        print(f"[CONNECT] {self.master}")

        self.conn = mavutil.mavlink_connection(
            self.master,
            source_system=AI_SYS_ID,
            source_component=AI_COMP_ID,
            autoreconnect=True,
            robust_parsing=True
        )

        self.send_heartbeat()

        t = threading.Thread(target=self.reader_loop, daemon=True)
        t.start()

        print("[OK] AI event injector connected")

    def send_heartbeat(self):
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

            msg = self.conn.recv_match(blocking=True, timeout=0.2)

            if msg is None:
                continue

            msg_type = msg.get_type()

            if msg_type == "STATUSTEXT":
                text = msg.text
                if isinstance(text, bytes):
                    text = text.decode(errors="ignore")
                text = text.strip("\x00")
                print(f"\n[STATUSTEXT] {text}\n> ", end="", flush=True)

            elif msg_type == "COMMAND_LONG":
                cmd = int(msg.command)
                p1 = float(msg.param1)
                p2 = float(msg.param2)
                p3 = float(msg.param3)

                print(
                    f"\n[COMMAND_LONG] cmd={cmd}, "
                    f"p1={p1}, p2={p2}, p3={p3}\n> ",
                    end="",
                    flush=True
                )

                if self.auto_ack and cmd in [
                    CMD_START_AI_LANDING,
                    CMD_STOP_AI_LANDING,
                    CMD_START_AI_VISION,
                    CMD_STOP_AI_VISION
                ]:
                    self.send_ack(cmd, 0, 0)

            elif msg_type == "COMMAND_ACK":
                cmd = int(msg.command)
                result = int(msg.result)
                param2 = int(getattr(msg, "result_param2", 0))

                print(
                    f"\n[COMMAND_ACK] cmd={cmd}, result={result}, param2={param2}\n> ",
                    end="",
                    flush=True
                )

    # ------------------------------------------------------------
    # MAVLink 自訂訊息
    # ------------------------------------------------------------

    def make_custom_msg(self, class_name, values):
        if not hasattr(self.mav, class_name):
            raise RuntimeError(
                f"找不到 {class_name}\n"
                f"請確認 MAVLINK_DIALECT=ardupilotmega、PYTHONPATH、以及 mavgen。"
            )

        cls = getattr(self.mav, class_name)

        try:
            return cls(**values)
        except TypeError:
            fieldnames = getattr(cls, "fieldnames", None)

            if not fieldnames:
                sig = inspect.signature(cls)
                fieldnames = [
                    name for name in sig.parameters.keys()
                    if name != "self"
                ]

            args = []
            for name in fieldnames:
                if name not in values:
                    raise RuntimeError(f"{class_name} 缺少欄位：{name}")
                args.append(values[name])

            return cls(*args)

    def now_ms(self):
        return int(time.monotonic() * 1000) & 0xFFFFFFFF

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
        flags=3
    ):
        msg = self.make_custom_msg(
            "MAVLink_ai_landing_correction_message",
            {
                "time_boot_ms": self.now_ms(),
                "roll_err": roll,
                "pitch_err": pitch,
                "yaw_err": yaw,
                "x_err": x,
                "y_err": y,
                "z_err": z,
                "distance": distance,
                "confidence": confidence,
                "frame": self.frame,
                "flags": flags,
            }
        )

        self.conn.mav.send(msg)

        print(
            f"[SEND] CORR pitch={pitch:.3f}, "
            f"xyz=({x:.2f},{y:.2f},{z:.2f}), "
            f"conf={confidence:.2f}, flags={flags}"
        )

    def send_status(
        self,
        confidence=0.90,
        target_lost=0,
        reproj_error=0.05,
        covariance=0.02
    ):
        msg = self.make_custom_msg(
            "MAVLink_ai_landing_status_message",
            {
                "time_boot_ms": self.now_ms(),
                "visual_confidence": confidence,
                "target_lost": target_lost,
                "reproj_error": reproj_error,
                "covariance": covariance,
            }
        )

        self.conn.mav.send(msg)

        print(
            f"[SEND] STATUS conf={confidence:.2f}, "
            f"lost={target_lost}, reproj={reproj_error:.2f}, cov={covariance:.2f}"
        )

    def send_fish(
        self,
        coverage=45.0,
        fish_count=32,
        tuna=78.0,
        bird_count=1,
        fps=5.0
    ):
        """
        依照本機 pymavlink 的 ordered_fieldnames：
        time_boot_ms, fish_coverage_pct, tuna_similarity_pct, inference_fps,
        image_width, image_height, fish_count, bird_count
        """

        time_boot_ms = int(self.now_ms())

        fish_coverage_pct = float(coverage)
        tuna_similarity_pct = float(tuna)
        inference_fps = float(fps)

        image_width = int(640)
        image_height = int(480)
        fish_count = int(fish_count)
        bird_count = int(bird_count)

        self.conn.mav.ai_fish_detection_result_send(
            time_boot_ms,
            fish_coverage_pct,
            tuna_similarity_pct,
            inference_fps,
            image_width,
            image_height,
            fish_count,
            bird_count
        )

        print(
            f"[SEND] FISH cov={fish_coverage_pct:.1f}, fish={fish_count}, "
            f"tuna={tuna_similarity_pct:.1f}, bird={bird_count}, "
            f"fps={inference_fps:.1f}, size={image_width}x{image_height}"
        )

    def send_ack(self, command, result=0, result_param2=0):
        print(f"[SEND] ACK cmd={command}, result={result}, param2={result_param2}")

        try:
            self.conn.mav.command_ack_send(
                command,
                result,
                0,
                result_param2,
                1,
                1
            )
        except TypeError:
            self.conn.mav.command_ack_send(command, result)

    def param_set(self, name, value):
        print(f"[PARAM_SET] {name} = {value}")

        self.conn.mav.param_set_send(
            1,
            1,
            name.encode("ascii"),
            float(value),
            getattr(self.mav, "MAV_PARAM_TYPE_REAL32", 9)
        )

    # ------------------------------------------------------------
    # 常用事件
    # ------------------------------------------------------------

    def normal(self, seconds):
        end = time.time() + seconds

        while time.time() < end:
            self.send_status(0.90, 0, 0.05, 0.02)
            self.send_correction(confidence=0.90, flags=3)
            time.sleep(0.2)

    def l1(self, seconds):
        end = time.time() + seconds

        while time.time() < end:
            self.send_status(0.69, 0, 0.05, 0.02)
            self.send_correction(confidence=0.69, flags=3)
            time.sleep(0.2)

    def l2(self, seconds):
        end = time.time() + seconds

        while time.time() < end:
            self.send_status(0.40, 1, 0.35, 0.25)
            self.send_correction(pitch=-0.04, confidence=0.40, flags=3)
            time.sleep(0.2)

    def l3(self):
        for _ in range(30):
            self.send_status(0.35, 1, 0.40, 0.30)
            self.send_correction(pitch=-0.04, confidence=0.35, flags=3)
            time.sleep(0.2)

    def step_pos(self, seconds):
        end = time.time() + seconds

        while time.time() < end:
            self.send_status()
            self.send_correction(pitch=0.10, x=5.0)
            time.sleep(0.2)

    def step_neg(self, seconds):
        end = time.time() + seconds

        while time.time() < end:
            self.send_status()
            self.send_correction(pitch=-0.10, x=-5.0)
            time.sleep(0.2)

    def fish(self, seconds):
        end = time.time() + seconds

        while time.time() < end:
            self.send_fish()
            time.sleep(0.2)

    # ------------------------------------------------------------
    # loop 模式
    # ------------------------------------------------------------

    def start_loop(self, mode):
        self.stop_loop()

        self.loop_stop.clear()

        def worker():
            print(f"[LOOP] start {mode}")

            while not self.loop_stop.is_set():
                if mode == "normal":
                    self.send_status(0.90, 0, 0.05, 0.02)
                    self.send_correction(confidence=0.90, flags=3)

                elif mode == "l1":
                    self.send_status(0.69, 0, 0.05, 0.02)
                    self.send_correction(confidence=0.69, flags=3)

                elif mode == "l2":
                    self.send_status(0.40, 1, 0.35, 0.25)
                    self.send_correction(pitch=-0.04, confidence=0.40, flags=3)

                elif mode == "step_pos":
                    self.send_status()
                    self.send_correction(pitch=0.10, x=5.0)

                elif mode == "step_neg":
                    self.send_status()
                    self.send_correction(pitch=-0.10, x=-5.0)

                elif mode == "fish":
                    self.send_fish()

                time.sleep(0.2)

        self.loop_thread = threading.Thread(target=worker, daemon=True)
        self.loop_thread.start()

    def stop_loop(self):
        if self.loop_thread and self.loop_thread.is_alive():
            self.loop_stop.set()
            self.loop_thread.join(timeout=1)

        self.loop_thread = None

    # ------------------------------------------------------------
    # CLI
    # ------------------------------------------------------------

    def help(self):
        print(
            """
可用指令：

基本：
  normal 5             送正常資料 5 秒
  loop normal          持續送正常資料
  stop                 停止 loop
  drop 5               5 秒不送資料，測 AI_LINK_TIMEOUT

Landing 異常：
  corr_valid           送有效 correction
  corr_invalid         送 flags=0，測 AI_LANDING_CORR_INVALID
  l1 3                 低信心，測 L1
  l2 3                 目標遺失/低信心，測 L2
  l3                   repeated L2，測 L3 takeover

穩定性：
  step_pos 5           pitch=+0.10 rad
  step_neg 5           pitch=-0.10 rad
  loop step_pos
  loop step_neg

Camera offset：
  offset 1 2 3         設定 AIL_CAM_X/Y/Z
  xyz 10 20 30         送 correction xyz=(10,20,30)

Fish：
  fish 5               送 Fish Detection 5 秒
  loop fish            持續送 Fish Detection

ACK：
  ack 31010 0 0        回 START_AI_LANDING 成功
  ack 31011 0 0        回 STOP_AI_LANDING 成功
  ack 31012 0 0        回 START_AI_VISION 成功
  ack 31013 0 0        回 STOP_AI_VISION 成功
  ack 31012 4 1002     模擬 START_AI_VISION 失敗

設定：
  autoack on
  autoack off
  param AILND_STRT_ALT 50

其他：
  hb
  help
  quit
"""
        )

    def run(self):
        self.connect()
        self.help()

        while True:
            try:
                line = input("> ").strip()
            except EOFError:
                break

            if not line:
                continue

            try:
                parts = shlex.split(line)
                cmd = parts[0].lower()

                if cmd in ["quit", "exit", "q"]:
                    break

                elif cmd == "help":
                    self.help()

                elif cmd == "hb":
                    self.send_heartbeat()

                elif cmd == "normal":
                    sec = float(parts[1]) if len(parts) > 1 else 3
                    self.normal(sec)

                elif cmd == "corr_valid":
                    self.send_correction(flags=3)

                elif cmd == "corr_invalid":
                    self.send_correction(flags=0)

                elif cmd == "l1":
                    sec = float(parts[1]) if len(parts) > 1 else 3
                    self.l1(sec)

                elif cmd == "l2":
                    sec = float(parts[1]) if len(parts) > 1 else 3
                    self.l2(sec)

                elif cmd == "l3":
                    self.l3()

                elif cmd == "drop":
                    sec = float(parts[1]) if len(parts) > 1 else 5
                    print(f"[DROP] {sec} 秒內不送任何 AI 訊息")
                    time.sleep(sec)

                elif cmd == "step_pos":
                    sec = float(parts[1]) if len(parts) > 1 else 3
                    self.step_pos(sec)

                elif cmd == "step_neg":
                    sec = float(parts[1]) if len(parts) > 1 else 3
                    self.step_neg(sec)

                elif cmd == "fish":
                    sec = float(parts[1]) if len(parts) > 1 else 3
                    self.fish(sec)

                elif cmd == "offset":
                    x = float(parts[1])
                    y = float(parts[2])
                    z = float(parts[3])
                    self.param_set("AIL_CAM_X", x)
                    self.param_set("AIL_CAM_Y", y)
                    self.param_set("AIL_CAM_Z", z)

                elif cmd == "xyz":
                    x = float(parts[1])
                    y = float(parts[2])
                    z = float(parts[3])
                    self.send_status()
                    self.send_correction(x=x, y=y, z=z)

                elif cmd == "ack":
                    command = int(parts[1])
                    result = int(parts[2]) if len(parts) > 2 else 0
                    param2 = int(parts[3]) if len(parts) > 3 else 0
                    self.send_ack(command, result, param2)

                elif cmd == "autoack":
                    value = parts[1].lower()
                    self.auto_ack = value in ["on", "1", "true", "yes"]
                    print(f"[AUTOACK] {self.auto_ack}")

                elif cmd == "param":
                    name = parts[1]
                    value = float(parts[2])
                    self.param_set(name, value)

                elif cmd == "loop":
                    mode = parts[1].lower()
                    if mode not in ["normal", "l1", "l2", "step_pos", "step_neg", "fish"]:
                        print("loop 只支援 normal / l1 / l2 / step_pos / step_neg / fish")
                    else:
                        self.start_loop(mode)

                elif cmd == "stop":
                    self.stop_loop()

                else:
                    print("未知指令，輸入 help 查看")

            except Exception as e:
                print(f"[ERROR] {e}")

        self.stop_loop()
        self.stop_reader = True
        print("[EXIT]")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--master", default="tcp:127.0.0.1:5762")
    parser.add_argument("--frame", type=int, default=FRAME_CAMERA)
    args = parser.parse_args()

    app = FlightstackEventInjector(args.master, args.frame)
    app.run()


if __name__ == "__main__":
    main()