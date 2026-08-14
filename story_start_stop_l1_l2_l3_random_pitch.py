#!/usr/bin/env python3
# story_start_stop_l1_l2_l3_random_pitch.py
#
# 一次測完整流程：
# 1. 回應 START_AI_VISION / STOP_AI_VISION，也就是 COMMAND 31012 / 31013 的 ACK
# 2. 持續送 AI_LANDING_CORRECTION = MESSAGE 52100
# 3. 持續送 AI_LANDING_STATUS = MESSAGE 52102
# 4. pitch_err 使用隨機值，模擬風大造成修正角度變動
# 5. raw x/y/z 使用隨機值，驗證飛控端 AIL_CAM_X/Y/Z 自動補償成 corr
# 6. 測 normal -> L1 -> recovery -> direct L2 -> recovery -> repeated L2 -> L3 takeover

import argparse
import math
import os
import random
import sys
import time
from pathlib import Path

# 一定要使用 MAVLink2 + ardupilotmega
os.environ["MAVLINK20"] = "1"
os.environ["MAVLINK_DIALECT"] = "ardupilotmega"

HOME = Path.home()
ARDUPILOT = HOME / "ardupilot"

# 確保使用 ardupilot 內的 pymavlink / dialect
sys.path.insert(0, str(ARDUPILOT / "modules" / "mavlink"))
sys.path.insert(0, str(ARDUPILOT / "modules" / "mavlink" / "pymavlink"))

from pymavlink import mavutil

AI_SYSID = 42
AI_COMPID = 211

CMD_START_AI_VISION = 31012  # MAV_CMD_USER_3
CMD_STOP_AI_VISION = 31013   # MAV_CMD_USER_4


def now_ms() -> int:
    return int(time.monotonic() * 1000) & 0xFFFFFFFF


class StoryFullTester:
    def __init__(self, master: str, seed: int, hz: float):
        self.master = master
        self.rng = random.Random(seed)
        self.hz = hz
        self.dt = 1.0 / hz
        self.m = None
        self.seq = 0

    def connect(self):
        print(f"[CONNECT] {self.master}")

        self.m = mavutil.mavlink_connection(
            self.master,
            source_system=AI_SYSID,
            source_component=AI_COMPID,
            dialect="ardupilotmega",
        )

        print("[WAIT] heartbeat from FC...")
        self.m.wait_heartbeat(timeout=15)
        print(f"[OK] heartbeat target_system={self.m.target_system} target_component={self.m.target_component}")

        print("[DIALECT CHECK]")
        print("  ai_landing_correction_send =", hasattr(self.m.mav, "ai_landing_correction_send"))
        print("  ai_landing_status_send     =", hasattr(self.m.mav, "ai_landing_status_send"))

        if not hasattr(self.m.mav, "ai_landing_correction_send"):
            print("[FAIL] missing ai_landing_correction_send.")
            print("請確認 MAVLINK_DIALECT=ardupilotmega，且 pymavlink 已重新產生。")
            sys.exit(1)

        if not hasattr(self.m.mav, "ai_landing_status_send"):
            print("[FAIL] missing ai_landing_status_send.")
            print("請確認 MAVLINK_DIALECT=ardupilotmega，且 pymavlink 已重新產生。")
            sys.exit(1)

    def heartbeat(self):
        self.m.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            mavutil.mavlink.MAV_STATE_ACTIVE,
        )

    def poll_and_ack_commands(self):
        """
        接收 MAVProxy / 飛控送來的 COMMAND_LONG。
        如果 command 是 31012 / 31013，就回 COMMAND_ACK。
        這段是為了測 Story 3.2 START / STOP ACK。
        """
        while True:
            msg = self.m.recv_match(type="COMMAND_LONG", blocking=False)
            if msg is None:
                break

            cmd = int(msg.command)

            print(
                f"[RX COMMAND_LONG] cmd={cmd} "
                f"target_system={msg.target_system} target_component={msg.target_component} "
                f"p1={msg.param1:.1f} p2={msg.param2:.1f} p3={msg.param3:.1f}"
            )

            if cmd in (CMD_START_AI_VISION, CMD_STOP_AI_VISION):
                self.m.mav.command_ack_send(
                    cmd,
                    mavutil.mavlink.MAV_RESULT_ACCEPTED,
                    0,  # progress
                    0,  # result_param2
                    self.m.target_system or 1,
                    self.m.target_component or 1,
                )

                if cmd == CMD_START_AI_VISION:
                    print("[ACK] START_AI_VISION / 31012 accepted")
                else:
                    print("[ACK] STOP_AI_VISION / 31013 accepted")
            else:
                print(f"[IGNORE] command {cmd} is not AI vision START/STOP")

    def random_raw_and_pitch(self):
        """
        raw x/y/z 隨機：
        模擬 AI 視覺偵測到的 landing target 偏差每次不同。

        pitch 隨機：
        模擬風大、船體晃動、影像位置變化造成 AI 修正角度每次不同。
        """
        x = self.rng.uniform(-8.0, 8.0)
        y = self.rng.uniform(-8.0, 8.0)
        z = self.rng.uniform(2.0, 30.0)

        # pitch 範圍約 -4.6 度到 +4.6 度
        pitch = self.rng.uniform(-0.08, 0.08)

        # roll / yaw 也給一點變化
        roll = self.rng.uniform(-0.03, 0.03)
        yaw = self.rng.uniform(-0.05, 0.05)

        distance = math.sqrt(x * x + y * y + z * z)

        return roll, pitch, yaw, x, y, z, distance

    def send_ai(self, label: str, corr_conf: float, stat_conf: float, lost: int, reproj: float, cov: float):
        self.seq += 1

        self.heartbeat()
        self.poll_and_ack_commands()

        roll, pitch, yaw, x, y, z, distance = self.random_raw_and_pitch()

        # 52100：AI_LANDING_CORRECTION
        # 這是 AI -> 飛控的降落修正資料訊息
        self.m.mav.ai_landing_correction_send(
            now_ms(),
            float(roll),
            float(pitch),
            float(yaw),
            float(x),
            float(y),
            float(z),
            float(distance),
            float(corr_conf),
            200,   # frame = AI_LANDING_FRAME_CAMERA_FRD
            0b11,  # flags: bit0 yaw_valid, bit1 distance_valid
        )

        # 52102：AI_LANDING_STATUS
        # 這是 AI -> 飛控的視覺品質 / L1 L2 L3 判斷資料
        self.m.mav.ai_landing_status_send(
            now_ms(),
            float(stat_conf),
            int(lost),
            float(reproj),
            float(cov),
        )

        print(
            f"[SEND {self.seq:04d}] {label:14s} "
            f"52100 raw=({x:+.2f},{y:+.2f},{z:+.2f}) "
            f"pitch={pitch:+.4f}rad({math.degrees(pitch):+.2f}deg) "
            f"corr_conf={corr_conf:.2f} | "
            f"52102 stat_conf={stat_conf:.2f} lost={lost} reproj={reproj:.2f} cov={cov:.2f}"
        )

    def run_stage(
        self,
        label: str,
        duration_s: float,
        corr_conf: float,
        stat_conf: float,
        lost: int,
        reproj: float,
        cov: float,
    ):
        print("\n" + "=" * 90)
        print(f"[STAGE] {label} for {duration_s:.1f}s")
        print("=" * 90)

        end_t = time.time() + duration_s

        while time.time() < end_t:
            self.send_ai(label, corr_conf, stat_conf, lost, reproj, cov)
            time.sleep(self.dt)

    def run(self):
        print("\n[TEST PLAN]")
        print("  1. NORMAL：正常，應該 L1=0 L2=0 L3=0")
        print("  2. L1_WARNING：低信心但未達 L2，應該觸發 L1")
        print("  3. RECOVERY：恢復正常")
        print("  4. DIRECT_L2：直接觸發 L2，證明 L2 不一定要先經過 L1")
        print("  5. RECOVERY：給操作者重新 mode QLAND 的時間")
        print("  6. REPEAT_L2_L3：1 分鐘內第二次 L2，應觸發 L3 takeover")
        print("")

        # NORMAL：完全正常
        self.run_stage(
            label="NORMAL",
            duration_s=8,
            corr_conf=0.95,
            stat_conf=0.95,
            lost=0,
            reproj=0.01,
            cov=0.01,
        )

        # L1：低於 0.7，但高於 0.5，因此應該 L1，不應該 L2
        self.run_stage(
            label="L1_WARNING",
            duration_s=8,
            corr_conf=0.65,
            stat_conf=0.65,
            lost=0,
            reproj=0.12,
            cov=0.06,
        )

        # RECOVERY：恢復正常
        self.run_stage(
            label="RECOVERY",
            duration_s=6,
            corr_conf=0.95,
            stat_conf=0.95,
            lost=0,
            reproj=0.01,
            cov=0.01,
        )

        # DIRECT L2：直接送嚴重條件，不需要先 L1
        # lost=1 + conf<0.5 + reproj/cov 超過 L2 門檻
        self.run_stage(
            label="DIRECT_L2",
            duration_s=7,
            corr_conf=0.40,
            stat_conf=0.40,
            lost=1,
            reproj=0.35,
            cov=0.25,
        )

        print("\n" + "!" * 90)
        print("[OPERATOR ACTION REQUIRED]")
        print("如果 DIRECT_L2 後，MAVProxy 看到：")
        print("  AP: AI_LANDING_ABORT_L2 ...")
        print("  AP: AI_GO_AROUND ...")
        print("  AP: QLOITER ENTER")
        print("")
        print("這是正常的，因為 L2 會中止降落。")
        print("")
        print("要繼續測 L3，請在 Terminal 2 的 MAVProxy 手動輸入：")
        print("  mode QLAND")
        print("")
        print("原因：")
        print("  第一次 L2 會進 QLOITER。")
        print("  你重新 mode QLAND 後，下一次 L2 才能在降落流程中再次發生。")
        print("  1 分鐘內重複 L2，飛控才會觸發 L3 takeover。")
        print("!" * 90 + "\n")

        # 給操作者時間重新進 QLAND
        self.run_stage(
            label="RECOVERY",
            duration_s=10,
            corr_conf=0.95,
            stat_conf=0.95,
            lost=0,
            reproj=0.01,
            cov=0.01,
        )

        # REPEAT L2：1 分鐘內第二次 L2，應觸發 L3 takeover reason=5
        self.run_stage(
            label="REPEAT_L2_L3",
            duration_s=8,
            corr_conf=0.35,
            stat_conf=0.35,
            lost=1,
            reproj=0.40,
            cov=0.30,
        )

        print("\n" + "=" * 90)
        print("[DONE]")
        print("請在 MAVProxy 找以下證據：")
        print("  AP: AI_CORR conf=...                         代表 52100 有進來")
        print("  AP: AI_STAT conf=...                         代表 52102 有進來")
        print("  AP: QLAND AI pitch=...                       代表 pitch_err 被 QLAND 使用")
        print("  AP: AI_CAM_OFF raw=(...) corr=(...)          代表 raw 被 offset 補償成 corr")
        print("  AP: AI_CTRL_USE xyz=(...)                    代表控制使用 corr")
        print("  AP: AI_LANDING_L1_WARNING                    代表 L1 成功")
        print("  AP: AI_LANDING_ABORT_L2                      代表 L2 成功")
        print("  AP: AI_GO_AROUND                             代表 L2 後 go-around 成功")
        print("  AP: AI_LANDING_TAKEOVER reason=5             代表 L3 成功")
        print("=" * 90)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--master",
        default="tcp:127.0.0.1:5762",
        help="AI sender MAVLink connection. Default: tcp:127.0.0.1:5762",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=43143,
        help="Random seed. Same seed means repeatable random values.",
    )
    parser.add_argument(
        "--hz",
        type=float,
        default=5.0,
        help="Send rate. Default: 5Hz.",
    )

    args = parser.parse_args()

    tester = StoryFullTester(args.master, args.seed, args.hz)
    tester.connect()

    try:
        tester.run()
    except KeyboardInterrupt:
        print("\n[STOP] Ctrl+C")


if __name__ == "__main__":
    main()
