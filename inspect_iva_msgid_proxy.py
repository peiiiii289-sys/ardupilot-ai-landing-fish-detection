#!/usr/bin/env python3
# inspect_iva_msgid_proxy.py
# 用途：
#   檢查 Iva / AI 模組實際送進飛控的 MAVLink msgid。
#   這支程式是 TCP proxy：
#     Iva -> 本程式 listen port -> ArduPilot serial port
#   它會把封包照樣轉送給 ArduPilot，同時印出 AI -> FC 的 raw msgid。
#
# 使用範例 A：Iva 可以改 port
#   SITL 照舊 --serial1 tcp:5762
#   python3 inspect_iva_msgid_proxy.py --listen 127.0.0.1:5770 --forward 127.0.0.1:5762
#   Iva 改送 127.0.0.1:5770
#
# 使用範例 B：Iva 固定只能送 5762
#   SITL 改成 --serial1 tcp:5770
#   python3 inspect_iva_msgid_proxy.py --listen 127.0.0.1:5762 --forward 127.0.0.1:5770
#   Iva 維持送 127.0.0.1:5762
#
# 看到 msgid=52100 才是 AI_LANDING_CORRECTION。
# 看到 msgid=52102 是 AI_LANDING_STATUS。
# 看到 msgid=52101 是 AI_FISH_DETECTION_RESULT。

import argparse
import selectors
import socket
import time


MSG_NAMES = {
    52100: "AI_LANDING_CORRECTION",
    52101: "AI_FISH_DETECTION_RESULT",
    52102: "AI_LANDING_STATUS",
    31010: "MAV_CMD_START_AI_LANDING is a COMMAND_LONG command id, not a message id",
    31011: "MAV_CMD_STOP_AI_LANDING is a COMMAND_LONG command id, not a message id",
    31012: "MAV_CMD_START_AI_VISION is a COMMAND_LONG command id, not a message id",
    31013: "MAV_CMD_STOP_AI_VISION is a COMMAND_LONG command id, not a message id",
}


class RawMavlinkInspector:
    def __init__(self):
        self.buf = bytearray()
        self.count = 0

    def feed(self, data: bytes):
        self.buf.extend(data)
        self._parse()

    def _parse(self):
        while True:
            # 找 magic byte: MAVLink1=0xFE, MAVLink2=0xFD
            start = None
            for i, b in enumerate(self.buf):
                if b in (0xFD, 0xFE):
                    start = i
                    break

            if start is None:
                self.buf.clear()
                return

            if start > 0:
                del self.buf[:start]

            if len(self.buf) < 2:
                return

            magic = self.buf[0]
            payload_len = self.buf[1]

            if magic == 0xFD:
                # MAVLink2:
                # magic(1), len(1), incompat(1), compat(1), seq(1), sysid(1), compid(1), msgid(3)
                if len(self.buf) < 10:
                    return

                incompat_flags = self.buf[2]
                seq = self.buf[4]
                sysid = self.buf[5]
                compid = self.buf[6]
                msgid = self.buf[7] | (self.buf[8] << 8) | (self.buf[9] << 16)

                signature_len = 13 if (incompat_flags & 0x01) else 0
                frame_len = 10 + payload_len + 2 + signature_len

                if len(self.buf) < frame_len:
                    return

                frame = bytes(self.buf[:frame_len])
                del self.buf[:frame_len]
                self._print_frame("MAVLink2", msgid, sysid, compid, payload_len, seq, frame)
                continue

            if magic == 0xFE:
                # MAVLink1:
                # magic(1), len(1), seq(1), sysid(1), compid(1), msgid(1)
                if len(self.buf) < 6:
                    return

                seq = self.buf[2]
                sysid = self.buf[3]
                compid = self.buf[4]
                msgid = self.buf[5]
                frame_len = 6 + payload_len + 2

                if len(self.buf) < frame_len:
                    return

                frame = bytes(self.buf[:frame_len])
                del self.buf[:frame_len]
                self._print_frame("MAVLink1", msgid, sysid, compid, payload_len, seq, frame)
                continue

    def _print_frame(self, version, msgid, sysid, compid, payload_len, seq, frame):
        self.count += 1
        name = MSG_NAMES.get(msgid, "UNKNOWN_OR_STANDARD")
        mark = ""
        if msgid == 52100:
            mark = "  <<< OK: CORRECTION"
        elif msgid in (52101, 52102):
            mark = "  <<< custom but not correction"
        elif msgid not in (52100, 52101, 52102):
            mark = "  <<< CHECK: not 52100"

        print(
            f"[AI->FC #{self.count:05d}] {version} "
            f"msgid={msgid} name={name} sysid={sysid} compid={compid} "
            f"len={payload_len} seq={seq}{mark}",
            flush=True,
        )


def parse_host_port(value: str):
    if ":" not in value:
        raise argparse.ArgumentTypeError("format must be host:port, example 127.0.0.1:5770")
    host, port_s = value.rsplit(":", 1)
    return host, int(port_s)


def make_server(host, port):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(1)
    return srv


def make_client(host, port):
    while True:
        try:
            s = socket.create_connection((host, port), timeout=3)
            s.settimeout(None)
            return s
        except OSError as e:
            print(f"[WAIT] cannot connect to forward {host}:{port}: {e}", flush=True)
            time.sleep(1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--listen", type=parse_host_port, default=("127.0.0.1", 5770),
                    help="proxy listen address, default 127.0.0.1:5770")
    ap.add_argument("--forward", type=parse_host_port, default=("127.0.0.1", 5762),
                    help="ArduPilot serial tcp address, default 127.0.0.1:5762")
    args = ap.parse_args()

    listen_host, listen_port = args.listen
    forward_host, forward_port = args.forward

    print("[START] Iva MAVLink msgid inspector proxy", flush=True)
    print(f"[LISTEN] Iva should connect to {listen_host}:{listen_port}", flush=True)
    print(f"[FORWARD] proxy will connect to ArduPilot at {forward_host}:{forward_port}", flush=True)

    srv = make_server(listen_host, listen_port)
    print("[WAIT] waiting Iva / AI client...", flush=True)
    ai_sock, ai_addr = srv.accept()
    print(f"[OK] Iva connected from {ai_addr}", flush=True)

    fc_sock = make_client(forward_host, forward_port)
    print("[OK] connected to ArduPilot", flush=True)

    ai_sock.setblocking(False)
    fc_sock.setblocking(False)

    sel = selectors.DefaultSelector()
    sel.register(ai_sock, selectors.EVENT_READ, "ai")
    sel.register(fc_sock, selectors.EVENT_READ, "fc")

    inspector = RawMavlinkInspector()

    try:
        while True:
            for key, _ in sel.select(timeout=1):
                sock = key.fileobj
                side = key.data
                try:
                    data = sock.recv(4096)
                except ConnectionResetError:
                    data = b""

                if not data:
                    print(f"[CLOSE] {side} disconnected", flush=True)
                    return

                if side == "ai":
                    inspector.feed(data)
                    fc_sock.sendall(data)
                else:
                    ai_sock.sendall(data)

    except KeyboardInterrupt:
        print("\n[STOP] Ctrl+C", flush=True)
    finally:
        try:
            ai_sock.close()
        except Exception:
            pass
        try:
            fc_sock.close()
        except Exception:
            pass
        try:
            srv.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
