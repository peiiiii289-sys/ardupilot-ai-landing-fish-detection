import sys
import time
import socket

sys.path.insert(0, "/home/lucia/ardupilot/python_dialects")
import v20 as mavlink2


class SocketWriter:
    def __init__(self, sock):
        self.sock = sock

    def write(self, data):
        return self.sock.send(data)

    def flush(self):
        pass


sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(("127.0.0.1", 5762))

writer = SocketWriter(sock)

mav = mavlink2.MAVLink(writer)
mav.srcSystem = 42
mav.srcComponent = 211

start_time = time.monotonic()

print("=== FLAG TEST START ===")
print("0 = invalid all")
print("1 = yaw only")
print("2 = distance only")
print("3 = all valid")

while True:
    now = time.monotonic()
    elapsed = now - start_time
    time_boot_ms = int(elapsed * 1000)

    # ===== phase 切換（每 5 秒換一組 flags）=====
    if elapsed < 5:
        flags = 0
        phase = "flags=0 (invalid all)"

    elif elapsed < 10:
        flags = 1
        phase = "flags=1 (yaw only)"

    elif elapsed < 15:
        flags = 2
        phase = "flags=2 (distance only)"

    elif elapsed < 20:
        flags = 3
        phase = "flags=3 (all valid)"

    else:
        print("=== TEST END ===")
        break

    # ===== correction =====
    corr_msg = mavlink2.MAVLink_ai_landing_correction_message(
        time_boot_ms,
        0.01,     # roll_err
        -0.02,    # pitch_err
        0.03,     # yaw_err
        0.5,      # x_err
        -0.3,     # y_err
        1.2,      # z_err
        1.23,     # distance
        0.85,     # confidence
        200,      # frame
        flags
    )

    # ===== status（保持正常，避免干擾）=====
    status_msg = mavlink2.MAVLink_ai_landing_status_message(
        time_boot_ms,
        0.95,
        0,
        0.05,
        0.02
    )

    mav.send(corr_msg)
    mav.send(status_msg)

    print(
        f"{phase} | t={time_boot_ms} "
        f"flags={flags}"
    )

    time.sleep(0.1)