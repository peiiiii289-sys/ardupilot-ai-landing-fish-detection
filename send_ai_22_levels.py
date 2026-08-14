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

while True:
    now = time.monotonic()
    elapsed = now - start_time
    time_boot_ms = int(elapsed * 1000)

    # ===== CORR（固定）=====
    corr_msg = mavlink2.MAVLink_ai_landing_correction_message(
        time_boot_ms,
        0.01,
        -0.02,
        0.03,
        0.5,
        -0.3,
        1.2,
        1.23,
        0.85,
        200,
        0b11
    )

    # ===== Phase 控制（重點在這）=====
    if elapsed < 3.0:
        phase = "normal_1"
        visual_confidence = 0.90
        target_lost = 0
        reproj_error = 0.05
        covariance = 0.02

    elif elapsed < 8.0:
        phase = "l2_1"
        visual_confidence = 0.40
        target_lost = 0
        reproj_error = 0.40
        covariance = 0.30

    elif elapsed < 15.0:
        phase = "normal_2"
        visual_confidence = 0.95   # 🔥 確保退出 L2
        target_lost = 0
        reproj_error = 0.05
        covariance = 0.02

    elif elapsed < 22.0:
        phase = "l2_2"
        visual_confidence = 0.40
        target_lost = 0
        reproj_error = 0.40
        covariance = 0.30

    else:
        phase = "hold_l2"
        visual_confidence = 0.40
        target_lost = 0
        reproj_error = 0.40
        covariance = 0.30

    status_msg = mavlink2.MAVLink_ai_landing_status_message(
        time_boot_ms,
        visual_confidence,
        target_lost,
        reproj_error,
        covariance
    )

    mav.send(corr_msg)
    mav.send(status_msg)

    print(
        f"{phase} | t={time_boot_ms} "
        f"conf={visual_confidence:.2f} "
        f"lost={target_lost} "
        f"reproj={reproj_error:.2f} "
        f"cov={covariance:.2f}"
    )

    time.sleep(0.1)