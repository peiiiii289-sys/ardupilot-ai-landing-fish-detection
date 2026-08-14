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
    time_boot_ms = int((time.monotonic() - start_time) * 1000)

    # CORR
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
        0b11      # flags
    )

    # STATUS
    status_msg = mavlink2.MAVLink_ai_landing_status_message(
        time_boot_ms,
        0.85,     # visual_confidence
        0,        # target_lost
        0.12,     # reproj_error
        0.03      # covariance
    )

    mav.send(corr_msg)
    mav.send(status_msg)

    print(f"sent corr+status {time_boot_ms}")

    time.sleep(0.1)   # 10 Hz