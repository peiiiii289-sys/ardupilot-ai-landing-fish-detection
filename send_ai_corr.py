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

start_time = time.time()

while True:
    time_boot_ms = int((time.time() - start_time) * 1000)

    msg = mavlink2.MAVLink_ai_landing_correction_message(
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

    mav.send(msg)
    print("sent", time_boot_ms)
    time.sleep(0.1)