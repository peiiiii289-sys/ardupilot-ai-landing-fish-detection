import sys
import time
import socket

# 使用你自己的 custom dialect
sys.path.insert(0, "/home/lucia/ardupilot/python_dialects")
import v20 as mavlink2


class SocketWriter:
    def __init__(self, sock):
        self.sock = sock

    def write(self, data):
        return self.sock.send(data)

    def flush(self):
        pass


# 連線到 SITL SERIAL1（跟 CORR 一樣）
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(("127.0.0.1", 5762))

writer = SocketWriter(sock)

mav = mavlink2.MAVLink(writer)
mav.srcSystem = 42
mav.srcComponent = 211

# 用 monotonic 當時間來源（避免 overflow）
start_time = time.monotonic()

while True:
    time_boot_ms = int((time.monotonic() - start_time) * 1000)

    # ===== 這裡是你可以調整的測試模式 =====
    visual_confidence = 0.85   # 信心值
    target_lost = 0            # 0 = 沒丟目標
    reproj_error = 0.12        # 重投影誤差
    covariance = 0.03          # 協方差（穩定度）

    msg = mavlink2.MAVLink_ai_landing_status_message(
        time_boot_ms,
        visual_confidence,
        target_lost,
        reproj_error,
        covariance
    )

    mav.send(msg)
    print("sending:", msg.to_dict())
    print("AI_LANDING_STATUS sent", time_boot_ms)

    time.sleep(0.1)  # 🔥 10Hz（跟 CORR 一樣重要）