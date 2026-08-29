#!/usr/bin/env python3
"""
UAV-Mini UDP console + PID tuner.

ESP32 lắng nghe UDP tại <ESP32_IP>:4210.
Script này gửi gói keepalive đầu tiên để ESP32 học peer IP:port.
Sau đó:
  - Nhận log/telemetry/ACK từ ESP32.
  - Gửi lệnh arm/kill/status/help.
  - Gửi lệnh PID dạng @PID ...

Mặc định mở GUI (kéo slider hoặc gõ số để tune PID trực tiếp khi đang bay).
Dùng --cli để quay lại console dạng gõ lệnh text như cũ.

Cách dùng:
    python tools/uav_udp_console.py                       # GUI, nhập IP trong app
    python tools/uav_udp_console.py 192.168.1.19           # GUI, tự connect luôn
    python tools/uav_udp_console.py 192.168.1.19 --cli     # console text (như cũ)
    python tools/uav_udp_console.py 192.168.1.19 --debug

Lệnh console text (--cli):
    r / arm
    k / kill / disarm
    s / status
    h / help
    + / -
    0 / stop

    pid get
    pid set rate roll 3.5 0 0.02 60 260

Thoát (--cli):
    quit
    exit
"""

import argparse
import queue
import re
import socket
import sys
import threading
import time
from collections import deque
from typing import Callable, Optional

try:
    import tkinter as tk
    from tkinter import messagebox, scrolledtext, ttk
    _TK_AVAILABLE = True
except ImportError:
    _TK_AVAILABLE = False


DEFAULT_PORT = 4210
KEEPALIVE_INTERVAL_S = 5.0
RECV_BUF_SIZE = 4096


SHORTHAND_COMMANDS = {
    # Vào/thoát chế độ bay-cân-bằng (bản maintenance). Bản bay không có 'f'/'q'.
    "flight": "f", "f": "f",
    "exit": "q", "quit": "q", "q": "q",

    # ARM giờ là 'r' (trước kia 'f' vì flight-balance cũ tự arm). 'k'=KILL
    # (bypass FSM, cắt ngay). 'd'=DISARM THẬT (chỉ hợp lệ từ ARMED, xem
    # command_parser.c case 'd', MỚI) — TRƯỚC ĐÂY "disarm" trỏ nhầm sang 'k'
    # (chưa có 'd' riêng lúc đó), ĐÃ SỬA.
    "arm": "r", "r": "r",
    "kill": "k", "k": "k",
    "disarm": "d", "d": "d",

    # 'p'=heartbeat Commander watchdog (xem _start_cmdr_heartbeat() — console
    # đã tự gửi định kỳ, gõ tay chỉ để test/xác nhận thủ công).
    "ping": "p", "heartbeat": "p", "p": "p",

    "hold": "z", "z": "z",       # ALT HOLD toggle
    "log": "x", "x": "x",        # ALT LOG_ONLY toggle
    "takeoff": "t", "t": "t",    # TAKEOFF tu dong len alt target
    "land": "l", "landing": "l", "l": "l",   # LANDING tu dong ha + disarm

    "status": "s", "s": "s",
    "help": "h", "h": "h", "?": "h",

    "+": "+", "throttle+": "+",
    "-": "-", "throttle-": "-",
    "]": "]", "[": "[",
    ">": ">", "<": "<",

    "0": "0", "stop": "0",
}


class UavUdpConsole:
    def __init__(
        self,
        host: str,
        port: int,
        bind_port: "int | None" = None,
        debug: bool = False,
        line_callback: Optional[Callable[[str], None]] = None,
    ):
        self.addr = (host, port)
        self.debug = debug
        # When set, every received line is handed to this callback instead of
        # being printed directly (used by the GUI to stay off the socket thread).
        self.line_callback = line_callback

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Cho phép bind local port cố định nếu cần debug firewall/NAT.
        # Bình thường không cần bind, Windows sẽ tự cấp source port.
        if bind_port is not None:
            self.sock.bind(("0.0.0.0", bind_port))

        self.sock.settimeout(0.2)

        self._stop = threading.Event()
        self._rx_thread: threading.Thread | None = None
        self._keepalive_thread: threading.Thread | None = None

    def start(self):
        # Gửi gói đầu tiên để Windows cấp local port và ESP32 học peer.
        self._send_raw(b"\n")

        print(f"[PC] local UDP socket: {self.sock.getsockname()}")
        print(f"[PC] target ESP32     : {self.addr[0]}:{self.addr[1]}")

        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        self._keepalive_thread = threading.Thread(target=self._keepalive_loop, daemon=True)
        self._keepalive_thread.start()

    def stop(self):
        self._stop.set()

        try:
            self.sock.close()
        except OSError:
            pass

    def _send_raw(self, data: bytes):
        try:
            sent = self.sock.sendto(data, self.addr)
            if self.debug:
                print(f"[PC -> ESP32] {sent} bytes: {data!r}")
        except OSError as exc:
            print(f"[send error] {exc}", file=sys.stderr)

    def send_command(self, command: str):
        # Firmware parser thường cần newline để kết thúc lệnh nhiều ký tự.
        payload = (command + "\n").encode("utf-8")
        self._send_raw(payload)

    def _rx_loop(self):
        while not self._stop.is_set():
            try:
                data, from_addr = self.sock.recvfrom(RECV_BUF_SIZE)
            except socket.timeout:
                continue
            except OSError:
                break

            if not data:
                continue

            text = data.decode("utf-8", errors="replace")

            # Có thể ESP32 gửi nhiều dòng trong một UDP packet.
            lines = text.splitlines()
            if not lines:
                lines = [text]

            for line in lines:
                if self.line_callback is not None:
                    self.line_callback(line)
                else:
                    print(f"{line}")

    def _keepalive_loop(self):
        while not self._stop.wait(KEEPALIVE_INTERVAL_S):
            self._send_raw(b"\n")


def translate_command(raw: str) -> str:
    stripped = raw.strip()
    lowered = stripped.lower()

    if lowered in SHORTHAND_COMMANDS:
        return SHORTHAND_COMMANDS[lowered]

    # Gõ: pid get
    # Gửi: @pid get
    # Firmware sẽ uppercase thành @PID GET nếu code parser của m làm vậy.
    if lowered.startswith("pid"):
        return "@" + stripped

    return stripped


# ============================================================
# CLI MODE (console dạng gõ lệnh text, giữ nguyên hành vi cũ)
# ============================================================

def run_cli(args):
    console = UavUdpConsole(
        host=args.host,
        port=args.port,
        bind_port=args.bind_port,
        debug=args.debug,
    )

    console.start()

    print("")
    print(f"Đã mở UDP tới {args.host}:{args.port}.")
    print("Gõ 'help' xem lệnh firmware, 'quit' để thoát.")
    print("Ví dụ tune PID:")
    print("  pid get")
    print("  pid set rate roll 3.5 0 0.02 60 260")
    print("")

    try:
        while True:
            try:
                raw = input()
            except EOFError:
                break

            if raw.strip().lower() in ("quit", "exit"):
                break

            if not raw.strip():
                continue

            command = translate_command(raw)
            console.send_command(command)

            # Nhường chút thời gian để RX thread in ACK/log ngay sau command.
            time.sleep(0.01)

    except KeyboardInterrupt:
        pass
    finally:
        console.stop()
        print("Đã ngắt kết nối.")


# ============================================================
# GUI MODE (kéo slider hoặc gõ số để tune PID trực tiếp)
# ============================================================

PID_LOOPS = ("ANGLE", "RATE")
PID_AXES = ("ROLL", "PITCH", "YAW")
PID_FIELDS = ("kp", "ki", "kd", "ilimit", "outlimit")
# Chỉ hiện kp/ki/kd trên UI cho gọn; ilimit/outlimit vẫn được firmware yêu cầu (7
# token) nên GUI giữ ngầm (lấy từ @PID GET) rồi gửi kèm khi SET — xem GainGroup.
PID_VISIBLE_FIELDS = ("kp", "ki", "kd")

FIELD_RANGE = {
    "kp": (0.0, 20.0),
    "ki": (0.0, 10.0),
    "kd": (0.0, 5.0),
    "ilimit": (0.0, 1000.0),
    "outlimit": (0.0, 1000.0),
}

# Matches both the "PID GET" dump lines and "PID OK ..." set-confirmations:
#   PID ANGLE ROLL 0.1000 0.0000 0.0000 30.0000 30.0000
#   PID OK RATE PITCH 3.5000 0.0000 0.0200 60.0000 260.0000
PID_DUMP_RE = re.compile(
    r"^PID (?:OK )?(ANGLE|RATE) (ROLL|PITCH|YAW) "
    r"([-\d.]+) ([-\d.]+) ([-\d.]+) ([-\d.]+) ([-\d.]+)\s*$"
)
PID_ERR_RE = re.compile(r"^PID ERR (.*)$")

# ARM=1 THR=200 | R=0.12 P=-0.34 Y=1.20 | G=0.10 0.20 0.30 | valid=1 | ACC=1.002 ACCU=1
# ACC = |accel| in g (1.0 = pure gravity); ACCU = 0 means the EKF gated this
# sample out (vibration / non-gravity accel) and skipped the tilt correction.
# Dùng để phân biệt "pitch tăng that vi rung dong co" voi "pitch tang vi
# khung/prop mat can bang" khi tang throttle.
STATUS_RE = re.compile(
    r"^ARM=(\d) THR=(-?\d+) \| R=([-\d.]+) P=([-\d.]+) Y=([-\d.]+) \| "
    r"G=([-\d.]+) ([-\d.]+) ([-\d.]+) \| "
    # Cụm accel từng trục "A=ax ay az |" và duty 4 động cơ "M = m1 m2 m3 m4 |".
    # Đều non-capturing để KHÔNG dịch chỉ số group phía sau (A do ACC_PLOT_RE bắt
    # riêng cho đồ thị; M chỉ hiện trong log text).
    r"(?:A=[-\d.]+ [-\d.]+ [-\d.]+ \| )?"
    r"(?:M\s*=\s*-?\d+ -?\d+ -?\d+ -?\d+ \| )?"
    r"(?:valid|Val)=(\d)"          # firmware cũ 'valid=', mới 'Val='
    r"(?: \| ACC=([-\d.]+) ACCU=(\d))?"
    r"(?: \| YAWREL=([-\d.]+))?"
    # Cụm altitude của BẢN BAY (flight_control 3 tầng), append vào cuối dòng STATUS.
    # Optional để dòng STATUS cũ (không có altitude) vẫn khớp.
    r"(?: \| ALTm=([-\d.]+) VZ=([-\d.]+) TGT=([-\d.]+) MODE=(\d) AV=(\d) "
    r"TOF=([-\d.]+) TOK=(\d) TERR=(\d+)(?: TKO=(\d))?(?: KI=(\d))?(?: LAND=(\d))?)?"
    # TKO=pha takeoff, KI=I-term đang dùng, LAND=pha landing
    #
    # Đuôi mở rộng UAV-S3 (ground-station WiFi/UDP, xem src/telemetry_format.c)
    # — HOÀN TOÀN optional để log/firmware CŨ (không có đuôi này) vẫn khớp.
    # MAGOK/BAROOK/IMUOK = driver cảm biến còn sống; BALT = baro altitude (m);
    # DT = loop dt thực (ms, jitter); CMDAGE/HBAGE = tuổi lệnh setpoint/heartbeat
    # gần nhất (ms); FAULT = Commander fault_class_t gần nhất (0=none 1=soft 2=hard).
    r"(?: MAGOK=(\d) BAROOK=(\d) IMUOK=(\d) BALT=([-\d.]+) DT=([-\d.]+) "
    r"CMDAGE=(-?\d+) HBAGE=(-?\d+) FAULT=(\d))?"
    # BATV=điện áp pin (V), BCOMP=hệ số bù throttle theo pin ĐANG áp dụng
    # (>=1.0, xem tuning.h mục 10) — optional, None nếu firmware cũ hơn.
    r"(?: BATV=([-\d.]+) BCOMP=([-\d.]+))?"
    # BFILT=baro SAU median-of-3+LPF, BINNOV=|BFILT-ALTm| lần gần nhất,
    # BACC/BREJ=số mẫu baro đã nhận/đã bỏ (innovation gate, xem alt_estimator.h),
    # BCAL/BHLT=đã calibrate_ground()/còn healthy (std-dev), BSTD=std-dev áp
    # suất nền (Pa), VACC=vertical accel world-frame đã lọc (m/s^2) — optional.
    r"(?: BFILT=([-\d.]+) BINNOV=([-\d.]+) BACC=(\d+) BREJ=(\d+) "
    r"BCAL=(\d) BHLT=(\d) BSTD=([-\d.]+) VACC=([-\d.]+))?"
    # Đuôi accel-primary altitude estimator (xem alt_estimator.h) — optional,
    # None nếu firmware cũ hơn (trước refactor accel-primary). BDT=dt thật
    # giữa 2 mẫu baro (s), VACCR=accel world-frame TRƯỚC LPF (raw), ABIAS=bias
    # gia tốc đã học (state gamma, xem alt_estimator.h), VZAO/ZINE=Vz/Z tích
    # phân THUẦN accel (baro không chạm — so sánh với VZ/ALTm để thấy baro
    # đang kéo bao nhiêu), VZTGT=vz_target tầng ngoài cascade HOLD.
    r"(?: BDT=([-\d.]+) VACCR=([-\d.]+) ABIAS=([-\d.]+) VZAO=([-\d.]+) "
    r"ZINE=([-\d.]+) VZTGT=([-\d.]+))?"
    # Đuôi refactor an toàn/realtime (xem components/flight_core/include/
    # flight_core/telemetry.h) — optional, None nếu firmware cũ hơn.
    #   KILL = kill latch đang đóng (motor cắt cứng, cần ARM lại)
    #   TKOP = pha cất cánh (0=IDLE 1=SPOOL 2=LIFTOFF_CONFIRM 3=CLIMB 4=SETTLE)
    #   LSC  = điểm bằng chứng liftoff (0..4) — thấy vì sao detector chưa chốt
    #   LDT/LMX = dt vòng điều khiển hiện tại / đỉnh từ boot (us)
    #   DLM  = số tick trượt deadline (cộng dồn)
    #   IAGE/MAGE/BAGE = tuổi mẫu IMU/mag/baro (ms, -1 = chưa từng có mẫu)
    #   HDEG = heading_degraded (yaw chỉ còn gyro, roll/pitch VẪN tốt)
    #   MSAT = mixer bão hoà; MRPY = 3 cờ trục roll/pitch/yaw bị chặn
    #   MHR  = headroom duty còn lại cho attitude
    #   BTHR = collective TRƯỚC bù pin (THR ở đầu dòng là SAU bù)
    r"(?: KILL=(\d) TKOP=(\d) LSC=(\d) LDT=(-?\d+) LMX=(-?\d+) DLM=(\d+))?"
    r"(?: IAGE=(-?\d+) MAGE=(-?\d+) BAGE=(-?\d+) HDEG=(\d))?"
    r"(?: MSAT=(\d) MRPY=(\d)(\d)(\d) MHR=([-\d.]+) BTHR=(-?\d+))?"
    # Đuôi estimator GROUND/CANDIDATE/AIRBORNE + reacquire (alt_estimator.h) —
    # optional, None nếu firmware cũ hơn.
    #   AIRB   = đã confirmed rời đất chưa
    #   CAND   = pha LIFTOFF_CANDIDATE (inertial chạy để TẠO bằng chứng liftoff)
    #            AIRB=0 CAND=0 -> Z/Vz đang khoá 0 CÓ CHỦ ĐÍCH, không phải lỗi
    #   AZCORR = az sau trừ bias + deadband (m/s²) — giá trị THẬT được tích phân
    #   BCREJ  = reject baro LIÊN TỤC hiện tại (đứng lâu dù baro healthy = bất thường)
    #   BREACQ = đang REACQUIRE (đang kéo estimator về lại baro)
    #   BSEQ   = seq mẫu baro (phải tăng ~50/s nếu BMP280 còn sống)
    #   BFI    = baro fusion đã khởi tạo gốc toạ độ chưa
    r"(?: AIRB=(\d) CAND=(\d) AZCORR=([-\d.]+) BCREJ=(\d+) BREACQ=(\d) BSEQ=(\d+) BFI=(\d))?"
    #   ALTSRC = NGUON dang giu Z: 0=NONE 1=TOF 2=BARO 3=TOF+BARO.
    # Nhom RIENG va optional: firmware cu khong co truong nay, va neu gop
    # chung vao nhom AIRB o tren thi firmware cu se lam CA nhom do khong khop
    # -> mat luon AIRB/CAND/AZCORR/BCREJ/... ma khong bao loi gi.
    r"(?: ALTSRC=(\d+))?"
    # Battery debug (battery_driver.h) — truy ngược thang đo khi BATV sai:
    #   BATRAW/BATMV = ADC thô / mV tại chân (sau adc_cali nếu có)
    #   BATRATIO     = hệ số chia áp đang biên dịch vào firmware (phải = 3.20)
    #   BATVRAW      = mV/1000*ratio TRƯỚC sanity check (LUÔN có số, kể cả sai)
    #   BATVALID     = 0 -> BATV đầu dòng đã bị ép 0, KHÔNG dùng cho failsafe
    #   BATCALI      = 0 -> thiếu ADC calibration, mọi mẫu tự động invalid
    r"(?: BATRAW=(-?\d+) BATMV=(-?\d+) BATRATIO=([-\d.]+) BATVRAW=([-\d.]+) "
    r"BATVALID=(\d) BATCALI=(\d) BATAGE=(-?\d+))?"
    # Đuôi TAKEOFF closed-loop (takeoff_land.h) — optional, None nếu firmware
    # cũ hơn refactor này.
    #   TKOACT  = altitude dynamics + Z/Vz controller ĐANG chạy.
    #             ⚠ KHÁC AIRB: TKOACT=1 AIRB=0 là trạng thái HỢP LỆ và mong đợi
    #             — đó là cửa sổ controller đang nhấc drone lên khỏi mặt đất.
    #   TKOTGT  = độ cao mục tiêu TỪ LỆNH (m) — sẽ leo tới rồi HOLD ở đó
    #   ZSP/VZSP= setpoint quỹ đạo (m / m/s), velocity-limited ramp
    #   TKOBASE = hover baseline; TKOCORR = phần Vz-PID cộng thêm (duty)
    #   LSCORE  = điểm liftoff THẬT 0..4 (LSC ở đuôi cũ nay luôn 0, đã bỏ)
    #   EVT/EVA/EVV/EVZ = 4 bằng chứng: throttle / accel sustained / Vz / Z rise
    #   TKOI    = RIÊNG phần I của vòng Vz = hover THẬT đang được HỌC. Khởi đầu
    #             ÂM (prime_duty < hover_ff) rồi bò lên và HỘI TỤ.
    #   TKOGND  = mốc est_z lúc rời PRIME; TKOLIFT = đã rời đất (THÔNG TIN)
    #   TKOTILT/TKOEL = tilt (deg) + thời gian đã trôi của chuỗi (s)
    #   TKOAB   = lý do abort (0=NONE 1=TIMEOUT 2=TOF_LOST 3=TILT)
    #   ALTSAT  = cascade Vz đang bị kẹp trần/sàn duty
    r"(?: TKOACT=(\d) TKOTGT=([-\d.]+) ZSP=([-\d.]+) VZTGT=([-\d.]+) "
    r"TKOBASE=([-\d.]+) TKOCORR=([-\d.]+) TKOI=([-\d.]+) TKOGND=([-\d.]+) "
    r"TKOLIFT=(\d) TKOTILT=([-\d.]+) TKOEL=([-\d.]+) TKOAB=(\d) ALTSAT=(\d))?"
    # ARMREJ = VÌ SAO lệnh ARM gần nhất bị từ chối (arm_reject_t trong
    # telemetry.h); ARMRSEQ = số lần bị từ chối (đếm lên mỗi lần bấm mà trượt).
    # Đây là đường DUY NHẤT để GUI biết lý do: mọi log 'ARM tu choi' của firmware
    # chỉ ra console USB, KHÔNG đi qua UDP.
    r"(?: ARMREJ=(\d+) ARMRSEQ=(\d+))?"
    # TKOREJ = VI SAO lenh TAKEOFF gan nhat KHONG khoi dong duoc chuoi cat canh
    # (takeoff_reject_t); TKORSEQ = so lan bi tu choi.
    # ⚠ KHAC TKOAB= o tren: TKOAB la "da cat canh roi moi phai huy giua chung",
    # con TKOREJ la "chua tung bat dau". Truoc day nhanh nay HOAN TOAN im lang
    # voi GUI, va command_parser con tra loi "TAKEOFF started" theo DU DOAN chi
    # xet FSM state -> GUI bao thanh cong trong khi firmware da tu choi.
    r"(?: TKOREJ=(\d+) TKORSEQ=(\d+))?"
    # Duoi ToF surface-gated correction (alt_estimator.h).
    #   TOFV    = range da bu nghieng (m)
    #   TOFINN  = innovation so voi MAT SAN DA KHOA. Am lon = co be mat CAO hon
    #             san o duoi (ban/ghe/buc).
    #   TOFSURF = world-Z cua be mat DANG nhin thay
    #   TOFST   = 0 UNKNOWN / 1 FLOOR / 2 OTHER
    #   TOFCOR  = ToF co DANG duoc sua world-Z khong
    #   TOFGR   = range luc UAV nam tren san (chot o ARM)
    #   FLOORZ  = mat san khoa luc cat canh (KHONG doi giua chuyen bay)
    #   LANDZ   = be mat chon khi CMD_LAND (KHAC FLOORZ!)
    # ⚠ TOFST=2 voi TOFCOR=0 la HOP LE va MONG DOI khi bay qua ban.
    r"(?: TOFV=([-\d.]+) TOFINN=([-\d.]+) TOFSURF=([-\d.]+) TOFST=(\d) "
    r"TOFCOR=(\d) TOFGR=([-\d.]+) TOFA=(\d+) TOFR=(\d+) "
    r"FLOORZ=([-\d.]+) LANDZ=([-\d.]+) LANDV=(\d) TOFAGE=(-?\d+))?"
    #   TOFEN = ToF co trong firmware nay khong (1) hay bi TAT luc bien dich (0).
    # Phan biet "ToF TAT co chu dich" voi "ToF CO nhung hong" -- hai thu ma
    # TOFAGE=-1 mot minh KHONG tach duoc, khien GUI bao do nham o cau hinh
    # baro-only hoan toan hop le.
    # Nhom nay o CUOI regex -> index 107, KHONG dich chi so nao phia truoc.
    r"(?: TOFEN=(\d))?"
    #   TOFCORRZ/TOFCORRVZ/BAROCORRZ/BAROCORRVZ = luong DA SUA cua lan
    #     correction gan nhat (KHAC innovation = hai ben bat dong bao nhieu).
    #     Tach ra moi tune duoc gain: innovation lon + correction nho = gain
    #     thap dung thiet ke; innovation nho + correction lon = gain qua tay.
    #   BIASRES = residual da loc dang lai accel bias Z
    #   BIASADP = so tick THAT SU adapt bias (phai TANG DEU khi bay bang)
    # Nhom o CUOI regex -> khong dich chi so nao phia truoc.
    r"(?: TOFCORRZ=([-\d.]+) TOFCORRVZ=([-\d.]+) "
    r"BAROCORRZ=([-\d.]+) BAROCORRVZ=([-\d.]+) "
    r"BIASRES=([-\d.]+) BIASADP=(\d+))?"
    #   HOVLK = da CHOT ga hover theo pin chua (0 = dang chay hang so cu)
    #   HOVLV = vbat trung vi luc chot (V, do khi motor CHUA quay -> KHONG TAI)
    #   HOVLD = hover_ff suy ra tu model (duty)
    # Day la hai so can nhin DAU TIEN khi drone "nam i khong nhac": HOVLD lech
    # nhieu so voi hover THAT = model sai (doi motor/canh/pin ma chua do lai).
    # Nhom o CUOI regex -> khong dich chi so nao phia truoc.
    r"(?: HOVLK=(\d) HOVLV=([-\d.]+) HOVLD=([-\d.]+))?"
    # TOFALIVE = bao lau roi CHIP khong do duoc (ms), -1 = chua tung do duoc.
    # KHAC HAN TOFAGE (tuoi mau HOP LE). Nam sat san / ngoai tam: TOFAGE tang
    # vo han nhung TOFALIVE van nho -> 'khong co gi de do', KHONG phai loi.
    # Ca hai cung tang -> that su mat cam bien.
    r"(?: TOFALIVE=(-?\d+))?"
    # ---- DUOI TU DO: BO QUA moi truong la o cuoi dong ----
    #
    # TRUOC DAY cho nay la r"\s*$" — bat buoc dong STATUS phai KET THUC ngay
    # sau nhom cuoi cung ma regex biet. Hau qua: firmware them BAT KY khoi
    # telemetry moi nao o duoi (TOFZ/FLOORLOCK/AZBZ/VZP/CLR/FRAME...) la CA
    # DONG khong khop -> GUI bo qua toan bo -> mat luon nhung thu co san tu
    # dau nhu M= (duty 4 dong co) va ALTm= (do cao). Trieu chung nhin thay la
    # "GUI khong hien throttle/do cao nua", trong khi firmware van gui du.
    #
    # Regex nay la GIAO DIEN giua hai ban build co the lech phien ban nhau, nen
    # no phai BAO DUNG (tolerant): doc nhung gi minh hieu, lo phan con lai.
    # Them truong moi vao firmware KHONG duoc phep lam vo GUI cu.
    r"(?:\s.*)?$"
)

# alt_est_tof_surface_t (alt_estimator.h) — thu tu PHAI khop enum firmware.
TOF_SURFACE_NAMES = {"0": "UNKNOWN", "1": "FLOOR", "2": "OTHER"}

# arm_reject_t (components/flight_core/include/flight_core/telemetry.h) — thứ tự
# PHẢI khớp enum bên firmware. Mỗi dòng nói luôn CÁCH SỬA, vì đây là thứ người
# dùng đọc đúng lúc đang bế tắc không arm được.
ARM_REJECT_NAMES = {
    "0":  "",
    "1":  "CHUA CALIBRATE (thieu accel trong NVS) -> calib_accel_face x6 + calib_gyro (mag KHONG bat buoc)",
    "2":  "FSM KHONG o DISARMED -> bam KILL (k) roi ARM lai",
    "3":  "attitude CHUA hop le (Mahony chua init) -> de yen drone vai giay",
    "4":  "NGHIENG qua nguong -> dat drone bang phang",
    "5":  "IMU stale/khong khoe -> kiem tra bus I2C + day INT",
    "6":  "fresh gyro calib CHUA PASS -> giu yen drone, chay calib_gyro va xem cal_status",
    "7":  "accel CHUA calib 6-face -> chay calib_accel_face x6",
    "8":  "KHONG doc duoc dien ap pin (ADC loi?)",
    "9":  "PIN DUOI SAN -> sac truoc khi bay",
    "10": "baro CHUA co moc 0m -> chay calib_baro_ground",
    "11": "baro UNHEALTHY (nhieu ap suat lon luc calib) -> tranh gio/quat, calib lai",
    "12": "alt_estimator chua hop le (chua co mau baro moi)",
    "13": "vong dieu khien TRE HAN lien tuc -> timing chua on",
    "14": "khong muon duoc bus I2C de calib baro luc ARM",
    "15": "calib baro luc ARM THAT BAI -> khong co moc 0m tin cay",
    "16": ("HEARTBEAT da cu (>1000ms) luc bam ARM -> nguon dieu khien NGOAI chua ping. "
           "GUI phai DANG KET NOI (no tu gui 'p' moi 400ms); neu bay bang console USB "
           "thi phai go 'heartbeat' moi <1s. Firmware KHONG tu sinh heartbeat."),
    "17": ("chua du mau pin de CHOT ga hover -> cho ~0.5s roi ARM lai "
           "(ADC pin chay 10Hz, can 3 mau). KHONG tu het = ADC pin hong that."),
    "18": ("PIN QUA THAP de chot ga hover (<3.4V) -> SAC PIN. Khac 'PIN DUOI SAN': "
           "day la bien VUNG HOP LE CUA MODEL hover, duoi nguong nay ga hover "
           "tinh ra vuot tran collective nen bi kep thap hon hover THAT -> "
           "drone se nam i khong nhac noi."),
    "19": "MPU6050 read-back config KHONG hop le -> xem GYRO_CONFIG/ACCEL_CONFIG trong cal_status",
    "20": "gyro corrected mean con lech -> chay lai calib_gyro khi drone dung yen",
    "21": "chua co cua so gyro stationary 1s ngay truoc ARM -> giu yen drone roi ARM lai",
}

# takeoff_reject_t (telemetry.h) — thứ tự PHẢI khớp enum firmware. Cùng lý do
# tồn tại như ARM_REJECT_NAMES: mỗi dòng nói luôn CÁCH SỬA.
TAKEOFF_REJECT_NAMES = {
    "0": "",
    "1": "FSM KHONG o ARMED -> bam ARM truoc (dang bay thi dung LAND/KILL)",
    # Ma 2 gio CHI con nghia "khong thay chip ToF tren I2C luc boot".
    # TRUOC DAY no con bao gom "chua chot duoc mau floor" -- ma nam sat san thi
    # ToF doc 0.000m nen KHONG BAO GIO chot duoc -> moi lenh TAKEOFF bi tu choi
    # vinh vien, khong co duong thoat. Firmware da bo dieu kien do: khong co mau
    # thi chot goc = 0 va van cat canh.
    "2": ("ToF khong init duoc (khong thay chip tren I2C luc boot) -> khong co "
          "nguon do cao. Chay 'i2c_scan' va 'tof_test'; kiem tra day/nguon 2.8V."),
    "3": "alt_estimator KHONG hop le (state khong huu han) -> ARM lai de reset",
    # Ma 4 KHONG con duoc gan: latch ga hover da tra ve luc ARM, nen ly do tu
    # choi tuong ung nam o ARM_REJECT_NAMES. Giu cho de khong dich so.
}

# Tên pha cất cánh — khớp takeoff_phase_t (takeoff_land.h), kiến trúc PID +
# slew: IDLE -> PRIME -> CLIMB -> HOLD, hoặc -> ABORT.
TAKEOFF_PHASE_NAMES = {
    "0": "IDLE",
    "1": "PRIME",   # motor quay đều, Z/Vz PID CHƯA chạy
    "2": "CLIMB",   # target trượt dần, cascade cầm lái
    "3": "HOLD",    # đã bàn giao sang HOLDING
    "4": "ABORT",
}
# Màu label pha cất cánh (xem _update_takeoff_phase_label) — cam khi chưa rời
# đất, xanh sau khi rời đất, đỏ khi abort.
_TKO_PHASE_COLOR = {
    "1": "#b35c00",   # PRIME — cam, chưa có vòng kín nào cầm lái
    "2": "#b35c00",   # CLIMB — cam cho tới khi TKOLIFT=1 (đổi xanh ở hàm dưới)
    "3": "#0a7d2c",   # HOLD — xanh
    "4": "#c00000",   # ABORT — đỏ
}
# Lý do abort — khớp takeoff_abort_reason_t (takeoff_land.h).
TAKEOFF_ABORT_NAMES = {
    "0": "",
    # 1 KHONG con nghia "khong nhac noi" — bang chung do da chuyen sang ma 5.
    # Gio no chi con nghia "chuoi bi ket, khong tien pha" (han chot tinh theo
    # target trong takeoff_begin()).
    "1": "TIMEOUT (chuoi bi ket, khong tien pha)",
    "2": "ALT_SOURCE_LOST (mat HET ToF va baro)", "3": "TILT (nghieng sap lat)",
    # 4 = takeoff_run() bi goi khi chuoi CHUA active (loi luong o tang tren).
    # Truoc day no muon ma 1 nen hien ra la "TIMEOUT" — sai han nguyen nhan.
    "4": "NOT_ACTIVE (loi luong: chua takeoff_begin)",
    # 5 = bang chung "khong nhac noi" KIEU MOI: ga kich tran lien tuc, KHONG
    # dung do cao do duoc (baro-only khong du chinh xac cho viec do).
    "5": "STUCK: KHONG NHAC NOI (ga kich tran lien tuc)"
         " -> kiem canh quat vuong/lap nguoc, pin (BATV), khoi luong, hover_ff",
}

# Lightweight extractor JUST for the live plot: pulls R/P/Y + the three gyro
# values out of ANY line that contains them, via re.search (not anchored). This
# matches both the flight build's STATUS line (ARM=... | R=.. | G=.. | valid=..)
# and the maintenance build's live-EKF line (R=.. P=.. Y=.. | G=.. | ACC=.. ...),
# so the chart works regardless of which firmware/mode is running.
# Y is optional: the maintenance flight-balance line prints "R=.. P=.. | G=.."
# (no yaw), while the live-Mahony and flight STATUS lines include "Y=..".
PLOT_RE = re.compile(
    r"R=([-\d.]+) P=([-\d.]+)(?: Y=([-\d.]+))? \| "
    r"G=([-\d.]+) ([-\d.]+) ([-\d.]+)"
)

# Cụm độ cao trong dòng STATUS (re.search, không anchored): lấy ALTm (estimator),
# TGT (target) và TOF (mẫu thô) cho đồ thị Altitude.
ALT_PLOT_RE = re.compile(
    r"ALTm=([-\d.]+) VZ=[-\d.]+ TGT=([-\d.]+) MODE=\d AV=\d TOF=([-\d.]+)"
)
# BFILT = baro SAU median-of-3+LPF (xem alt_estimator.h) — search riêng vì nằm
# xa cụm ALTm/TGT/TOF trong dòng STATUS, optional (None nếu firmware cũ hơn).
BARO_FILT_RE = re.compile(r"BFILT=([-\d.]+)")
# BALT = baro TRƯỚC median-of-3+LPF (RAW tương đối so mốc calibrate_ground(),
# xem telemetry.h baro_alt_m) — search riêng cùng lý do BARO_FILT_RE. Vẽ CẢ
# raw lẫn filtered trên đồ thị Altitude để soi median-of-3+LPF đang lọc bao
# nhiêu (yêu cầu refactor alt_estimator "Plot 1" — Baro raw + Baro filtered).
BARO_RAW_RE = re.compile(r"BALT=([-\d.]+)")

# Cụm accel từng trục "| A=ax ay az |" (body-frame, đơn vị g, ĐÃ qua LPF) cho đồ
# thị Accel. "| A=" đủ đặc trưng để không dính "ACC=" hay "ARM=".
ACC_PLOT_RE = re.compile(
    r"\| A=([-\d.]+) ([-\d.]+) ([-\d.]+) \|"
)

# Duty 4 motor "M = m1 m2 m3 m4" -> hiện trên nhãn telemetry (đồ thị cột 3 giờ là
# altitude). \s* khớp cả "M=" lẫn "M = ". re.search.
MOTOR_RE = re.compile(
    r"M\s*=\s*(-?\d+) (-?\d+) (-?\d+) (-?\d+)"
)

# Mahony filter gains reply: "MAH KP=.. KI=.." (from @MAH GET) or
# "MAH OK KP=.. KI=.." (from @MAH SET).
MAH_RE = re.compile(r"^MAH (?:OK )?KP=([-\d.]+) KI=([-\d.]+)")

# Pilot setpoint reply: "SP R=.. P=.. Y=.." (from @SP GET) or
# "SP OK R=.. P=.. Y=.." (from @SP SET). Y is the yaw-RATE target (dps).
SP_RE = re.compile(r"^SP (?:OK )?R=([-\d.]+) P=([-\d.]+) Y=([-\d.]+)")

# Trim reply: "TRIM roll=.. pitch=.." (GET) / "TRIM SET OK roll=.. pitch=.." (SET).
TRIM_RE = re.compile(r"^TRIM (?:SET OK )?roll=([-\d.]+) pitch=([-\d.]+)")

# Altitude controller @ALT reply (hover đã tách sang @TKO):
#   "ALT kp=.. vzkp=.. vzki=.. vzilim=.. tgt=.. mode=.."   (GET)
#   "ALT SET OK kp=.. vzkp=.. vzki=.. vzilim=.."           (SET)
FLIGHT_ALT_RE = re.compile(
    r"^ALT (?:SET OK )?kp=([-\d.]+) vzkp=([-\d.]+) vzki=([-\d.]+) "
    r"vzilim=([-\d.]+)(?: tgt=([-\d.]+) mode=(\d))?"
)

# Ground/takeoff @TKO reply (kiến trúc PID + slew, xem TakeoffGroup):
#   "TKO hover=.. prime=.. ms=.. climb=.."        (GET)
#   "TKO SET OK hover=.. prime=.. ms=.. climb=.." (SET)
# climb= là TUỲ CHỌN trong regex: firmware trả lời "(climb giu nguyen)" khi
# ground-station gửi bản 3 tham số cũ.
TKO_RE = re.compile(
    r"^TKO (?:SET OK )?hover=([-\d.]+) prime=([-\d.]+) ms=([-\d.]+)"
    r"(?: climb=([-\d.]+))?"
)

# Landing @LAND reply:
#   "LAND dvz=.. flarealt=.. fvz=.. tdalt=.."        (GET)
#   "LAND SET OK dvz=.. flarealt=.. fvz=.. tdalt=.." (SET)
LAND_RE = re.compile(
    r"^LAND (?:SET OK )?dvz=([-\d.]+) flarealt=([-\d.]+) fvz=([-\d.]+) tdalt=([-\d.]+)"
)

# Commander @CMDR reply (geofence + ngưỡng fault, xem commander.h):
#   "CMDR altmin=.. altmax=.. battfloor=.. hbms=.. tilt=.. motorsatms=.."       (GET)
#   "CMDR SET OK altmin=.. altmax=.. battfloor=.. hbms=.. tilt=.. motorsatms=.." (SET)
CMDR_RE = re.compile(
    r"^CMDR (?:SET OK )?altmin=([-\d.]+) altmax=([-\d.]+) battfloor=([-\d.]+) "
    r"hbms=(-?\d+) tilt=([-\d.]+) motorsatms=(-?\d+)"
)

# ============================================================
# LIVE PLOT (Serial-Plotter style, pure-Tkinter Canvas)
# ============================================================
# Each channel maps to a field parsed out of the STATUS line. Colours are
# chosen to stay readable on the default (light) Tk background.
#   key -> (legend label, line colour)
# Angles and gyro live on separate plots (very different magnitudes), each
# with its own auto-scaled Y axis.
PLOT_ANGLE_CHANNELS = (
    ("roll",  "Roll (deg)",   "#d81b60"),
    ("pitch", "Pitch (deg)",  "#1e88e5"),
    ("yaw",   "Yaw (deg)",    "#8e24aa"),
)

PLOT_GYRO_CHANNELS = (
    ("gx", "Gyro X (dps)", "#e65100"),
    ("gy", "Gyro Y (dps)", "#2e7d32"),
    ("gz", "Gyro Z (dps)", "#00838f"),
)

# Accel body-frame (g), ĐÃ qua LPF — đứng yên phải thấy ax,ay ~ 0 và az ~ 1.0;
# rung motor lộ ra ở biên độ răng cưa của cả 3 đường.
PLOT_ACC_CHANNELS = (
    ("ax", "Acc X (g)", "#d81b60"),
    ("ay", "Acc Y (g)", "#1e88e5"),
    ("az", "Acc Z (g)", "#6d4c41"),
)
# Độ cao (mét, cùng thang): estimator vs ToF thô vs target — soi chất lượng
# TẦNG 2 (alt bám tof, dead-reckon khi mất) và độ bám target khi HOLD/TAKEOFF.
PLOT_ALT_CHANNELS = (
    ("alt",  "Alt est (m)",       "#1e88e5"),
    ("tof",  "ToF raw (m)",       "#e65100"),
    ("tgt",  "Target (m)",        "#2e7d32"),
    ("braw", "Baro raw (m)",      "#c2185b"),
    ("baro", "Baro filtered (m)", "#8e24aa"),
    ("zine", "Z inertial (m)",    "#6d4c41"),
)
# Vz: so estimate CUỐI (baro đã kéo) với Vz tích phân THUẦN accel (baro không
# chạm) — chênh lệch lớn/nhỏ dần theo thời gian là bình thường (baro correction
# đang hoạt động); chênh KHÔNG BAO GIỜ hội tụ = nghi baro lệch hệ thống hoặc
# alpha/beta (tuning.h/alt_estimator.h) sai. Xem README test E.
PLOT_VZ_CHANNELS = (
    ("vz",    "Vz est (m/s)",         "#1e88e5"),
    ("vzao",  "Vz accel-only (m/s)",  "#e65100"),
    ("vztgt", "Vz target (m/s)",      "#2e7d32"),
)
# Az world-frame (m/s^2, gravity đã trừ) — raw vs SAU LPF: đứng yên motor
# chạy ở ga hover phải thấy đường "filtered" GẦN PHẲNG quanh 0 (xem README
# test G) — nếu vẫn nhặt rung tần số motor rõ, hạ IMU_ACCEL_DLPF_CFG
# (tuning.h mục 11) trước khi nghi ngờ LPF phần mềm.
PLOT_AZ_CHANNELS = (
    ("azraw",  "Az raw (m/s2)",      "#d81b60"),
    ("azfilt", "Az filtered (m/s2)", "#1e88e5"),
)
# How many samples to keep on screen. STATUS arrives ~10 Hz, so 600 points is
# ~60 s of history.
PLOT_MAX_POINTS = 600


class PlotPanel:
    """Scrolling multi-line strip chart drawn on a Tk Canvas.

    Mimics the Arduino IDE Serial Plotter: newest sample on the right, the
    Y axis auto-scales to whatever channels are currently enabled. One panel
    holds one group of channels (e.g. the three angles, or the three gyro
    rates) so each gets its own Y scale. Per-channel checkboxes toggle lines.
    """

    def __init__(self, parent, channels, title,
                 max_points=PLOT_MAX_POINTS, redraw_ms=50):
        self.channels = channels
        self.max_points = max_points
        self.redraw_ms = redraw_ms
        self.paused = False

        self.buffers = {key: deque(maxlen=max_points) for key, _, _ in channels}
        self.enabled = {key: tk.BooleanVar(value=True) for key, _, _ in channels}
        self.latest = {key: None for key, _, _ in channels}

        self.frame = ttk.LabelFrame(parent, text=title)

        # --- control row: per-channel checkboxes + pause/clear ---
        controls = ttk.Frame(self.frame)
        controls.pack(side=tk.TOP, fill=tk.X, padx=4, pady=(2, 0))

        for key, label, colour in channels:
            # tk.Checkbutton (not ttk) so the check text can be coloured to
            # match its line, like a plot legend.
            cb = tk.Checkbutton(
                controls,
                text=label,
                variable=self.enabled[key],
                fg=colour,
                activeforeground=colour,
                selectcolor="",
            )
            cb.pack(side=tk.LEFT, padx=(0, 6))

        self.pause_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            controls, text="Pause", variable=self.pause_var,
            command=self._on_pause,
        ).pack(side=tk.LEFT, padx=(12, 6))

        ttk.Button(controls, text="Clear", command=self.clear).pack(side=tk.LEFT)

        self.readout_var = tk.StringVar(value="")
        ttk.Label(controls, textvariable=self.readout_var, foreground="gray").pack(
            side=tk.RIGHT, padx=(6, 0)
        )

        # --- the canvas itself ---
        self.canvas = tk.Canvas(
            self.frame, height=150, background="#fafafa", highlightthickness=1,
            highlightbackground="#cccccc",
        )
        self.canvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=4, pady=4)

        self._redraw()

    def _on_pause(self):
        self.paused = self.pause_var.get()

    def clear(self):
        for buf in self.buffers.values():
            buf.clear()

    def push(self, values: dict):
        """Append one sample. `values` maps channel key -> float."""
        if self.paused:
            return
        for key, _, _ in self.channels:
            v = values.get(key)
            if v is None:
                continue
            self.buffers[key].append(v)
            self.latest[key] = v

    def _redraw(self):
        c = self.canvas
        c.delete("all")

        w = c.winfo_width()
        h = c.winfo_height()

        if w < 4 or h < 4:
            self.canvas.after(self.redraw_ms, self._redraw)
            return

        pad_l, pad_r, pad_t, pad_b = 46, 8, 8, 6
        plot_w = w - pad_l - pad_r
        plot_h = h - pad_t - pad_b

        active = [
            key for key, _, _ in self.channels
            if self.enabled[key].get() and len(self.buffers[key]) > 0
        ]

        # Empty state: make it obvious the panel is alive but has no data yet,
        # instead of a blank rectangle that looks "broken".
        if not active:
            c.create_rectangle(pad_l, pad_t, pad_l + plot_w, pad_t + plot_h,
                               outline="#cccccc")
            c.create_text(
                pad_l + plot_w / 2, pad_t + plot_h / 2,
                text="Cho du lieu tu ESP (connect + co dong R=.. P=.. Y=.. | G=..)",
                fill="#999999",
            )
            self.canvas.after(self.redraw_ms, self._redraw)
            return

        # Y range across enabled channels only.
        y_min = None
        y_max = None
        for key in active:
            b = self.buffers[key]
            lo = min(b)
            hi = max(b)
            y_min = lo if y_min is None else min(y_min, lo)
            y_max = hi if y_max is None else max(y_max, hi)

        if y_min is None or y_max is None:
            y_min, y_max = -1.0, 1.0
        if y_max - y_min < 1e-6:
            y_min -= 1.0
            y_max += 1.0

        span = y_max - y_min
        y_min -= 0.08 * span
        y_max += 0.08 * span
        span = y_max - y_min

        def to_x(i, n):
            # Newest sample (i = n-1) pinned to the right edge; each sample a
            # fixed pixel step so the trace scrolls right-to-left at constant
            # density regardless of how full the buffer is.
            step = plot_w / (self.max_points - 1)
            return pad_l + plot_w - (n - 1 - i) * step

        def to_y(v):
            return pad_t + (y_max - v) / span * plot_h

        # Plot border.
        c.create_rectangle(pad_l, pad_t, pad_l + plot_w, pad_t + plot_h,
                           outline="#cccccc")

        # Horizontal gridlines + Y labels: top, bottom, and 0 if in range.
        for gy in (y_max, (y_max + y_min) * 0.5, y_min):
            yy = to_y(gy)
            c.create_line(pad_l, yy, pad_l + plot_w, yy, fill="#eeeeee")
            c.create_text(pad_l - 4, yy, text=f"{gy:.0f}", anchor="e",
                         fill="#888888", font=("TkDefaultFont", 7))

        if y_min < 0.0 < y_max:
            y0 = to_y(0.0)
            c.create_line(pad_l, y0, pad_l + plot_w, y0, fill="#bbbbbb")

        # One polyline per enabled channel.
        for key, _, colour in self.channels:
            if not self.enabled[key].get():
                continue
            b = self.buffers[key]
            n = len(b)
            if n < 2:
                continue
            coords = []
            for i, v in enumerate(b):
                coords.append(to_x(i, n))
                coords.append(to_y(v))
            c.create_line(*coords, fill=colour, width=1)

        # Latest-value readout (legend-style, updated in place).
        parts = []
        for key, label, _ in self.channels:
            if self.enabled[key].get() and self.latest[key] is not None:
                short = label.split(" ")[0]
                parts.append(f"{short}={self.latest[key]:.1f}")
        self.readout_var.set("  ".join(parts))

        self.canvas.after(self.redraw_ms, self._redraw)


class GainRow:
    """One kp/ki/kd/ilimit/outlimit slider + editable value box."""

    def __init__(self, parent, row, label, value_range, on_apply,
                 slider_len=150, label_width=9):
        self.on_apply = on_apply
        self._suppress = False

        ttk.Label(parent, text=label, width=label_width, anchor="e").grid(
            row=row, column=0, padx=(4, 2), pady=1, sticky="e"
        )

        self.var = tk.DoubleVar(value=0.0)
        lo, hi = value_range
        self.scale = ttk.Scale(
            parent, from_=lo, to=hi, orient=tk.HORIZONTAL, length=slider_len,
            variable=self.var, command=self._on_scale_move,
        )
        self.scale.grid(row=row, column=1, padx=2, pady=1, sticky="ew")
        # Only actually send the new gain when the drag is released, not on
        # every intermediate tick: PID SET resets the integrator each time.
        self.scale.bind("<ButtonRelease-1>", self._on_scale_release)

        self.entry_var = tk.StringVar(value="0.0000")
        self.entry = ttk.Entry(parent, textvariable=self.entry_var, width=9)
        self.entry.grid(row=row, column=2, padx=(2, 4), pady=1)
        self.entry.bind("<Return>", self._on_entry_commit)
        self.entry.bind("<FocusOut>", self._on_entry_commit)

    def _on_scale_move(self, _value):
        if self._suppress:
            return
        self.entry_var.set(f"{self.var.get():.4f}")

    def _on_scale_release(self, _event):
        if self._suppress:
            return
        self.on_apply()

    def _on_entry_commit(self, _event):
        if self._suppress:
            return

        try:
            value = float(self.entry_var.get())
        except ValueError:
            self.entry_var.set(f"{self.var.get():.4f}")
            return

        self._suppress = True
        try:
            self.entry_var.set(f"{value:.4f}")
            lo, hi = float(self.scale["from"]), float(self.scale["to"])
            self.var.set(min(max(value, lo), hi))
        finally:
            self._suppress = False

        self.on_apply()

    def set_value(self, value: float):
        self._suppress = True
        try:
            self.var.set(value)
            self.entry_var.set(f"{value:.4f}")
        finally:
            self._suppress = False

    def get_value(self) -> float:
        try:
            return float(self.entry_var.get())
        except ValueError:
            return self.var.get()


class GainGroup:
    """One loop/axis pair (e.g. ANGLE ROLL): chỉ hiện kp/ki/kd cho gọn.

    ilimit/outlimit KHÔNG hiện trên UI nhưng firmware @PID SET vẫn cần đủ 7 token,
    nên giữ ngầm (self._ilim/_olim), cập nhật từ @PID GET, gửi kèm khi SET. Mặc định
    khởi điểm > 0 để lỡ SET trước khi GET về cũng không kẹp output = 0.
    """

    def __init__(self, parent, loop, axis, send_pid_set):
        self.loop = loop
        self.axis = axis
        self.send_pid_set = send_pid_set
        self._ilim = 60.0    # ngầm; @PID GET sẽ ghi đè đúng giá trị firmware
        self._olim = 150.0

        self.frame = ttk.LabelFrame(parent, text=f"{loop} {axis}")
        self.rows = {
            field: GainRow(self.frame, i, field, FIELD_RANGE[field], self._apply)
            for i, field in enumerate(PID_VISIBLE_FIELDS)
        }

        btn_row = len(PID_VISIBLE_FIELDS)
        ttk.Button(self.frame, text="Set now", command=self._apply).grid(
            row=btn_row, column=0, columnspan=2, pady=(4, 2), sticky="ew"
        )

        self.status_var = tk.StringVar(value="")
        self.status_label = ttk.Label(self.frame, textvariable=self.status_var, foreground="gray")
        self.status_label.grid(row=btn_row, column=2, pady=(4, 2), sticky="w")

    def _apply(self):
        values = {field: self.rows[field].get_value() for field in PID_VISIBLE_FIELDS}
        values["ilimit"] = self._ilim      # ngầm, kèm theo cho đủ token firmware
        values["outlimit"] = self._olim
        self.send_pid_set(self.loop, self.axis, values)
        self.status_var.set("sending...")
        self.status_label.configure(foreground="gray")

    def set_values(self, kp, ki, kd, ilim, olim):
        self.rows["kp"].set_value(kp)
        self.rows["ki"].set_value(ki)
        self.rows["kd"].set_value(kd)
        self._ilim = ilim                  # lưu ngầm (không hiện UI)
        self._olim = olim

    def mark_ok(self):
        self.status_var.set("OK")
        self.status_label.configure(foreground="green")

    def mark_err(self, msg):
        self.status_var.set(f"ERR: {msg}")
        self.status_label.configure(foreground="red")


class FlightAltGroup:
    """Altitude controller 3 tầng: alt_kp / vz_kp / vz_ki / vz_ilimit + target + mode.

    hover/spool/takeoff/landing đã TÁCH sang panel riêng. Gửi @ALT SET/TGT/MODE;
    live đọc từ cụm ALTm/VZ/... của STATUS.
    """

    FIELDS = ("kp", "vzkp", "vzki", "vzilim", "tgt")

    def __init__(self, parent, on_set, on_get, on_mode, on_tgt_step):
        self.frame = ttk.LabelFrame(parent, text="ALT controller (flight)")

        self.rows = {
            "kp":     GainRow(self.frame, 0, "alt_kp",  (0.0, 10.0),   on_set, slider_len=110, label_width=7),
            # Trần 2000 khớp MOTOR_SAFE_MAX_DUTY firmware hiện tại (11-bit LEDC,
            # xem tuning.h) — vz_kp/ki/ilim là hằng số THANG DUTY, không phải %.
            "vzkp":   GainRow(self.frame, 1, "vz_kp",   (0.0, 2000.0), on_set, slider_len=110, label_width=7),
            "vzki":   GainRow(self.frame, 2, "vz_ki",   (0.0, 2000.0), on_set, slider_len=110, label_width=7),
            "vzilim": GainRow(self.frame, 3, "vz_ilim", (0.0, 2000.0), on_set, slider_len=110, label_width=7),
            "tgt":    GainRow(self.frame, 4, "tgt(m)",  (0.0, 2.0),    on_set, slider_len=110, label_width=7),
        }

        self.live_var = tk.StringVar(value="ALT: -- m | vz -- | mode -- | tof -- | av -")
        ttk.Label(self.frame, textvariable=self.live_var, foreground="#0057b3").grid(
            row=5, column=0, columnspan=3, sticky="w", padx=4, pady=(2, 0))

        btns = ttk.Frame(self.frame)
        btns.grid(row=6, column=0, columnspan=3, sticky="w", padx=2, pady=(2, 0))
        ttk.Button(btns, text="Get", width=4, command=on_get).pack(side=tk.LEFT, padx=1)
        ttk.Button(btns, text="Set", width=4, command=on_set).pack(side=tk.LEFT, padx=1)
        ttk.Button(btns, text="HOLD", width=5, command=lambda: on_mode(2)).pack(side=tk.LEFT, padx=1)
        ttk.Button(btns, text="LOG", width=4, command=lambda: on_mode(1)).pack(side=tk.LEFT, padx=1)
        ttk.Button(btns, text="OFF", width=4, command=lambda: on_mode(0)).pack(side=tk.LEFT, padx=1)

        btns2 = ttk.Frame(self.frame)
        btns2.grid(row=7, column=0, columnspan=3, sticky="w", padx=2)
        ttk.Button(btns2, text="Tgt +10cm", command=lambda: on_tgt_step(1)).pack(side=tk.LEFT, padx=1)
        ttk.Button(btns2, text="Tgt -10cm", command=lambda: on_tgt_step(-1)).pack(side=tk.LEFT, padx=1)

        self.status_var = tk.StringVar(value="")
        self.status_label = ttk.Label(self.frame, textvariable=self.status_var, foreground="gray")
        self.status_label.grid(row=8, column=0, columnspan=3, sticky="w", padx=4)

    def get_values(self):
        return {f: self.rows[f].get_value() for f in self.FIELDS}

    def set_gains(self, kp, vzkp, vzki, vzilim, tgt=None):
        self.rows["kp"].set_value(kp)
        self.rows["vzkp"].set_value(vzkp)
        self.rows["vzki"].set_value(vzki)
        self.rows["vzilim"].set_value(vzilim)
        if tgt is not None:
            self.rows["tgt"].set_value(tgt)

    def set_live(self, alt_m, vz, tgt, mode, tof, av, terr):
        # Khop telemetry_format_alt_mode() (src/telemetry_format.c). Ban truoc
        # thieu ca 4=LANDING lan 5=FLYING -> panel nay hien so tran ("mode 4")
        # thay vi ten khi dang ha canh.
        names = {0: "OFF", 1: "LOG", 2: "HOLD", 3: "TAKEOFF",
                 4: "LANDING", 5: "FLYING"}
        self.live_var.set(
            f"ALT: {alt_m:.2f} m | vz {vz:+.2f} | tgt {tgt:.2f} | "
            f"mode {names.get(mode, mode)} | tof {tof:.2f} | av {av} | terr {terr}")

    def mark_ok(self):
        self.status_var.set("OK")
        self.status_label.configure(foreground="green")


class TakeoffGroup:
    """Thông số ground/takeoff (@TKO): hover, spool_duty, spool_ms.

    `tgt(m)` KHÔNG thuộc @TKO (không phải tham số tune) — nó là THAM SỐ CỦA
    LỆNH `@ALT TAKEOFF <m>`. Đặt ô đó ở đây vì đây là nơi người dùng bấm
    TAKEOFF, nhưng nó đi theo đường lệnh riêng.

    Chuỗi cất cánh giờ closed-loop: SPOOL -> CONTROL_ACTIVE (estimator + Z/Vz
    PID mở) -> liftoff confirm -> CLIMB tới tgt -> SETTLE -> HOLDING(tgt).
    """

    FIELDS = ("hover", "prime", "ms", "climb")

    def __init__(self, parent, on_set, on_get, on_takeoff):
        self.frame = ttk.LabelFrame(parent, text="Takeoff / Ground (@TKO)")
        SL, LW = 100, 8
        specs = [
            # Trần 2000 khớp MOTOR_SAFE_MAX_DUTY firmware hiện tại (11-bit LEDC).
            # hover = feedforward THÔ của vòng Vz. KHÔNG cần chính xác — phần I
            # tự học hover thật, hover chỉ quyết định hội tụ nhanh hay chậm.
            ("hover",   "hover_ff",   (0.0, 2000.0)),
            # ⚠ prime_duty PHẢI thấp hơn hover: pha PRIME chỉ để motor quay đều,
            # KHÔNG được đủ sức nhấc drone (xem takeoff_land.h).
            ("prime",   "prime_duty", (0.0, 2000.0)),
            ("ms",      "prime_ms",   (1.0, 3000.0)),
            # max_climb = tốc độ TRƯỢT của target + trần |vz_target|. Đây là
            # núm "leo nhanh hay chậm" duy nhất.
            ("climb",   "max_climb",  (0.05, 2.0)),
        ]
        self.rows = {key: GainRow(self.frame, i, lbl, rng, on_set,
                                  slider_len=SL, label_width=LW)
                     for i, (key, lbl, rng) in enumerate(specs)}
        r = len(specs)

        # Target độ cao cho LỆNH takeoff (không phải @TKO tune). Mặc định 1.00m
        # theo yêu cầu người dùng (trước là 0.30m).
        # 1.00m nằm trong tầm tin cậy của VL53L1X LONG (~2.6m trong nhà) và dưới
        # trần ALT_EST_MAX_FLIGHT_Z_M, nên không chạm giới hạn nào.
        tgtf = ttk.Frame(self.frame)
        tgtf.grid(row=r, column=0, columnspan=3, sticky="w", padx=4, pady=(4, 0))
        ttk.Label(tgtf, text="tgt(m):").pack(side=tk.LEFT)
        self.target_var = tk.StringVar(value="1.00")
        ttk.Spinbox(tgtf, from_=0.05, to=3.0, increment=0.05, width=6,
                    textvariable=self.target_var).pack(side=tk.LEFT, padx=(2, 6))
        ttk.Label(tgtf, text="do cao SE LEO TOI roi HOLD", foreground="gray").pack(side=tk.LEFT)
        r += 1

        ttk.Label(self.frame,
                  text="PRIME (khong PID) -> CLIMB (target truot, I tu hoc hover) -> HOLDING(tgt)",
                  foreground="gray").grid(row=r, column=0, columnspan=3, sticky="w", padx=4)

        btns = ttk.Frame(self.frame)
        btns.grid(row=r + 1, column=0, columnspan=3, sticky="w", padx=2, pady=(2, 0))
        ttk.Button(btns, text="Get", width=4, command=on_get).pack(side=tk.LEFT, padx=1)
        ttk.Button(btns, text="Set", width=4, command=on_set).pack(side=tk.LEFT, padx=1)
        self.takeoff_btn = tk.Button(
            btns, text="TAKEOFF (t)", command=on_takeoff,
            bg="#0a7d2c", fg="white", activebackground="#0c9235",
            font=("Segoe UI", 9, "bold"))
        self.takeoff_btn.pack(side=tk.LEFT, padx=(6, 1))

        self.status_var = tk.StringVar(value="")
        ttk.Label(self.frame, textvariable=self.status_var, foreground="gray").grid(
            row=r + 2, column=0, columnspan=3, sticky="w", padx=4)

    def get_values(self):
        return {f: self.rows[f].get_value() for f in self.FIELDS}

    def set_values(self, hover, prime, ms, climb=None):
        self.rows["hover"].set_value(hover)
        self.rows["prime"].set_value(prime)
        self.rows["ms"].set_value(ms)
        # climb=None khi firmware trả lời bản 3 tham số cũ -> giữ nguyên ô hiện
        # tại thay vì ghi 0 (0 sẽ làm target không bao giờ trượt tới đích).
        if climb is not None:
            self.rows["climb"].set_value(climb)

    def mark_ok(self):
        self.status_var.set("OK")


class LandingGroup:
    """Thông số hạ cánh (@LAND): descent_vz, flare_alt, flare_vz, touchdown_alt.
    + nút LAND kích hạ tự động. Gửi @LAND SET/GET, @LAND (bare) để kích."""

    FIELDS = ("dvz", "flarealt", "fvz", "tdalt")

    def __init__(self, parent, on_set, on_get, on_land):
        self.frame = ttk.LabelFrame(parent, text="Landing (@LAND)")
        self.rows = {
            "dvz":      GainRow(self.frame, 0, "descent_vz", (0.05, 1.0), on_set, slider_len=110, label_width=9),
            "flarealt": GainRow(self.frame, 1, "flare_alt",  (0.05, 1.0), on_set, slider_len=110, label_width=9),
            "fvz":      GainRow(self.frame, 2, "flare_vz",   (0.02, 0.5), on_set, slider_len=110, label_width=9),
            "tdalt":    GainRow(self.frame, 3, "td_alt",     (0.02, 0.5), on_set, slider_len=110, label_width=9),
        }
        ttk.Label(self.frame, text="hạ vz cố định -> flare -> chạm đất -> disarm",
                  foreground="gray").grid(row=4, column=0, columnspan=3, sticky="w", padx=4)

        btns = ttk.Frame(self.frame)
        btns.grid(row=5, column=0, columnspan=3, sticky="w", padx=2, pady=(2, 0))
        ttk.Button(btns, text="Get", width=5, command=on_get).pack(side=tk.LEFT, padx=1)
        ttk.Button(btns, text="Set", width=5, command=on_set).pack(side=tk.LEFT, padx=1)
        self.land_btn = tk.Button(
            btns, text="LAND (l)", command=on_land,
            bg="#b5651d", fg="white", activebackground="#c9761f",
            font=("Segoe UI", 9, "bold"))
        self.land_btn.pack(side=tk.LEFT, padx=(6, 1))

        self.status_var = tk.StringVar(value="")
        ttk.Label(self.frame, textvariable=self.status_var, foreground="gray").grid(
            row=6, column=0, columnspan=3, sticky="w", padx=4)

    def get_values(self):
        return {f: self.rows[f].get_value() for f in self.FIELDS}

    def set_values(self, dvz, flarealt, fvz, tdalt):
        self.rows["dvz"].set_value(dvz)
        self.rows["flarealt"].set_value(flarealt)
        self.rows["fvz"].set_value(fvz)
        self.rows["tdalt"].set_value(tdalt)

    def mark_ok(self):
        self.status_var.set("OK")


class CommanderGroup:
    """Geofence + ngưỡng fault Commander (@CMDR — xem commander.h). KHÁC
    Takeoff/Landing: chạy được BẤT KỲ LÚC NÀO (kể cả đang bay), không cần
    DISARMED — chỉnh geofence/failsafe live khi cần."""

    FIELDS = ("altmin", "altmax", "battfloor", "hbms", "tilt", "motorsatms")

    def __init__(self, parent, on_set, on_get):
        self.frame = ttk.LabelFrame(parent, text="Commander / Failsafe (@CMDR)")
        SL, LW = 100, 9
        specs = [
            ("altmin",     "alt_min_m",    (-5.0, 20.0)),
            ("altmax",     "alt_max_m",    (0.1, 50.0)),
            ("battfloor",  "batt_floor_v", (0.0, 12.6)),
            ("hbms",       "hb_timeout_ms",(100.0, 10000.0)),
            ("tilt",       "hard_tilt_deg",(10.0, 90.0)),
            ("motorsatms", "motorsat_ms",  (100.0, 10000.0)),
        ]
        self.rows = {key: GainRow(self.frame, i, lbl, rng, on_set,
                                  slider_len=SL, label_width=LW)
                     for i, (key, lbl, rng) in enumerate(specs)}
        r = len(specs)
        ttk.Label(self.frame, text="geofence + nguong SOFT/HARD fault -- doi duoc BAT KY LUC NAO",
                  foreground="gray").grid(row=r, column=0, columnspan=3, sticky="w", padx=4)

        btns = ttk.Frame(self.frame)
        btns.grid(row=r + 1, column=0, columnspan=3, sticky="w", padx=2, pady=(2, 0))
        ttk.Button(btns, text="Get", width=4, command=on_get).pack(side=tk.LEFT, padx=1)
        ttk.Button(btns, text="Set", width=4, command=on_set).pack(side=tk.LEFT, padx=1)

        self.status_var = tk.StringVar(value="")
        ttk.Label(self.frame, textvariable=self.status_var, foreground="gray").grid(
            row=r + 2, column=0, columnspan=3, sticky="w", padx=4)

    def get_values(self):
        return {f: self.rows[f].get_value() for f in self.FIELDS}

    def set_values(self, altmin, altmax, battfloor, hbms, tilt, motorsatms):
        self.rows["altmin"].set_value(altmin)
        self.rows["altmax"].set_value(altmax)
        self.rows["battfloor"].set_value(battfloor)
        self.rows["hbms"].set_value(hbms)
        self.rows["tilt"].set_value(tilt)
        self.rows["motorsatms"].set_value(motorsatms)

    def mark_ok(self):
        self.status_var.set("OK")


class FlightCommandGroup:
    """Timed-command 1 phát (@HOVER/@MOVE/@YAW — xem command_parser.c
    handle_hover/handle_move/handle_yaw, MỚI, trước đây chỉ gọi được qua USB/
    MicroPython). KHÁC panel Takeoff/Landing/Commander: KHÔNG có state để
    Get/Set lại — mỗi hàng chỉ là 1 nút bắn 1 lệnh với tham số đọc từ ô nhập
    ngay lúc bấm, giống triết lý panel Test Motor."""

    MOVE_DIRS = ("forward", "back", "left", "right", "up", "down", "cw", "ccw")

    def __init__(self, parent, on_hover, on_move, on_yaw):
        self.frame = ttk.LabelFrame(parent, text="Flight Commands (@HOVER / @MOVE / @YAW)")

        # ---- Hover ----
        ttk.Label(self.frame, text="Hover", width=6, anchor="e").grid(
            row=0, column=0, padx=(4, 2), pady=2, sticky="e")
        self.hover_sec = tk.StringVar(value="2.0")
        ttk.Entry(self.frame, textvariable=self.hover_sec, width=6).grid(
            row=0, column=1, padx=2, pady=2, sticky="w")
        ttk.Label(self.frame, text="sec").grid(row=0, column=2, padx=(0, 4), sticky="w")
        ttk.Button(self.frame, text="Send", width=6,
                  command=lambda: on_hover(self._safe_float(self.hover_sec, 0.0))
                  ).grid(row=0, column=3, padx=(2, 4), pady=2)

        # ---- Move ----
        ttk.Label(self.frame, text="Move", width=6, anchor="e").grid(
            row=1, column=0, padx=(4, 2), pady=2, sticky="e")
        self.move_dir = tk.StringVar(value=self.MOVE_DIRS[0])
        ttk.Combobox(self.frame, textvariable=self.move_dir, values=self.MOVE_DIRS,
                    width=8, state="readonly").grid(row=1, column=1, padx=2, pady=2, sticky="w")
        self.move_pct = tk.StringVar(value="30")
        ttk.Entry(self.frame, textvariable=self.move_pct, width=5).grid(
            row=1, column=2, padx=2, pady=2, sticky="w")
        ttk.Label(self.frame, text="%").grid(row=1, column=3, sticky="w")
        self.move_sec = tk.StringVar(value="1.0")
        ttk.Entry(self.frame, textvariable=self.move_sec, width=5).grid(
            row=1, column=4, padx=2, pady=2, sticky="w")
        ttk.Label(self.frame, text="sec").grid(row=1, column=5, padx=(0, 4), sticky="w")
        ttk.Button(self.frame, text="Send", width=6,
                  command=lambda: on_move(
                      self.move_dir.get(),
                      int(self._safe_float(self.move_pct, 0.0)),
                      self._safe_float(self.move_sec, 0.0))
                  ).grid(row=1, column=6, padx=(2, 4), pady=2)

        # ---- Yaw ----
        ttk.Label(self.frame, text="Yaw", width=6, anchor="e").grid(
            row=2, column=0, padx=(4, 2), pady=2, sticky="e")
        self.yaw_deg = tk.StringVar(value="90")
        ttk.Entry(self.frame, textvariable=self.yaw_deg, width=6).grid(
            row=2, column=1, padx=2, pady=2, sticky="w")
        ttk.Label(self.frame, text="deg (+phai/-trai)").grid(
            row=2, column=2, columnspan=2, padx=(0, 4), sticky="w")
        ttk.Button(self.frame, text="Send", width=6,
                  command=lambda: on_yaw(self._safe_float(self.yaw_deg, 0.0))
                  ).grid(row=2, column=6, padx=(2, 4), pady=2)

        self.status_var = tk.StringVar(value="")
        ttk.Label(self.frame, textvariable=self.status_var, foreground="gray").grid(
            row=3, column=0, columnspan=7, sticky="w", padx=4, pady=(2, 0))

    @staticmethod
    def _safe_float(var: tk.StringVar, default: float) -> float:
        try:
            return float(var.get())
        except ValueError:
            return default


class TestMotorGroup:
    """Test 1 động cơ riêng lẻ ở duty thấp (@TEST — xem command_parser.c::
    handle_test(), command.h CMD_TEST_MOTOR). CHỈ dùng bench-test, cánh quạt
    PHẢI tháo hết — 1 xung ~500ms tự dừng, KHÔNG có GET (không phải tham số
    tune). Slider hiển thị THANG DUTY (0..500, KHỚP trần thật của firmware:
    TEST_MOTOR_MAX_DUTY_PCT=25% * MOTOR_SAFE_MAX_DUTY=2000 — xem flight_core.c)
    để khớp cảm giác với các slider duty khác trong GUI (hover/spool...),
    quy đổi sang duty_pct (0-100) NGAY TRƯỚC KHI gửi vì đó mới là đơn vị dây
    (@TEST <motor> <duty_pct>)."""

    # Trần thật của firmware — xem TEST_MOTOR_MAX_DUTY_PCT/MOTOR_SAFE_MAX_DUTY
    # trong flight_core.c. Đặt CỨNG ở đây CHỈ để label/slider hiển thị đúng
    # kỳ vọng cho người dùng — KHÔNG phải nơi enforce an toàn thật (đó là
    # flight_core.c, GUI gửi bao nhiêu cũng bị clamp lại ở firmware).
    DUTY_MAX = 500
    DUTY_TO_PCT_DIVISOR = 20   # pct = duty / 20  (MOTOR_SAFE_MAX_DUTY=2000 -> /100*100/2000 = /20)

    def __init__(self, parent, on_test):
        self.on_test = on_test
        self.frame = ttk.LabelFrame(parent, text="Test Motor (@TEST) — BENCH ONLY")

        warn = tk.Label(
            self.frame,
            text="!!! THAO HET CANH QUAT truoc khi test !!! CHI chay duoc luc DISARMED, tu dong dung ~500ms.",
            fg="#a00000", font=("Segoe UI", 9, "bold"), wraplength=420, justify="left")
        warn.grid(row=0, column=0, columnspan=4, sticky="w", padx=4, pady=(2, 4))

        self.duty_row = GainRow(self.frame, 1, "duty", (0.0, float(self.DUTY_MAX)),
                                on_apply=lambda: None,   # slider tự nó KHÔNG gửi gì — chỉ nút M1..M4 mới gửi
                                slider_len=160, label_width=6)

        btns = ttk.Frame(self.frame)
        btns.grid(row=2, column=0, columnspan=4, sticky="w", padx=2, pady=(4, 2))
        for idx in (1, 2, 3, 4):
            ttk.Button(btns, text=f"Test M{idx}", width=8,
                       command=lambda i=idx: self._on_test_clicked(i)).pack(side=tk.LEFT, padx=2)

        self.status_var = tk.StringVar(value="")
        ttk.Label(self.frame, textvariable=self.status_var, foreground="gray").grid(
            row=3, column=0, columnspan=4, sticky="w", padx=4)

    def _on_test_clicked(self, motor_idx: int):
        duty = self.duty_row.get_value()
        pct = int(round(duty / self.DUTY_TO_PCT_DIVISOR))
        confirmed = messagebox.askyesno(
            "Test Motor",
            f"Motor M{motor_idx} se quay ~500ms o duty~{duty:.0f} (~{pct}%).\n"
            "Da THAO HET canh quat / co dinh khung chua?\n\n"
            f"Xac nhan test M{motor_idx}?",
        )
        if not confirmed:
            self.status_var.set("Da huy.")
            return
        self.on_test(motor_idx, pct)
        self.status_var.set(f"sending... M{motor_idx} @ {pct}%")


class CalibrationGroup:
    """Panel calibration (@CAL — xem command_parser.c::handle_cal(), MỚI).

    Không có slider/GET-SET như các panel khác — mỗi nút chỉ KÍCH 1 bước
    trong quy trình gyro/accel-6face/mag/baro (giống hệt lệnh console
    calib_*), tiến độ đọc qua "Refresh status" (gửi @CAL STATUS, reply hiện ở
    label dưới cùng — xem PidTunerApp._on_line() nhánh `line.startswith("CAL ")`).
    """

    def __init__(self, parent, send_cmd):
        self.frame = ttk.LabelFrame(parent, text="Calibration (@CAL)")
        row_kwargs = {"side": tk.TOP, "fill": tk.X, "padx": 4, "pady": 2}

        r = ttk.Frame(self.frame); r.pack(**row_kwargs)
        ttk.Label(r, text="Gyro:", width=6).pack(side=tk.LEFT)
        ttk.Button(r, text="Start (~1.5s, đứng yên)", command=lambda: send_cmd("@CAL GYRO START")).pack(side=tk.LEFT, padx=1)
        ttk.Button(r, text="Abort", width=6, command=lambda: send_cmd("@CAL GYRO ABORT")).pack(side=tk.LEFT, padx=1)

        r = ttk.Frame(self.frame); r.pack(**row_kwargs)
        ttk.Label(r, text="Accel:", width=6).pack(side=tk.LEFT)
        ttk.Button(r, text="Next face (giữ yên ~0.5s)", command=lambda: send_cmd("@CAL ACC NEXT")).pack(side=tk.LEFT, padx=1)
        ttk.Button(r, text="Abort", width=6, command=lambda: send_cmd("@CAL ACC ABORT")).pack(side=tk.LEFT, padx=1)
        ttk.Label(self.frame, text="6 lần, đổi hướng đặt drone mỗi lần (bao phủ +-g cả 3 trục)",
                  foreground="gray", wraplength=260, justify="left").pack(side=tk.TOP, anchor="w", padx=4)

        r = ttk.Frame(self.frame); r.pack(**row_kwargs)
        ttk.Label(r, text="Mag:", width=6).pack(side=tk.LEFT)
        ttk.Button(r, text="Start", width=6, command=lambda: send_cmd("@CAL MAG START")).pack(side=tk.LEFT, padx=1)
        ttk.Button(r, text="Stop", width=6, command=lambda: send_cmd("@CAL MAG STOP")).pack(side=tk.LEFT, padx=1)
        ttk.Button(r, text="Abort", width=6, command=lambda: send_cmd("@CAL MAG ABORT")).pack(side=tk.LEFT, padx=1)
        ttk.Label(self.frame, text="Start rồi XOAY hình số 8 liên tục suốt 60s (tự động tính kết quả "
                  "khi hết giờ — Stop chỉ để kết thúc SỚM hơn nếu muốn)",
                  foreground="gray", wraplength=260, justify="left").pack(side=tk.TOP, anchor="w", padx=4)

        r = ttk.Frame(self.frame); r.pack(**row_kwargs)
        ttk.Label(r, text="Baro:", width=6).pack(side=tk.LEFT)
        ttk.Button(r, text="Ground cal (~1s, đứng yên)", command=lambda: send_cmd("@CAL BARO START")).pack(side=tk.LEFT, padx=1)

        r = ttk.Frame(self.frame); r.pack(**row_kwargs)
        ttk.Button(r, text="Refresh status", command=lambda: send_cmd("@CAL STATUS")).pack(side=tk.LEFT, padx=1)
        erase_btn = tk.Button(
            r, text="ERASE ALL", command=lambda: send_cmd("@CAL RESET"),
            bg="#b00020", fg="white", activebackground="#c9302c")
        erase_btn.pack(side=tk.LEFT, padx=(6, 1))

        self.status_var = tk.StringVar(value="(chưa có dữ liệu — bấm Refresh status)")
        ttk.Label(self.frame, textvariable=self.status_var, foreground="#0057b3",
                  wraplength=280, justify="left").pack(side=tk.TOP, anchor="w", padx=4, pady=(4, 2))

    def set_status(self, text: str):
        self.status_var.set(text)


class PidTunerApp:
    def __init__(self, root, host, port, bind_port, debug):
        self.root = root
        self.root.title("UAV-Mini PID Tuner")

        self.console: Optional[UavUdpConsole] = None
        self.line_queue: "queue.Queue[str]" = queue.Queue()

        self.host = host
        self.port = port
        self.bind_port = bind_port
        self.debug = debug

        self.groups: dict[tuple[str, str], GainGroup] = {}

        # ---- Manual control state (tab "Manual Control") ----
        # cmd_roll/pitch/yaw = LỆNH bay (firmware cộng lên trim). alt dùng >/< (step
        # cố định firmware). Chỉ tác dụng khi tab Manual đang chọn.
        self.cmd_roll = 0.0
        self.cmd_pitch = 0.0
        self.cmd_yaw = 0.0
        self._held = set()             # token hướng đang giữ: roll+/roll-/pitch±/yaw±
        self._keys_down = set()        # keysym đang giữ (chống auto-repeat Windows)
        self._release_after = {}       # keysym -> after id (chống auto-repeat X11)
        self._ws_repeat_after = None   # timer auto-repeat W/S (alt)
        # (Da bo _pid_owns_throttle: W/S khong con re nhanh theo MODE o phia
        # GUI — xem khoi comment tren _ws_press(). Firmware la noi duy nhat
        # dich offset theo state.)
        self._ws_repeat_key = None
        self._hb_after = None          # heartbeat @SP
        self._cmdr_hb_after = None     # heartbeat Commander watchdog ('p'), xem _start_cmdr_heartbeat()
        self._last_sent_sp = None      # BUG 2: (r,p,y) đã gửi lần cuối (dedup)
        self._last_sp_time = 0.0       # BUG 2: thời điểm gửi cuối (keepalive 1Hz)
        self._last_key_event = 0.0     # BUG 4: timestamp key event cuối (watchdog kẹt phím)
        self._kw_after = None          # BUG 4: timer watchdog kẹt phím
        # ---- Auto-brake (chỉ roll/pitch) ----
        # Nhả phím -> nghiêng NGƯỢC một nhịp ngắn để phanh quán tính, rồi về 0.
        # Mỗi trục có timer + giá trị phanh riêng (độc lập). Tk vars (spinbox +
        # checkbox) tạo trong _build_manual_tab.
        self._hold_start = {}                          # name hướng -> timestamp nhấn
        self._brake_after = {"roll": None, "pitch": None}  # timer kết thúc phanh
        self._brake_cmd = {"roll": 0.0, "pitch": 0.0}      # góc phanh đang áp (override)
        self._brake_secs = {"roll": 0.0, "pitch": 0.0}     # thời lượng phanh (hiển thị)
        self._manual_btns = {}         # tên -> nút (để highlight active)
        self._manual_tab_id = None     # id tab Manual trong notebook
        self._last_alt_target = None   # TGT= từ telemetry (hiện trên tab Manual)
        # nút hướng -> token; phím -> nút hướng.
        # Quy ước dấu (khớp firmware, ĐÃ ĐỔI theo yêu cầu: roll=tiến/lùi,
        # pitch=trái/phải — KHÁC chuẩn hàng không, xem CMD_SET_ATTITUDE trong
        # flight_core.c): tiến=roll âm, lùi=roll dương; phải=pitch âm,
        # trái=pitch dương; A=yaw dương, D=yaw âm (đúng chiều drone thật).
        self._dir_token = {"up": "roll-", "down": "roll+", "left": "pitch+",
                           "right": "pitch-", "yaw_l": "yaw+", "yaw_r": "yaw-"}
        self.KEY_TO_DIR = {"up": "up", "down": "down", "left": "left",
                           "right": "right", "a": "yaw_l", "d": "yaw_r"}
        # Phím tắt trim (tab Manual): q/e = roll -/+, 1/3 = pitch -/+, mỗi nhấn 1 bước.
        self._TRIM_KEYS = {"q": ("roll", -1), "e": ("roll", +1),
                           "1": ("pitch", -1), "3": ("pitch", +1)}
        self._trim_keys_down = set()   # chặn auto-repeat -> 1 nhấn = 1 bước

        self._build_ui()
        self.root.after(50, self._poll_queue)
        self.root.after(200, self._key_watchdog)   # BUG 4: watchdog kẹt phím
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        # Kill switch always reachable, regardless of focused widget.
        self.root.bind("<Escape>", lambda _e: self.send_kill())

        if self.host:
            self.root.after(200, self.connect)

    # ---------------- UI construction ----------------

    def _make_scrollable(self, parent):
        """Boc noi dung trong Canvas + 2 thanh cuon (DOC va NGANG).

        TRUOC: chi cuon DOC, va tu ep canvas RONG THEO NOI DUNG (widget
        canvas.configure(width=...)) — hop ly khi nua ben nay duoc chiem bao
        nhieu rong tuy thich, phan con lai (Log) nuot het phan thua.

        TU KHI Log/Params moi ben bi khoa CUNG 50% man hinh (place(relwidth=
        0.5), xem _build_ui): noi dung params (luoi PID 3 cot + ALT/TKO/LAND +
        CMDR + Mahony + Calib, hoac 4 LabelFrame ngang cua tab Manual) RONG
        HON han nua man hinh do -> ep canvas rong theo se PHA vo khoa 50/50,
        con khong ep thi content bi CAT MAT ben phai khong xem/bam duoc.
        Dung ca hai deu sai — can THANH CUON NGANG thay the.

        Tra ve (container, inner) giong het truoc: container la thu can
        pack/grid vao `parent`, inner la frame THAT de dung noi dung nhu binh
        thuong (code goi ham nay chi doi target tu `parent` sang `inner`).
        """
        container = ttk.Frame(parent)
        container.grid_rowconfigure(0, weight=1)
        container.grid_columnconfigure(0, weight=1)

        canvas = tk.Canvas(container, highlightthickness=0, bd=0)
        vbar = ttk.Scrollbar(container, orient="vertical", command=canvas.yview)
        hbar = ttk.Scrollbar(container, orient="horizontal", command=canvas.xview)
        canvas.configure(yscrollcommand=vbar.set, xscrollcommand=hbar.set)
        # grid (khong pack): can goc duoi-phai TRONG (khong co thanh cuon nao
        # de o do) va 2 thanh nam dung canh canvas ma no dieu khien.
        canvas.grid(row=0, column=0, sticky="nsew")
        vbar.grid(row=0, column=1, sticky="ns")
        hbar.grid(row=1, column=0, sticky="ew")

        inner = ttk.Frame(canvas)
        inner_id = canvas.create_window((0, 0), window=inner, anchor="nw")

        def _on_inner_configure(_e):
            canvas.configure(scrollregion=canvas.bbox("all"))
        inner.bind("<Configure>", _on_inner_configure)

        def _on_canvas_configure(e):
            # Rong inner = max(rong canvas, rong noi dung yeu cau): canvas
            # RONG HON content (man hinh lon) -> content GIAN theo cho khong
            # de trong lech; canvas HEP HON content (truong hop CHINH tu khi
            # khoa 50/50) -> content giu nguyen rong that, phan thua xem bang
            # hbar. KHONG con dung canvas.configure(width=..) nhu ban cu —
            # do la thu pha vo khoa relwidth=0.5 cua container ben ngoai.
            req_w = inner.winfo_reqwidth()
            canvas.itemconfig(inner_id, width=max(e.width, req_w))
        canvas.bind("<Configure>", _on_canvas_configure)

        # Lan chuot: doc mac dinh, SHIFT+lan chuot = ngang (quy uoc pho bien
        # cua Windows/trinh duyet). Chi bind_all khi con tro DANG O TREN canvas
        # nay (Enter/Leave) de khong nuot lan chuot cua cac vung cuon khac
        # (vd ScrolledText log ben nua kia).
        def _wheel_y(e):
            canvas.yview_scroll(int(-1 * (e.delta / 120)), "units")
        def _wheel_x(e):
            canvas.xview_scroll(int(-1 * (e.delta / 120)), "units")
        def _bind_wheel(_e):
            canvas.bind_all("<MouseWheel>", _wheel_y)
            canvas.bind_all("<Shift-MouseWheel>", _wheel_x)
        def _unbind_wheel(_e):
            canvas.unbind_all("<MouseWheel>")
            canvas.unbind_all("<Shift-MouseWheel>")
        canvas.bind("<Enter>", _bind_wheel)
        canvas.bind("<Leave>", _unbind_wheel)

        return container, inner

    def _build_ui(self):
        top = ttk.Frame(self.root, padding=6)
        top.pack(side=tk.TOP, fill=tk.X)

        ttk.Label(top, text="ESP32 IP:").pack(side=tk.LEFT)
        self.host_var = tk.StringVar(value=self.host or "")
        ttk.Entry(top, textvariable=self.host_var, width=16).pack(side=tk.LEFT, padx=(2, 8))

        ttk.Label(top, text="Port:").pack(side=tk.LEFT)
        self.port_var = tk.StringVar(value=str(self.port))
        ttk.Entry(top, textvariable=self.port_var, width=6).pack(side=tk.LEFT, padx=(2, 8))

        self.connect_btn = ttk.Button(top, text="Connect", command=self.toggle_connect)
        self.connect_btn.pack(side=tk.LEFT, padx=(0, 8))

        self.conn_status_var = tk.StringVar(value="Disconnected")
        self.conn_status_label = ttk.Label(top, textvariable=self.conn_status_var, foreground="red")
        self.conn_status_label.pack(side=tk.LEFT)

        warn = ttk.Label(
            self.root,
            text="CANH BAO: kiem tra gain truoc khi bay, KILL luon san sang (nut do / phim Esc).",
            foreground="#b35c00",
            padding=(6, 0),
        )
        warn.pack(side=tk.TOP, fill=tk.X)

        flight = ttk.Frame(self.root, padding=6)
        flight.pack(side=tk.TOP, fill=tk.X)

        # ---- Mode + arm (khớp phím firmware) ----
        # 'f' = VÀO flight mode, bật STATUS streaming định kỳ; 'q' = THOÁT flight
        # mode, tắt streaming (xem command_parser.c case 'f'/'q', MỚI) — một
        # chiều bật/tắt trên 2 phím KHÁC NHAU (không phải toggle trên cùng 1
        # phím). MẶC ĐỊNH TẮT lúc mới kết nối (KHÔNG tự phát STATUS chỉ vì UDP
        # peer vừa học được). Lệnh khác (@CAL/@PID/../r/k/t/l) LUÔN hoạt động
        # bất kể cờ này. CÁC NÚT Thr+/- phía dưới GIỜ ĐÃ HOẠT ĐỘNG THẬT (bench-
        # test tăng ga tay để tune PID, xem command_parser.c case 'b'/'n'/
        # '+'/'-'/']'/'['/'0' — CHỈ có tác dụng SAU KHI bấm "Bench (b)" để vào
        # FSM_BENCH_RAMP, xem GHI CHÚ 3 flight_state_machine.h). HOLD(z)/LOG(x)/
        # Status(e) VẪN LÀ TÀN DƯ từ console UAV-Mini cũ — firmware hiện KHÔNG
        # xử lý (sẽ trả "ERR unknown"), giữ lại vì ngoài phạm vi thay đổi lần
        # này. "Exit (q)" KHÔNG phải tàn dư — nó chính là phím thoát flight
        # mode ở trên, đã hoạt động thật.
        ttk.Button(flight, text="Flight (f)", command=lambda: self.send_raw_cmd("f")).pack(side=tk.LEFT, padx=2)
        ttk.Button(flight, text="ARM (r)", command=self.send_arm).pack(side=tk.LEFT, padx=2)
        ttk.Button(flight, text="DISARM (d)", command=self.send_disarm).pack(side=tk.LEFT, padx=2)

        kill_btn = tk.Button(
            flight, text="KILL (k)", command=self.send_kill,
            bg="red", fg="white", activebackground="#a00000", activeforeground="white",
        )
        kill_btn.pack(side=tk.LEFT, padx=2)
        ttk.Button(flight, text="Exit (q)", command=lambda: self.send_raw_cmd("q")).pack(side=tk.LEFT, padx=(2, 8))

        # ---- Bench-test tăng ga tay để tune PID (xem GHI CHÚ 3
        # flight_state_machine.h) — 'b' vào FSM_BENCH_RAMP (CHỈ từ ARMED),
        # 'n' dừng NGAY (không latch, về ARMED, bench lại được luôn). Throttle
        # +/-/]/[/0 phía dưới CHỈ có tác dụng SAU KHI bấm "Bench (b)".
        bench_btn = tk.Button(
            flight, text="Bench (b)", command=self.send_bench_start,
            bg="#ffa000", fg="black", activebackground="#c67100",
        )
        bench_btn.pack(side=tk.LEFT, padx=2)
        ttk.Button(flight, text="Bench Stop (n)", command=lambda: self.send_raw_cmd("n")).pack(side=tk.LEFT, padx=(2, 8))

        # ---- Throttle bench-test (+/-=±20, ]/[=±5, 0=về 0) ----
        ttk.Button(flight, text="Thr +20", command=lambda: self.send_raw_cmd("+")).pack(side=tk.LEFT, padx=2)
        ttk.Button(flight, text="Thr +5",  command=lambda: self.send_raw_cmd("]")).pack(side=tk.LEFT, padx=2)
        ttk.Button(flight, text="Thr -5",  command=lambda: self.send_raw_cmd("[")).pack(side=tk.LEFT, padx=2)
        ttk.Button(flight, text="Thr -20", command=lambda: self.send_raw_cmd("-")).pack(side=tk.LEFT, padx=2)
        ttk.Button(flight, text="Thr 0",   command=lambda: self.send_raw_cmd("0")).pack(side=tk.LEFT, padx=(2, 8))

        # ---- Altitude 3 tầng (z=HOLD, x=LOG, >/<=target ±10cm) ----
        ttk.Button(flight, text="HOLD (z)", command=lambda: self.send_raw_cmd("z")).pack(side=tk.LEFT, padx=2)
        ttk.Button(flight, text="LOG (x)",  command=lambda: self.send_raw_cmd("x")).pack(side=tk.LEFT, padx=2)
        ttk.Button(flight, text="Alt +",    command=lambda: self.send_raw_cmd(">")).pack(side=tk.LEFT, padx=2)
        ttk.Button(flight, text="Alt -",    command=lambda: self.send_raw_cmd("<")).pack(side=tk.LEFT, padx=(2, 8))

        ttk.Button(flight, text="Status (e)", command=lambda: self.send_raw_cmd("e")).pack(side=tk.LEFT, padx=2)
        ttk.Button(flight, text="GET ALL PID", command=self.get_all_pid).pack(side=tk.LEFT, padx=(8, 2))

        # Cập nhật khi thấy reply "FLIGHT MODE ON/OFF..." (xem command_parser.c
        # case 'f') — chỉ hiển thị, KHÔNG tự suy đoán trạng thái nếu chưa bấm.
        self.telem_stream_var = tk.StringVar(value="STATUS streaming: ? (bấm Flight (f) để bật)")
        ttk.Label(flight, textvariable=self.telem_stream_var, foreground="gray").pack(side=tk.LEFT, padx=(8, 0))

        # Chỉ báo Commander heartbeat ('p') — TỰ chạy suốt phiên kết nối (xem
        # _start_cmdr_heartbeat()), KHÔNG cần bấm gì. Hiện ở đây để biết nó
        # đang chạy (mất kết nối/đứng → vòng lặp tự dừng, xem disconnect()).
        self.cmdr_hb_var = tk.StringVar(value="Commander heartbeat: OFF (chua ket noi)")
        ttk.Label(flight, textvariable=self.cmdr_hb_var, foreground="gray").pack(side=tk.LEFT, padx=(8, 0))

        # Điện áp pin — RIÊNG, đậm + đổi màu theo mức, không lẫn trong dòng
        # telemetry dài (trước đây chỉ có "| BAT=...V" ở cuối text, dễ bị bỏ
        # sót). Ngưỡng màu tham chiếu COMMANDER_DEFAULT_BATTERY_FLOOR_V=3.3V
        # (1S LiPo, xem tuning.h mục 5) — đỏ khi CHẠM/DƯỚI sàn auto-land, cam
        # khi đang tụt gần sàn, xanh khi còn khoẻ.
        self.battery_var = tk.StringVar(value="BAT: -")
        self.battery_label = tk.Label(flight, textvariable=self.battery_var,
                                       font=("Segoe UI", 10, "bold"), fg="gray")
        self.battery_label.pack(side=tk.LEFT, padx=(8, 0))

        self.telemetry_var = tk.StringVar(value="ARM=? THR=? | R=? P=? Y=? | valid=?")
        ttk.Label(flight, textvariable=self.telemetry_var, foreground="#0057b3").pack(side=tk.LEFT, padx=(16, 0))

        # Lý do TỪ CHỐI ARM — hàng RIÊNG ngay dưới thanh nút, đỏ + đậm.
        # Không nhồi vào telemetry_var (dòng đó đã rất dài, người đọc bỏ qua) và
        # không để trong notebook (phải luôn thấy được). Đây thường là thông tin
        # duy nhất trả lời được "vì sao tôi không arm được".
        # Trang thai be mat ToF — hang rieng, luon thay duoc.
        self.tof_var = tk.StringVar(value="")
        self.tof_label = tk.Label(self.root, textvariable=self.tof_var,
                                   font=("Segoe UI", 9), fg="gray", anchor="w")
        self.tof_label.pack(side=tk.TOP, fill=tk.X, padx=6)

        self.arm_reject_var = tk.StringVar(value="")
        tk.Label(self.root, textvariable=self.arm_reject_var,
                 font=("Segoe UI", 10, "bold"), fg="#c0392b", anchor="w").pack(
            side=tk.TOP, fill=tk.X, padx=6)

        # ===== THÂN: [NOTEBOOK: Config | Manual Control]  |  LOG (luôn hiện) =====
        # Thanh nút trên (flight) + 4 đồ thị (dựng sau) NẰM NGOÀI notebook -> luôn hiện.
        #
        # Log NẰM NGOÀI notebook (trước đây nó là con của config_tab): khi sang
        # tab Manual Control thì log biến mất hoàn toàn, tức đang bay tay là lúc
        # CẦN xem log nhất lại không xem được. Giờ nó là em cùng cấp của
        # notebook trong `body` nên hiện ở CẢ HAI tab, và cùng một widget nên
        # nội dung không bị reset khi đổi tab.
        #
        # ---- CHIA ĐÔI CỨNG: Log 50% TRÁI | notebook (params) 50% PHẢI ----
        # Dùng place(relwidth=0.5) chứ KHÔNG dùng pack(expand=True) như bản cũ.
        # Lý do: pack chia phần dư ĐỀU NHAU nhưng phần NỀN vẫn theo kích thước
        # mỗi widget tự yêu cầu — mà notebook (3 cột PID + Mahony + Calib) yêu
        # cầu rộng gấp mấy lần ô log, nên log thực tế chỉ được ~1/3 và co lại
        # mỗi khi thêm panel mới vào Config. place() bỏ qua hoàn toàn kích thước
        # yêu cầu -> đúng 50/50, KHÔNG đổi dù sau này thêm bao nhiêu panel.
        #
        # Nửa phải (notebook, cả 2 tab) giờ dùng _make_scrollable() với CẢ
        # thanh cuộn DỌC lẫn NGANG: khoá cứng 50% nghĩa là nội dung params
        # (rộng hơn nửa màn hình) không còn chỗ để "tự nới bề ngang" như bản
        # cũ nữa — phải cuộn để xem hết thay vì bị cắt mất bên phải.
        #
        # body PHẢI có height tường minh + pack_propagate(False): con dùng
        # place() KHÔNG báo kích thước lên cha, nên nếu không đặt thì body xin
        # 0px và pack sẽ bóp nó lại thành vạch mỏng khi cửa sổ chật.
        body = ttk.Frame(self.root, height=560)
        body.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        body.pack_propagate(False)

        self.notebook = ttk.Notebook(body)
        self.notebook.place(relx=0.5, rely=0.0, relwidth=0.5, relheight=1.0)

        config_tab = ttk.Frame(self.notebook)
        self.notebook.add(config_tab, text="Config")
        manual_tab = ttk.Frame(self.notebook)
        self.notebook.add(manual_tab, text="Manual Control")
        self.notebook.bind("<<NotebookTabChanged>>", self._on_tab_changed)

        # ===== Config tab: chỉ còn khu tuning (log đã ra ngoài, xem `body`) =====
        mid = ttk.Frame(config_tab, padding=6)
        mid.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        # Boc trong Canvas+2 Scrollbar (xem _make_scrollable) vi cot nay xep
        # chong RAT nhieu group (3x3 PID + ALT/TKO/LAND + CMDR + TEST_MOTOR +
        # FLIGHT_CMD + Mahony + Calibration) -> vua CAO hon man hinh (can cuon
        # doc) vua RONG hon nua cua so danh cho tab nay (can cuon ngang, tu
        # khi Log/Params bi khoa cung 50/50 - xem _build_ui).
        #
        # fill=BOTH + expand=True (KHONG con fill=Y/expand=False nhu ban cu):
        # container gio phai LAP DAY dung nua-cua-so danh cho no (dieu khien
        # boi place(relwidth=0.5) o notebook ben ngoai), khong con "co lai
        # theo be rong noi dung" nhu truoc — do la co che da chuyen sang
        # thanh cuon ngang (hbar) trong _make_scrollable.
        left_container, left = self._make_scrollable(mid)
        left_container.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Bố cục lưới 3×3 (cột = ROLL/PITCH/YAW):
        #   Hàng 0: ANGLE ROLL   ANGLE PITCH   ANGLE YAW
        #   Hàng 1: RATE ROLL    RATE PITCH    RATE YAW
        #   Hàng 2: ALT CONTROLLER  TAKEOFF    LANDING
        # sticky="new": top-align, không giãn dọc. Panel cao (ALT/TKO/LAND) nằm HÀNG
        # CUỐI nên không đẩy hàng PID -> RATE luôn sát ngay dưới ANGLE.
        groups_frame = ttk.Frame(left)
        groups_frame.pack(side=tk.TOP, anchor="w")
        for c in range(3):
            groups_frame.columnconfigure(c, uniform="setcols")

        # Hàng 0-1: PID cascade (ANGLE/RATE × ROLL/PITCH/YAW) — có cả ANGLE YAW.
        for col, axis in enumerate(PID_AXES):
            for row, loop in enumerate(PID_LOOPS):
                g = GainGroup(groups_frame, loop, axis, self.send_pid_set)
                g.frame.grid(row=row, column=col, padx=4, pady=4, sticky="new")
                self.groups[(loop, axis)] = g

        # Hàng 2: ALT controller | Takeoff | Landing.
        self.flight_alt = FlightAltGroup(
            groups_frame,
            self.send_flight_alt_set,
            self.get_flight_alt,
            self.send_flight_alt_mode,
            lambda d: self.send_raw_cmd(">" if d > 0 else "<"),
        )
        self.flight_alt.frame.grid(row=2, column=0, padx=4, pady=4, sticky="new")

        self.tko = TakeoffGroup(groups_frame, self.send_flight_tko_set,
                                self.get_flight_tko, self.send_flight_takeoff)
        self.tko.frame.grid(row=2, column=1, padx=4, pady=4, sticky="new")

        self.land = LandingGroup(groups_frame, self.send_flight_land_set,
                                 self.get_flight_land, self.send_flight_landing)
        self.land.frame.grid(row=2, column=2, padx=4, pady=4, sticky="new")

        # Hàng 3: Commander/Failsafe (@CMDR) — geofence + ngưỡng fault, chạy
        # được BẤT KỲ LÚC NÀO (kể cả đang bay), riêng hàng vì rộng hơn (6 field).
        self.cmdr = CommanderGroup(groups_frame, self.send_flight_cmdr_set,
                                   self.get_flight_cmdr)
        self.cmdr.frame.grid(row=3, column=0, columnspan=3, padx=4, pady=4, sticky="new")

        # Hàng 4: Test Motor (@TEST) — bench-only, xem TestMotorGroup.
        self.test_motor = TestMotorGroup(groups_frame, self.send_test_motor)
        self.test_motor.frame.grid(row=4, column=0, columnspan=3, padx=4, pady=4, sticky="new")

        # Hàng 5: Flight Commands (@HOVER/@MOVE/@YAW) — xem FlightCommandGroup.
        self.flight_cmd = FlightCommandGroup(
            groups_frame, self.send_flight_hover, self.send_flight_move, self.send_flight_yaw)
        self.flight_cmd.frame.grid(row=5, column=0, columnspan=3, padx=4, pady=4, sticky="new")

        # Mahony filter + angle target, dưới lưới PID (vẫn thuộc cột TRÁI).
        self._build_mahony_panel(left)

        # Calibration (@CAL, mới) — dưới Mahony, vẫn cột TRÁI.
        self.calib = CalibrationGroup(left, self.send_cal_cmd)
        self.calib.frame.pack(side=tk.TOP, fill=tk.X, pady=(0, 6))

        # ===== Log (bên TRÁI notebook) — LUÔN HIỆN ở cả 2 tab =====
        # parent = `body` (ngoài notebook), KHÔNG phải `mid`/config_tab.
        # CHIẾM CỐ ĐỊNH nửa TRÁI cửa sổ (relwidth=0.5, xem khối comment ở `body`).
        # width=48/height=36 giờ chỉ còn ý nghĩa lúc widget chưa được map — kích
        # thước thật do place() quyết định, không do 2 số này.
        log_frame = ttk.Frame(body)
        log_frame.place(relx=0.0, rely=0.0, relwidth=0.5, relheight=1.0)
        ttk.Label(log_frame, text="Log (ESP32 + GUI):").pack(side=tk.TOP, anchor="w")
        self.log_text = scrolledtext.ScrolledText(log_frame, width=48, height=36,
                                                  state="disabled", wrap="none")
        self.log_text.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        # ===== Tab Manual Control (mới) =====
        self._build_manual_tab(manual_tab)

        # ===== 4 đồ thị NGOÀI notebook (luôn hiện) — dựng SAU cùng =====
        self._build_plots()

        # Bind phím điều khiển tay + watchdog an toàn (chỉ tác dụng khi ở tab Manual).
        self._bind_manual_keys()

    def _build_mahony_panel(self, parent):
        # 3 cột gọn: Mahony (Kp/Ki) | Angle target (R/P/Y) | Trim (roll/pitch).
        frame = ttk.Frame(parent)
        frame.pack(side=tk.TOP, fill=tk.X, pady=(0, 6))
        SL = 90   # slider ngắn cho gọn

        # Cột 0: Mahony filter gain.
        ff = ttk.LabelFrame(frame, text="Mahony")
        ff.pack(side=tk.LEFT, anchor="n", padx=(2, 4))
        self.mah_kp = GainRow(ff, 0, "Kp", (0.0, 5.0), self.send_mahony_set,
                              slider_len=SL, label_width=4)
        self.mah_ki = GainRow(ff, 1, "Ki", (0.0, 2.0), self.send_mahony_set,
                              slider_len=SL, label_width=4)
        mbf = ttk.Frame(ff)
        mbf.grid(row=2, column=0, columnspan=3, sticky="ew", padx=4, pady=(2, 2))
        ttk.Button(mbf, text="Set", command=self.send_mahony_set).pack(side=tk.LEFT, expand=True, fill=tk.X)
        ttk.Button(mbf, text="Get", command=self.get_mahony).pack(side=tk.LEFT, padx=(4, 0))
        self.mah_status_var = tk.StringVar(value="")
        ttk.Label(ff, textvariable=self.mah_status_var, foreground="gray").grid(
            row=3, column=0, columnspan=3, sticky="w", padx=4)

        # Cột 1: Setpoint @SP — Roll/Pitch = GÓC (deg, cộng lên trim), Yaw = yaw-RATE
        # (dps). Firmware LUÔN coi tham số yaw của @SP SET là TỐC ĐỘ, không phải góc.
        tf = ttk.LabelFrame(frame, text="SP: R/P deg | Yaw dps")
        tf.pack(side=tk.LEFT, anchor="n", padx=4)
        # Nhãn "Roll"/"Pitch" khớp quy ước ĐÃ ĐỔI của firmware (roll=tiến/lùi,
        # pitch=trái/phải — xem CMD_SET_ATTITUDE trong flight_core.c), KHÔNG
        # phải nghĩa hàng không chuẩn.
        self.target_roll = GainRow(tf, 0, "Roll (F/B)", (-30.0, 30.0), self.send_target_set,
                                   slider_len=SL, label_width=9)
        self.target_pitch = GainRow(tf, 1, "Pitch (L/R)", (-30.0, 30.0), self.send_target_set,
                                    slider_len=SL, label_width=9)
        # yaw-RATE (dps), KHÔNG phải góc -> thang ±90 dps cho hợp lý (firmware clamp ±180).
        self.target_yaw = GainRow(tf, 2, "Yaw dps", (-90.0, 90.0), self.send_target_set,
                                  slider_len=SL, label_width=5)
        tbf = ttk.Frame(tf)
        tbf.grid(row=3, column=0, columnspan=3, sticky="ew", padx=4, pady=(2, 2))
        ttk.Button(tbf, text="Set", command=self.send_target_set).pack(side=tk.LEFT, expand=True, fill=tk.X)
        ttk.Button(tbf, text="0", command=self.zero_target, width=3).pack(side=tk.LEFT, padx=(4, 0))
        self.target_status_var = tk.StringVar(value="")
        ttk.Label(tf, textvariable=self.target_status_var, foreground="gray").grid(
            row=4, column=0, columnspan=3, sticky="w", padx=4)

        # Cột 2: TRIM roll/pitch (@TRIM) — bù lệch cơ khí/CG để hover thẳng.
        trf = ttk.LabelFrame(frame, text="Trim (deg) @TRIM")
        trf.pack(side=tk.LEFT, anchor="n", padx=(4, 2))
        self.trim_roll = GainRow(trf, 0, "roll", (-10.0, 10.0), self.send_flight_trim_set,
                                 slider_len=SL, label_width=5)
        self.trim_pitch = GainRow(trf, 1, "pitch", (-10.0, 10.0), self.send_flight_trim_set,
                                  slider_len=SL, label_width=5)
        trbf = ttk.Frame(trf)
        trbf.grid(row=2, column=0, columnspan=3, sticky="ew", padx=4, pady=(2, 2))
        ttk.Button(trbf, text="Set", command=self.send_flight_trim_set).pack(side=tk.LEFT, expand=True, fill=tk.X)
        ttk.Button(trbf, text="Get", command=self.get_flight_trim).pack(side=tk.LEFT, padx=(4, 0))
        self.trim_status_var = tk.StringVar(value="")
        ttk.Label(trf, textvariable=self.trim_status_var, foreground="gray").grid(
            row=3, column=0, columnspan=3, sticky="w", padx=4)

    def _build_plots(self):
        # 6 đồ thị live (angle / gyro / accel / altitude / Vz / Az world) —
        # parent = self.root nên NẰM NGOÀI notebook, luôn hiển thị bất kể đang
        # ở tab Config hay Manual, dữ liệu KHÔNG reset khi đổi tab (cùng object
        # PlotPanel). Hàng 2 (Vz/Az) phục vụ soi accel-primary altitude
        # estimator (xem alt_estimator.h) — KHÔNG gộp throttle vào cùng panel
        # với Az (thang giá trị lệch cả trăm-nghìn lần, auto-scale 1 trục Y sẽ
        # ép Az thành 1 đường phẳng vô nghĩa); throttle vẫn xem qua nhãn
        # "THR=" trên telemetry_var.
        plots = ttk.Frame(self.root)
        plots.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=6, pady=(0, 4))
        plots.rowconfigure(0, weight=1)
        plots.rowconfigure(1, weight=1)
        plots.columnconfigure(0, weight=1, uniform="plots")
        plots.columnconfigure(1, weight=1, uniform="plots")
        plots.columnconfigure(2, weight=1, uniform="plots")
        plots.columnconfigure(3, weight=1, uniform="plots")

        # Symmetric padx (1px) để khe hở nằm GIỮA các đồ thị, không làm cái nào hẹp hơn.
        self.plot_angle = PlotPanel(plots, PLOT_ANGLE_CHANNELS, "Angle (deg)")
        self.plot_angle.frame.grid(row=0, column=0, sticky="nsew", padx=(0, 1), pady=(0, 1))

        self.plot_gyro = PlotPanel(plots, PLOT_GYRO_CHANNELS, "Gyro rate (dps)")
        self.plot_gyro.frame.grid(row=0, column=1, sticky="nsew", padx=(1, 1), pady=(0, 1))

        self.plot_acc = PlotPanel(plots, PLOT_ACC_CHANNELS, "Accel (g)")
        self.plot_acc.frame.grid(row=0, column=2, sticky="nsew", padx=(1, 1), pady=(0, 1))

        self.plot_alt = PlotPanel(plots, PLOT_ALT_CHANNELS, "Altitude (m)")
        self.plot_alt.frame.grid(row=0, column=3, sticky="nsew", padx=(1, 0), pady=(0, 1))

        self.plot_vz = PlotPanel(plots, PLOT_VZ_CHANNELS, "Vz (m/s)")
        self.plot_vz.frame.grid(row=1, column=0, columnspan=2, sticky="nsew", padx=(0, 1), pady=(1, 0))

        self.plot_az = PlotPanel(plots, PLOT_AZ_CHANNELS, "Az world (m/s^2)")
        self.plot_az.frame.grid(row=1, column=2, columnspan=2, sticky="nsew", padx=(1, 0), pady=(1, 0))

    # ---------------- Manual Control tab ----------------

    def _build_manual_tab(self, parent):
        self._manual_tab_id = str(parent)
        # Cuon duoc nhu tab Config (xem _make_scrollable) - man hinh thap se
        # cat mat nut Trim/Takeoff/Land/Panic phia duoi neu khong cuon.
        outer_container, outer = self._make_scrollable(parent)
        outer_container.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        outer.configure(padding=10)

        tk.Label(
            outer,
            text=("CANH BAO: Nha phim = VE THANG BANG, KHONG phai dung lai. "
                  "Nhan phim NGUOC de phanh. Drone khong co cam bien ngang -> se TROI."),
            fg="#a00000", font=("Segoe UI", 10, "bold"),
            wraplength=1000, justify="left").pack(side=tk.TOP, anchor="w", pady=(0, 8))

        self.manual_status_var = tk.StringVar(value="ARM=?  mode=?  alt=?m")
        ttk.Label(outer, textvariable=self.manual_status_var, foreground="#0057b3",
                  font=("Segoe UI", 11, "bold")).pack(side=tk.TOP, anchor="w")
        self.manual_cmd_var = tk.StringVar()
        ttk.Label(outer, textvariable=self.manual_cmd_var,
                  font=("Consolas", 11)).pack(side=tk.TOP, anchor="w")
        self.manual_brake_var = tk.StringVar(value="")
        ttk.Label(outer, textvariable=self.manual_brake_var,
                  font=("Segoe UI", 11, "bold"), foreground="#c0007a").pack(
                      side=tk.TOP, anchor="w")
        self._update_manual_cmd_label()
        # Throttle từng động cơ (M1..M4) từ telemetry.
        self.manual_motor_var = tk.StringVar(value="M1=--  M2=--  M3=--  M4=--")
        ttk.Label(outer, textvariable=self.manual_motor_var,
                  font=("Consolas", 11), foreground="#7a3d00").pack(
                      side=tk.TOP, anchor="w", pady=(0, 10))

        steps = ttk.LabelFrame(outer, text="Steps")
        steps.pack(side=tk.TOP, anchor="w", pady=(0, 10))
        self.tilt_step_var = tk.DoubleVar(value=4.0)
        # NANG 30 -> 90 deg/s (yeu cau nguoi dung). Nam trong tran firmware
        # SP_YAW_RATE_MAX_DPS = 180 (tuning.h muc 6) nen KHONG bi clamp am tham.
        #
        # ⚠ 90 deg/s la nhanh: giu A/D mot giay la drone quay 1/4 vong. Yaw
        # nhanh cung an vao bien ga cua mixer (yaw_correction cong/tru vao ca 4
        # motor), nen o ga cao co the cham tran collective va mat mot phan tham
        # quyen roll/pitch. Neu thay drone kem on dinh khi vua yaw vua bay ngang
        # thi ha so nay xuong ~45-60.
        self.yaw_step_var = tk.DoubleVar(value=90.0)
        ttk.Label(steps, text="TILT (deg):").grid(row=0, column=0, padx=4, pady=3, sticky="e")
        tk.Spinbox(steps, from_=1.0, to=10.0, increment=0.5, width=6,
                   textvariable=self.tilt_step_var).grid(row=0, column=1, padx=(0, 14))
        ttk.Label(steps, text="YAW rate (deg/s):").grid(row=0, column=2, padx=4, pady=3, sticky="e")
        tk.Spinbox(steps, from_=10.0, to=90.0, increment=5.0, width=6,
                   textvariable=self.yaw_step_var).grid(row=0, column=3, padx=(0, 14))
        ttk.Label(steps, text="(ALT step co dinh 10cm)", foreground="gray").grid(row=0, column=4, padx=4)

        # ---- Auto-brake (roll/pitch): nhả phím -> nghiêng ngược 1 nhịp để phanh ----
        brakef = ttk.LabelFrame(outer, text="Auto-brake (roll/pitch)")
        brakef.pack(side=tk.TOP, anchor="w", pady=(0, 10))
        self._autobrake_var = tk.BooleanVar(value=True)
        self.brake_k_var = tk.DoubleVar(value=0.9)
        self.brake_ratio_var = tk.DoubleVar(value=0.8)
        self.brake_min_var = tk.DoubleVar(value=0.10)
        self.brake_max_var = tk.DoubleVar(value=1.00)
        ttk.Checkbutton(brakef, text="Auto-brake", variable=self._autobrake_var).grid(
            row=0, column=0, padx=4, pady=3, sticky="w")

        def _brake_sb(col, label, var, lo, hi, inc):
            ttk.Label(brakef, text=label).grid(row=0, column=col, padx=(12, 2),
                                               pady=3, sticky="e")
            tk.Spinbox(brakef, from_=lo, to=hi, increment=inc, width=6,
                       textvariable=var).grid(row=0, column=col + 1, padx=(0, 4))

        # K: brake_time = K*hold_time (trôi tiếp->tăng K, lùi ngược->giảm K)
        _brake_sb(1, "K:", self.brake_k_var, 0.0, 3.0, 0.1)
        # ratio: góc phanh = ratio*TILT_STEP (nhỏ hơn cho đỡ giật)
        _brake_sb(3, "ratio:", self.brake_ratio_var, 0.0, 1.0, 0.05)
        _brake_sb(5, "min(s):", self.brake_min_var, 0.02, 2.0, 0.02)
        _brake_sb(7, "max(s):", self.brake_max_var, 0.1, 3.0, 0.1)

        pad = ttk.Frame(outer)
        pad.pack(side=tk.TOP, anchor="w")

        yawf = ttk.LabelFrame(pad, text="Yaw (A/D) - giu = xoay, nha = dung")
        yawf.grid(row=0, column=0, padx=6, pady=4, sticky="n")
        self._mk_dir_btn(yawf, "yaw_l", "A\n<xoay").grid(row=0, column=0, padx=2, pady=2)
        self._mk_dir_btn(yawf, "yaw_r", "D\nxoay>").grid(row=0, column=1, padx=2, pady=2)

        rpf = ttk.LabelFrame(pad, text="Roll/Pitch (mui ten) - giu = nghieng, nha = ve 0")
        rpf.grid(row=0, column=1, padx=6, pady=4, sticky="n")
        self._mk_dir_btn(rpf, "up",    "^\ntien").grid(row=0, column=1, padx=2, pady=2)
        self._mk_dir_btn(rpf, "left",  "<\ntrai").grid(row=1, column=0, padx=2, pady=2)
        self._mk_dir_btn(rpf, "down",  "v\nlui").grid(row=1, column=1, padx=2, pady=2)
        self._mk_dir_btn(rpf, "right", ">\nphai").grid(row=1, column=2, padx=2, pady=2)

        # ====================================================================
        # W/S — nhan va hanh vi PHU THUOC MODE, cap nhat tu STATUS
        # ====================================================================
        # ⚠ NUT BAM PHAI DI QUA CUNG DUONG VOI PHIM (_ws_press/_stop_ws_repeat).
        # Truoc day nut goi THANG self._alt_step(), bo qua toan bo logic re nhanh
        # theo mode -> bam nut o FLYING van gui buoc do cao (vo tac dung vi FLYING
        # khong dung target), trong khi giu phim W lai gui offset ga. Cung mot
        # thao tac, hai ket qua khac nhau, khong co gi tren man hinh giai thich.
        #
        # Nhan cung KHONG duoc hard-code: no phai noi dung cai dang xay ra o mode
        # HIEN TAI, neu khong thi no la mot lop noi doi thu hai chong len lop dau.
        self.alt_frame = ttk.LabelFrame(pad, text="Do cao / Ga")
        altf = self.alt_frame
        altf.grid(row=0, column=2, padx=6, pady=4, sticky="n")
        # <ButtonPress>/<ButtonRelease> chu khong phai command=: o FLYING, W/S la
        # lenh GIU (momentary) nen phai biet luc nha nut, ma command= chi bao
        # "da click xong".
        self.btn_w = tk.Button(altf, text="W", width=10, height=3)
        self.btn_w.grid(row=0, column=0, padx=2, pady=2)
        self.btn_w.bind("<ButtonPress-1>",   lambda e: self._ws_press(+1))
        self.btn_w.bind("<ButtonRelease-1>", lambda e: self._stop_ws_repeat())
        self.btn_s = tk.Button(altf, text="S", width=10, height=3)
        self.btn_s.grid(row=1, column=0, padx=2, pady=2)
        self.btn_s.bind("<ButtonPress-1>",   lambda e: self._ws_press(-1))
        self.btn_s.bind("<ButtonRelease-1>", lambda e: self._stop_ws_repeat())
        # Dong nhan dong: _update_ws_labels() ghi de moi khi STATUS doi mode.
        self.ws_hint_var = tk.StringVar(value="cho STATUS...")
        ttk.Label(altf, textvariable=self.ws_hint_var,
                  foreground="gray").grid(row=2, column=0, padx=2, pady=(2, 2))
        self._update_ws_labels(None)

        tlf = ttk.LabelFrame(pad, text="Takeoff / Land")
        tlf.grid(row=0, column=3, padx=6, pady=4, sticky="n")
        self.manual_takeoff_btn = tk.Button(
            tlf, text="TAKEOFF (T)", width=12, height=2,
            bg="#0a7d2c", fg="white", activebackground="#0c9235",
            font=("Segoe UI", 9, "bold"), command=self.send_flight_takeoff)
        self.manual_takeoff_btn.grid(row=0, column=0, padx=2, pady=2)
        tk.Button(tlf, text="LAND (L)", width=12, height=2,
                  bg="#b5651d", fg="white", activebackground="#c9761f",
                  font=("Segoe UI", 9, "bold"),
                  command=self.send_flight_landing).grid(row=1, column=0, padx=2, pady=2)

        # Label RIÊNG cho pha cất cánh (TKOP trong STATUS) — tách khỏi dòng
        # telemetry dài (manual_status_var) để KHÔNG bị lẫn giữa lúc đang bận
        # theo dõi drone thật. Đổi màu/chữ theo _TKO_PHASE_STYLE ngay khi
        # nhận STATUS mới (xem _handle_line -> _update_takeoff_phase_label).
        self.manual_tko_var = tk.StringVar(value="TAKEOFF: -")
        self.manual_tko_label = tk.Label(
            tlf, textvariable=self.manual_tko_var, font=("Segoe UI", 10, "bold"),
            fg="gray", anchor="w")
        self.manual_tko_label.grid(row=2, column=0, padx=2, pady=(6, 2), sticky="ew")

        # ---- Trim live (@TRIM): dò bias roll/pitch ngay khi bay, mỗi bước 0.02 deg ----
        # Dùng CHUNG state trim_roll/trim_pitch (panel Config) làm nguồn -> Config &
        # Manual luôn đồng bộ. Mỗi nút nudge -> gửi @TRIM SET ngay.
        # ====================================================================
        # TRIM — KHOA KHI DANG BAY (yeu cau nguoi dung)
        # ====================================================================
        # Trim la phep do BIAS CO KHI (drone lech ve mot huong khi le ra phai
        # dung yen). Do no khi DANG BAY la sai ban chat: luc do drone dang chiu
        # gio, hieu ung mat dat, va chinh lenh nghieng cua nguoi lai — khong
        # phan biet duoc "lech do lap dat" voi "lech do dang bay".
        #
        # Te hon: moi lan bam la mot buoc nhay setpoint goc. O tren khong, mot
        # buoc 0.02deg khong sao, nhung giu nut / bam nham nhieu lan se dich
        # setpoint di dang ke ma khong co gi keo lai.
        #
        # Giu lai nut (KHONG xoa) vi luc DISARMED / dat tren ban chung van la
        # cach dung de do bias. Chi khoa khi ARM.
        self.trim_frame = ttk.LabelFrame(pad, text="Trim live (@TRIM) - do bias")
        trimf = self.trim_frame
        trimf.grid(row=0, column=4, padx=6, pady=4, sticky="n")
        self._trim_widgets = []
        self.trim_step_var = tk.DoubleVar(value=0.02)
        self.manual_trim_var = tk.StringVar(value="roll=?  pitch=?")
        ttk.Label(trimf, textvariable=self.manual_trim_var,
                  font=("Consolas", 10)).grid(row=0, column=0, columnspan=3,
                                              sticky="w", padx=2, pady=(2, 4))
        ttk.Label(trimf, text="roll (q/e)").grid(row=1, column=0, padx=2, sticky="e")
        _b = tk.Button(trimf, text="-", width=3,
                       command=lambda: self._trim_nudge("roll", -1))
        _b.grid(row=1, column=1, padx=1); self._trim_widgets.append(_b)
        _b = tk.Button(trimf, text="+", width=3,
                       command=lambda: self._trim_nudge("roll", +1))
        _b.grid(row=1, column=2, padx=1); self._trim_widgets.append(_b)
        ttk.Label(trimf, text="pitch (1/3)").grid(row=2, column=0, padx=2, sticky="e")
        _b = tk.Button(trimf, text="-", width=3,
                       command=lambda: self._trim_nudge("pitch", -1))
        _b.grid(row=2, column=1, padx=1); self._trim_widgets.append(_b)
        _b = tk.Button(trimf, text="+", width=3,
                       command=lambda: self._trim_nudge("pitch", +1))
        _b.grid(row=2, column=2, padx=1); self._trim_widgets.append(_b)
        ttk.Label(trimf, text="step").grid(row=3, column=0, padx=2, pady=(4, 0), sticky="e")
        tk.Spinbox(trimf, from_=0.01, to=0.50, increment=0.01, width=5,
                   textvariable=self.trim_step_var).grid(row=3, column=1, columnspan=2,
                                                         padx=1, pady=(4, 0))
        ttk.Button(trimf, text="Get", width=6, command=self.get_flight_trim).grid(
            row=4, column=0, columnspan=3, pady=(4, 0), sticky="ew")
        self._update_manual_trim_label()

        tk.Button(outer,
                  text="PANIC / LEVEL (Space)  -  ve thang bang, giu do cao (KHONG cat motor)",
                  bg="#f0a000", fg="black", activebackground="#ffb51a",
                  font=("Segoe UI", 11, "bold"),
                  command=self._panic_level).pack(side=tk.TOP, anchor="w", fill=tk.X, pady=(12, 4))

        ttk.Label(outer, foreground="gray",
                  text="Meo: neu vua go spinbox thi click vao vung nay de GUI bat lai phim. "
                       "KILL (Esc / nut do) luon san sang.").pack(side=tk.TOP, anchor="w")

        # Focus sink: nhận focus để phím không bị spinbox nuốt; click nền -> lấy focus về.
        self._focus_sink_w = tk.Frame(outer, width=1, height=1, takefocus=1)
        self._focus_sink_w.pack(side=tk.TOP)
        for w in (outer, pad):
            w.bind("<Button-1>", lambda _e: self._focus_sink())

    def _mk_dir_btn(self, parent, name, text):
        b = tk.Button(parent, text=text, width=8, height=3)
        b._def_bg = b.cget("background")
        b.bind("<ButtonPress-1>", lambda _e, n=name: self._dir_press(n, True))
        b.bind("<ButtonRelease-1>", lambda _e, n=name: self._dir_press(n, False))
        self._manual_btns[name] = b
        return b

    # ---- momentary roll/pitch/yaw ----
    def _dir_press(self, name, active):
        token = self._dir_token[name]
        axis = token[:-1]            # "roll" / "pitch" / "yaw"
        if active:
            # QUY TẮC 1: bất kỳ phím hướng nào được NHẤN -> hủy MỌI phanh đang chạy,
            # người lái luôn thắng, xử lý lệnh mới ngay. Ghi mốc giữ để tính brake.
            self._cancel_all_brakes()
            self._hold_start[name] = time.time()
            self._held.add(token)
        else:
            self._held.discard(token)
            # Auto-brake CHỈ cho roll/pitch (KHÔNG yaw). Bỏ qua nếu tắt, hoặc trục
            # vẫn còn phím giữ (lệnh giữ thắng phanh).
            if (axis in ("roll", "pitch") and self._autobrake_on()
                    and not self._axis_held(axis)):
                self._start_brake(name, token, axis)
        self._highlight(name, active)
        self._recompute_cmd()
        self._send_sp()
        self._update_brake_label()

    def _recompute_cmd(self):
        tilt = self._get_step(self.tilt_step_var, 4.0)
        yaw = self._get_step(self.yaw_step_var, 90.0)
        h = self._held
        roll_held = (tilt if "roll+" in h else 0.0) - (tilt if "roll-" in h else 0.0)
        pitch_held = (tilt if "pitch+" in h else 0.0) - (tilt if "pitch-" in h else 0.0)
        # Phím GIỮ thắng phanh; khi không giữ mà đang phanh -> dùng góc phanh (ngược).
        self.cmd_roll = roll_held if roll_held != 0.0 else self._brake_cmd["roll"]
        self.cmd_pitch = pitch_held if pitch_held != 0.0 else self._brake_cmd["pitch"]
        self.cmd_yaw = (yaw if "yaw+" in h else 0.0) - (yaw if "yaw-" in h else 0.0)

    # ---- auto-brake helpers ----
    def _autobrake_on(self):
        try:
            return bool(self._autobrake_var.get())
        except Exception:
            return True

    def _axis_held(self, axis):
        return (axis + "+") in self._held or (axis + "-") in self._held

    def _start_brake(self, name, token, axis):
        # brake_time = clamp(K*hold_time, min, max). Nghiêng NGƯỢC hướng vừa nhả,
        # độ lớn = ratio*TILT_STEP. Kết thúc bằng after() (không sleep -> không block).
        hold_time = time.time() - self._hold_start.get(name, time.time())
        k = self._get_step(self.brake_k_var, 0.6)
        ratio = self._get_step(self.brake_ratio_var, 0.7)
        bmin = self._get_step(self.brake_min_var, 0.10)
        bmax = self._get_step(self.brake_max_var, 1.00)
        brake_time = min(max(k * hold_time, bmin), bmax)
        tilt = self._get_step(self.tilt_step_var, 4.0)
        sign = 1.0 if token.endswith("+") else -1.0
        self._brake_cmd[axis] = -sign * tilt * ratio     # ngược dấu hướng vừa nhả
        self._brake_secs[axis] = brake_time
        self._brake_after[axis] = self.root.after(
            int(brake_time * 1000), lambda a=axis: self._end_brake(a))

    def _end_brake(self, axis):
        self._brake_after[axis] = None
        self._brake_cmd[axis] = 0.0
        self._recompute_cmd()
        self._send_sp()
        self._update_brake_label()

    def _cancel_brake(self, axis):
        if self._brake_after[axis] is not None:
            self.root.after_cancel(self._brake_after[axis])
            self._brake_after[axis] = None
        self._brake_cmd[axis] = 0.0

    def _cancel_all_brakes(self):
        for ax in ("roll", "pitch"):
            self._cancel_brake(ax)

    def _update_brake_label(self):
        if not hasattr(self, "manual_brake_var"):
            return
        parts = [f"{ax} ({self._brake_secs[ax]:.2f}s)"
                 for ax in ("roll", "pitch") if self._brake_after[ax] is not None]
        self.manual_brake_var.set(("BRAKING " + ", ".join(parts)) if parts else "")

    def _get_step(self, var, default):
        try:
            return float(var.get())
        except Exception:
            return default

    def _highlight(self, name, active):
        b = self._manual_btns.get(name)
        if b is not None:
            b.configure(bg="#7ec8ff" if active else b._def_bg)

    def _send_sp(self):
        # Gửi NGAY khi cmd đổi (nhấn/nhả phím) VÀ ghi lại tracker để heartbeat
        # 100ms sau không gửi lại cùng giá trị (BUG 2: chống double-send).
        if self.console is not None:
            cur = (self.cmd_roll, self.cmd_pitch, self.cmd_yaw)
            self.console.send_command(f"@SP SET {cur[0]:.2f} {cur[1]:.2f} {cur[2]:.2f}")
            self._last_sent_sp = cur
            self._last_sp_time = time.time()
        self._update_manual_cmd_label()

    def _update_manual_cmd_label(self):
        if not hasattr(self, "manual_cmd_var"):
            return
        tgt = "--" if self._last_alt_target is None else f"{self._last_alt_target}m"
        self.manual_cmd_var.set(
            f"cmd_roll={self.cmd_roll:+.1f}  cmd_pitch={self.cmd_pitch:+.1f}  "
            f"cmd_yaw_rate={self.cmd_yaw:+.1f}   |   alt_target={tgt}")

    # ---- trim live (@TRIM) — dò bias roll/pitch trong tab Manual ----
    def _trim_nudge(self, axis, sign):
        # Cộng/trừ step (mặc định 0.02 deg) vào trim của trục rồi gửi @TRIM SET NGAY.
        # Dùng chung GainRow trim_roll/trim_pitch (panel Config) làm nguồn dữ liệu.
        step = self._get_step(self.trim_step_var, 0.02)
        row = self.trim_roll if axis == "roll" else self.trim_pitch
        newv = min(max(row.get_value() + sign * step, -10.0), 10.0)
        row.set_value(newv)
        self.send_flight_trim_set()          # gửi cả roll+pitch (đủ 2 token firmware)
        self._update_manual_trim_label()
        self._focus_sink()                   # trả focus về sink -> phím bay chạy tiếp

    def _update_manual_trim_label(self):
        if not hasattr(self, "manual_trim_var"):
            return
        self.manual_trim_var.set(
            f"roll={self.trim_roll.get_value():+.2f}  "
            f"pitch={self.trim_pitch.get_value():+.2f}")

    # ---- điện áp pin (BATV/BCOMP trong STATUS) — label riêng trên thanh top ----
    def _update_battery_label(self, batv, bcomp, batvalid=None, batvraw=None,
                               hovlk=None, hovlv=None, hovld=None):
        if not hasattr(self, "battery_var"):
            return
        if batv is None:
            self.battery_var.set("BAT: -")
            self.battery_label.configure(fg="gray")
            return
        # Mẫu bị firmware loại (ngoài dải 1S hoặc thiếu ADC calibration, xem
        # battery_driver.h): BATV đầu dòng đã bị ép 0.00 nên hiện nó là vô
        # nghĩa — hiện SỐ THÔ kèm chữ INVALID mới nói đúng chuyện gì đang xảy
        # ra (vd đọc 6.07V trên pin 1S = lắp nhầm pin/sai điện trở chia áp).
        if batvalid == "0":
            raw_txt = f" ({batvraw}V thô)" if batvraw is not None else ""
            self.battery_var.set(f"BAT: INVALID{raw_txt}")
            self.battery_label.configure(fg="#c0392b")
            return
        try:
            v = float(batv)
        except (TypeError, ValueError):
            self.battery_var.set("BAT: -")
            self.battery_label.configure(fg="gray")
            return
        # Ngưỡng màu — xem comment lúc tạo widget (khớp COMMANDER_DEFAULT_
        # BATTERY_FLOOR_V=3.3V mặc định cho 1S, tuning.h mục 5).
        if v <= 3.3:
            color = "#c0392b"    # đỏ — chạm/dưới sàn auto-land
        elif v <= 3.7:
            color = "#b35c00"    # cam — đang tụt gần sàn
        else:
            color = "#0a7d2c"    # xanh — còn khoẻ
        comp_txt = ""
        try:
            if bcomp is not None and abs(float(bcomp) - 1.0) > 1e-3:
                comp_txt = f" (comp={bcomp}x)"
        except (TypeError, ValueError):
            pass
        # Ga hover DA CHOT theo pin (hover_model.h) — hien ngay canh dien ap vi
        # hai so nay chi co nghia khi doc CUNG NHAU: hover_ff duoc suy ra tu
        # dung dien ap KHONG TAI luc ARM, khong phai tu BATV hien tai (BATV
        # dang tut theo tai motor, do la LY DO latch chi chot mot lan).
        latch_txt = ""
        if hovlk == "1":
            try:
                latch_txt = f" | hover@{float(hovlv):.2f}V={float(hovld):.0f}"
            except (TypeError, ValueError):
                latch_txt = ""
        self.battery_var.set(f"BAT: {v:.2f}V{comp_txt}{latch_txt}")
        self.battery_label.configure(fg=color)

    # ---- lý do từ chối ARM (ARMREJ trong STATUS) ----
    def _update_arm_reject(self, armrej, armrseq):
        """Hiện VÌ SAO ARM bị từ chối, ngay trên thanh nút.

        Không có cái này thì GUI hoàn toàn mù: firmware log lý do bằng ESP_LOGW
        nên chỉ console USB thấy. Người dùng bấm ARM qua WiFi -> không arm, không
        biết tại sao.

        Ghi vào Log MỘT LẦN mỗi lần bị từ chối MỚI (theo ARMRSEQ) để nó nằm lại
        trong lịch sử, thay vì chỉ nhấp nháy trên label rồi mất.
        """
        if not hasattr(self, "arm_reject_var"):
            return
        if armrej is None or armrej == "0":
            self.arm_reject_var.set("")
            return
        txt = ARM_REJECT_NAMES.get(armrej, f"ma {armrej} (firmware moi hon GUI?)")
        self.arm_reject_var.set(f"ARM BI TU CHOI: {txt}")
        if armrseq is not None and armrseq != getattr(self, "_last_armrseq", None):
            self._last_armrseq = armrseq
            self._log(f"[ARM] TU CHOI (lan {armrseq}): {txt}")

    # ---- lý do từ chối TAKEOFF (TKOREJ trong STATUS) ----
    def _update_takeoff_reject(self, tkorej, tkorseq):
        """Hiện VÌ SAO chuỗi cất cánh không khởi động.

        Dùng CHUNG nhãn với pha cất cánh (manual_tko_var) chứ không thêm nhãn
        mới: hai thứ này KHÔNG BAO GIỜ đúng cùng lúc — bị từ chối nghĩa là pha
        vẫn IDLE. Đặt cùng chỗ để mắt chỉ phải nhìn một nơi khi bấm TAKEOFF.

        LATCH cho tới lần bấm sau (firmware xoá TKOREJ về 0 ở đầu mỗi lệnh
        TAKEOFF mới): nếu không latch thì lý do bị pha IDLE ghi đè ngay tick sau
        và người dùng không kịp đọc.
        """
        if tkorej is None or tkorej == "0":
            self._tko_reject_latch = None
            return
        txt = TAKEOFF_REJECT_NAMES.get(tkorej, f"ma {tkorej} (firmware moi hon GUI?)")
        self._tko_reject_latch = txt
        if hasattr(self, "manual_tko_var"):
            self.manual_tko_var.set(f"TAKEOFF BI TU CHOI: {txt}")
            self.manual_tko_label.configure(fg="#c00000")
        if hasattr(self, "tko") and hasattr(self.tko, "status_var"):
            self.tko.status_var.set(f"BI TU CHOI: {txt}")
        if tkorseq is not None and tkorseq != getattr(self, "_last_tkorseq", None):
            self._last_tkorseq = tkorseq
            self._log(f"[TAKEOFF] TU CHOI (lan {tkorseq}): {txt}")

    # ---- ToF: be mat dang nhin thay (san hay ban?) ----
    # Nguong suc khoe ToF — khop SENSOR_TOF_STALE_US (200ms) ben firmware.
    TOF_STALE_MS = 200

    def _tof_health(self, tofage, tof_raw, tofen=None, tofalive=None):
        """Ket luan ToF co dang chay khong. Tra (text, mau).

        HAI CAU HOI KHAC NHAU, truoc day bi gop lam mot:
          TOFAGE   = bao lau roi chua co mau HOP LE  -> 'co so de bay khong'
          TOFALIVE = bao lau roi CHIP khong do duoc  -> 'con cam bien khong'

        Dat drone xuong san thi ToF doc 0.000m (duoi tam mu ~4cm cua VL53L1X).
        Mau do bi loai -> TOFAGE tang vo han, TRONG KHI chip van do deu moi
        40ms. Ban cu chi nhin TOFAGE nen bao do 'mat tin hieu' -- sai han: do
        la 'khong co gi de do'. Dung mau do cho tinh huong binh thuong lam
        nguoi dung di tim loi phan cung khong ton tai, va te hon la lam ho bo
        qua canh bao do THAT khi no den.

        tofen=0 -> ToF bi TAT luc bien dich, TOFAGE=-1 la KET QUA MONG DOI.
        """
        if tofen == "0":
            return ("ToF: TAT theo cau hinh (SENSOR_TOF_ENABLED=0)", "gray")
        if tofage is None:
            return ("ToF: ? (firmware cu, khong co TOFAGE)", "gray")
        try:
            age = int(tofage)
        except (TypeError, ValueError):
            return ("ToF: ?", "gray")

        # alive=None -> firmware cu chua gui TOFALIVE. Khong the phan biet ->
        # giu nguyen hanh vi cu de khong am tham bo qua mot loi that.
        alive = None
        if tofalive is not None:
            try:
                alive = int(tofalive)
            except (TypeError, ValueError):
                alive = None

        # Chip THAT SU im: -1 = chua tung do duoc, hoac qua han.
        hw_dead = alive is not None and (alive < 0 or alive > self.TOF_STALE_MS)

        if age < 0:
            if alive is not None and not hw_dead:
                # Chip dang do nhung chua mau nao hop le: dung luc nam tren san.
                return (f"ToF: chip do binh thuong ({alive}ms) nhung chua co mau dung duoc (nam sat san?)", "#b35c00")
            return ("ToF LOI: CHUA TUNG co mau (hub khong doc) -> chay 'tof_test'", "#c0392b")

        if age > self.TOF_STALE_MS:
            if alive is not None and not hw_dead:
                # Day la ca da lam GUI bao do oan.
                return (f"ToF: khong co gi de do ({age}ms khong mau hop le, chip van do {alive}ms) -- ngoai tam / sat san", "#b35c00")
            return (f"ToF LOI: mau STALE {age}ms (>{self.TOF_STALE_MS}) -> mat tin hieu", "#c0392b")

        return (f"ToF OK ({age}ms, raw={tof_raw}m)", "#0a7d2c")
    # PHAI KHOP alt_source_t trong
    # components/flight_core/include/flight_core/alt_estimator.h.
    # Bang cu HOAN TOAN LECH: no map 0->"KHONG NGUON", 1->"ToF", 2->"BARO",
    # 3->"ToF+BARO" -- tuc la doc enum cua mot phien ban firmware khong con
    # ton tai. Hau qua nhin thay tren man hinh: nam tren san (ALTSRC=0 =
    # GROUND_LOCK, hoan toan binh thuong) bi hien do loet thanh
    # "Z<-KHONG NGUON", con luc dang bay bang ToF (ALTSRC=2 = TOF_FUSED) lai
    # hien "BARO" trong khi baro da bi TAT tu lau.
    ALT_SOURCE_NAMES = {
        "0": ("NAM DAT", "gray"),         # GROUND_LOCK: chua cat canh, Z khoa 0
        "1": ("IMU", "#b35c00"),          # IMU_PREDICT_ONLY: khong co correction
        "2": ("ToF", "#0a7d2c"),          # TOF_FUSED: dang fuse ToF -- trang thai tot
        "3": ("ToF (bridge)", "#b35c00"), # TOF_SHORT_BRIDGE: ho ngan, dang coast
        "4": ("MAT ToF", "#c0392b"),      # TOF_LOST: that su mat nguon do cao
    }

    def _alt_source_prefix(self, altsrc):
        """Tien to "Z<-NGUON" cho nhan do cao.

        Day la thong tin ma ADEGR KHONG noi duoc: degraded=0 chi nghia la "co
        correction", khong noi cua AI. Z<-BARO trong khi ban tuong dang bay bang
        ToF nghia la do cao so voi SAN dang troi (baro do ap suat, khong co tham
        chieu mat san) -- va moi co khac van xanh.
        """
        if altsrc is None:
            return "", None                      # firmware cu, khong co truong nay
        name, color = self.ALT_SOURCE_NAMES.get(altsrc, ("?%s" % altsrc, None))
        return "Z<-%s | " % name, color

    def _update_tof_label(self, tofst, tofcor, tofsurf, tofinn, floorz,
                           tofage=None, tof_raw=None, tof_valid=None, altsrc=None,
                           tofen=None, tofalive=None):
        """Hien ToF DANG nhin be mat nao va co dang sua world-Z khong.

        Day la thong tin duy nhat giai thich duoc vi sao UAV KHONG bam theo ToF
        khi bay qua ban: TOFST=OTHER + TOFCOR=0 la DUNG THIET KE, khong phai loi
        cam bien. Khong hien ra thi nguoi dung se tuong ToF hong.
        """
        if not hasattr(self, "tof_var"):
            return
        pfx, pcolor = self._alt_source_prefix(altsrc)
        # SUC KHOE truoc TIEN: sensor chet thi phan loai be mat vo nghia.
        health, hcolor = self._tof_health(tofage, tof_raw, tofen, tofalive)
        if hcolor == "#c0392b":
            self.tof_var.set(pfx + health)
            self.tof_label.configure(fg=hcolor)
            return
        if tofst is None or tofen == "0":
            # ToF bi TAT luc bien dich (SENSOR_TOF_ENABLED=0) hoac firmware cu.
            # Tien to Z<- van co nghia va van phai hien: no cho biet BARO dang
            # ganh, tuc la he thong VAN dang bay binh thuong khong can ToF.
            self.tof_var.set(pfx + health)
            self.tof_label.configure(fg=pcolor or "gray")
            return
        # ---- NHAN DIEN SAN DA BI BO HAN (firmware) ----
        # TRUOC DAY o day hien: "SAN (innov=...)" / "BE MAT KHAC (cao ~Xm,
        # san khoa Ym)". Toan bo ngon ngu do thuoc ve he thong do san +
        # innovation gate, ma ca hai deu da bi go khoi firmware:
        #   - innovation gate: bo (bay qua vat the khong con tu ha canh)
        #   - may do san:      bo (ToF gio do TUYET DOI, khong con goc toa do)
        # Giu lai chung tren man hinh la noi doi voi nguoi dung: "san khoa"
        # luon 0.00m va innov chi la hieu so voi mot moc khong con y nghia.
        #
        # GIO chi hien thu THAT SU con quyet dinh: co dang fuse ToF khong
        # (TOFCOR). Do la dieu duy nhat anh huong toi do cao.
        if tofcor == "1":
            self.tof_var.set(pfx + f"{health} | dang dung ToF cho do cao")
            self.tof_label.configure(fg="#0a7d2c")
        else:
            # Khong fuse: ngoai tam / be mat hap thu / nam sat san. KHONG phai
            # loi -- do cao dang coast bang IMU, va _tof_health() o tren da noi
            # ro chip con do hay khong.
            self.tof_var.set(pfx + f"{health} | chua co mau dung duoc (coast IMU)")
            self.tof_label.configure(fg="#b35c00")
    # ---- pha cất cánh — label riêng cạnh nút TAKEOFF ----
    def _update_takeoff_phase_label(self, tkop, airb, altm, tkoact=None,
                                     tkotgt=None, zsp=None, tkoi=None,
                                     tkognd=None, tkolift=None, tkoel=None,
                                     tkoab=None):
        """Nhãn pha cất cánh (closed-loop) cạnh nút TAKEOFF.

        Hiển thị theo pha THẬT của firmware (TKOP 0..4) thay vì latch như bản
        cũ — chuỗi có 2 pha kéo dài thật (PRIME/CLIMB) nên không còn khoảnh
        khắc nào nhấp nháy mất trong 1 khung hình.

        Trong CONTROL_ACTIVE, hiện luôn 4 bằng chứng liftoff còn thiếu — đó là
        thông tin duy nhất trả lời được "vì sao nó chưa chịu rời đất".
        """
        if not hasattr(self, "manual_tko_var"):
            return

        # TỪ CHỐI: ưu tiên trên cả ABORT, vì nó nghĩa là chuỗi CHƯA TỪNG bắt
        # đầu. Không có guard này thì nhãn "BI TU CHOI" vừa đặt ở
        # _update_takeoff_reject() bị pha IDLE ghi đè ngay tick sau (STATUS về
        # ~10Hz) và người dùng không kịp đọc chữ nào.
        if getattr(self, "_tko_reject_latch", None):
            return

        # ABORT: ưu tiên tuyệt đối, LATCH lại để không nhấp nháy mất (pha ABORT
        # chỉ tồn tại đúng 1 tick trước khi FSM đổi state).
        if tkop == "4" or (tkoab is not None and tkoab != "0"):
            why = TAKEOFF_ABORT_NAMES.get(tkoab or "0", "?")
            self._tko_abort_latch = why or "?"
        if getattr(self, "_tko_abort_latch", None) and airb != "1":
            self.manual_tko_var.set(f"TAKEOFF ABORT: {self._tko_abort_latch}")
            self.manual_tko_label.configure(fg="#c00000")
            return

        if tkop is None or tkop == "0":
            # Ngoài chuỗi cất cánh. Nếu đang bay thì báo đang HOLD.
            self._tko_abort_latch = None
            if airb == "1":
                alt_txt = f" {altm}m" if altm is not None else ""
                self.manual_tko_var.set(f"AIRBORNE — HOLDING{alt_txt}")
                self.manual_tko_label.configure(fg="#0a7d2c")
            else:
                self.manual_tko_var.set("TAKEOFF: -")
                self.manual_tko_label.configure(fg="gray")
            return

        self._tko_abort_latch = None
        name = TAKEOFF_PHASE_NAMES.get(tkop, f"?{tkop}")
        color = _TKO_PHASE_COLOR.get(tkop, "gray")

        if tkop == "1":
            txt = "TAKEOFF: PRIME (motor quay deu, Z/Vz PID CHUA chay)"
        elif tkop == "2":
            # Pha duy nhất đáng soi kỹ. Hai con số quan trọng nhất:
            #   ZSP  = target ĐANG TRƯỢT — phải bò đều từ TKOGND lên TKOTGT.
            #   TKOI = phần I của vòng Vz = hover THẬT đang được học.
            # Sau khi rời đất thì đổi sang xanh (TKOLIFT=1).
            lift_txt = "da roi dat" if tkolift == "1" else "CHUA roi dat"
            i_txt = f" I={tkoi}" if tkoi is not None else ""
            el_txt = f" t={tkoel}s" if tkoel is not None else ""
            txt = (f"TAKEOFF: CLIMB {zsp}->{tkotgt}m ({lift_txt}){i_txt}{el_txt}")
            if tkolift == "1":
                color = "#0a7d2c"
        else:
            zsp_txt = f" zsp={zsp}" if zsp is not None else ""
            tgt_txt = f" -> {tkotgt}m" if tkotgt is not None else ""
            txt = f"TAKEOFF: {name}{tgt_txt}{zsp_txt}"

        self.manual_tko_var.set(txt)
        self.manual_tko_label.configure(fg=color)

    # ---- alt (W/S) — step cố định firmware ('>'/'<') ----
    # ============ W/S — GIU de LEN/XUONG, NHA de giu tai cho ============
    # Ngu nghia: GIU W -> leo 0.3 m/s; NHA -> giu do cao dang o.
    # Day la MOMENTARY, khong phai buoc cong don.
    #
    # Gui gi: "@THR OFFSET <duty>" — gia tri TUYET DOI, 0 = nha.
    # VI SAO tuyet doi + gui lai dinh ky, thay vi "+100 luc nhan / -100 luc nha":
    # day la UDP. Mat dung goi "nha phim" thi cap +/- khong can va throttle dinh
    # +100 VINH VIEN trong luc motor dang quay. Voi gia tri tuyet doi, mot goi
    # mat chi tre 1 chu ky; ground-station chet han thi firmware tu ve 0 sau
    # BENCH_OFFSET_STALE_US (400ms). Cung nguyen tac da dung cho @SP.
    #
    # ⚠ Y NGHIA THEO STATE — GUI gui GIONG NHAU, firmware dich khac nhau:
    #   BENCH_RAMP     -> cong THANG vao duty (drone kep tren gia, do luc nang).
    #                     O day gia tri 100 THAT SU la 100 duty.
    #   HOLDING/FLYING -> chi lay DAU: len/xuong voi van toc ALT_HOLD_WS_VZ_MS
    #                     (0.3 m/s, xem tuning.h). Do lon 100 KHONG duoc dung.
    #                     Nha phim -> target neo vao do cao hien tai.
    #   con lai        -> firmware bo qua, ep 0
    #
    # ⚠ W/S CHI AN KHI DANG O TAB "Manual" (_manual_active()). O tab khac,
    # _on_manual_keypress() return ngay va phim khong lam gi ca -- khong phai
    # loi firmware.
    #
    # VI SAO GUI KHONG con re nhanh theo MODE: truoc day o che do HOLD, GUI tu
    # doi sang gui '>'/'<' (buoc do cao roi rac). Hai duong cho cung mot phim la
    # hai hanh vi khac han duoi tay nguoi lai — giu W thi khi thi bo len muot,
    # khi thi nhay tung nac, tuy theo mot bien noi bo (_pid_owns_throttle) ma
    # nguoi dung khong nhin thay.
    #
    # Te hon: bien do duoc gan BEN TRONG khoi STATUS_RE.match(), va khoi do
    # KHONG BAO GIO chay khi firmware con cat dong STATUS o 512 byte — nen no
    # ket o False, va nhanh '>'/'<' thuc ra chua tung chay. Sua buffer STATUS
    # xong thi hanh vi tu nhien doi, dung kieu bug khong ai lan ra duoc.
    #
    # Gio GUI luon gui @THR OFFSET; viec dich sang y nghia dung cua tung state
    # do FIRMWARE lam, la noi DUY NHAT biet chac minh dang o state nao.
    # Bien do offset ga khi GIU W/S trong FLYING. Da qua 100 -> 200 -> 100 ->
    # 150 theo cam giac bay cua nguoi dung; khong co rang buoc ky thuat nao
    # ghim con so nay, chi can no du nho de khong nhay bac.
    # CHI ap dung o FLYING — o HOLDING thi W/S di duong khac han (buoc do cao,
    # xem WS_ALT_STEP_M ben duoi).
    WS_THROTTLE_OFFSET_DUTY = 150
    WS_OFFSET_KEEPALIVE_MS = 100     # < BENCH_OFFSET_STALE_US(400ms) nhieu lan
    # Buoc do cao moi lan bam W/S o HOLD. Phai KHOP voi buoc ma firmware ap dung
    # cho '>'/'<' (command_parser.c: +-0.10m) — GUI chi gui phim, khong gui so.
    WS_ALT_STEP_M = 0.10

    # ========================================================================
    # W/S RE NHANH THEO STATE (yeu cau nguoi dung)
    # ========================================================================
    #   HOLD (mode 2)   -> MOT buoc do cao +-0.10m moi lan BAM. Khong lap lai
    #                      khi giu; muon len 30cm thi bam 3 lan.
    #   FLYING (mode 5) -> +-200 duty, CHI CO TAC DUNG KHI DANG GIU. Nha phim la
    #                      ve 0 ngay (momentary), giong ga tay.
    #   con lai         -> giu nguyen duong offset cu (bench/manual throttle).
    #
    # VI SAO GUI phai re nhanh chu khong de firmware lam: hai hanh vi nay khac
    # nhau ve BAN CHAT — mot cai la su kien roi rac (bam = +10cm), mot cai la
    # trang thai lien tuc (giu = +200). Firmware chi nhan duoc "offset = N", no
    # KHONG phan biet duoc "vua bam" voi "dang giu", nen khong the tu suy ra.
    #
    # ⚠ Doc mode tu STATUS gan nhat. Mat goi STATUS -> mode cu -> co the gui nham
    # loai lenh mot lan. Chap nhan duoc vi: gui nham '>' o FLYING chi doi target
    # (ma FLYING khong dung target de lai ga), va gui nham offset o HOLD thi
    # firmware cong vao throttle cua alt_hold roi tu neo lai target — ca hai deu
    # khong nguy hiem, chi la mot nhip khong nhu y.
    def _set_trim_enabled(self, allow):
        """Khoa/mo nut TRIM. allow=False khi drone DANG BAY.

        Chan o GUI la lop DAU, khong phai lop duy nhat — nguoi dung van co the
        go '@TRIM SET' qua console. Day chi la bo mot cach bam nham de dang.
        """
        if not hasattr(self, "_trim_widgets"):
            return
        state = "normal" if allow else "disabled"
        for w in self._trim_widgets:
            try:
                w.configure(state=state)
            except tk.TclError:
                pass
        if hasattr(self, "trim_frame"):
            self.trim_frame.configure(
                text="Trim live (@TRIM) - do bias" if allow
                else "Trim - KHOA khi dang bay")

    def _update_ws_labels(self, amode):
        """Nhan nut W/S + dong goi y phai khop MODE HIEN TAI.

        Goi moi khi STATUS doi mode. amode=None -> chua co STATUS.

        VI SAO nhan phai dong: cung mot nut lam hai viec khac han nhau tuy state
        (HOLD = mot buoc do cao roi rac; FLYING = ga giu-nut). Nhan tinh se dung
        cho MOT trong hai va sai cho cai con lai — va nguoi lai chi phat hien ra
        khi drone da o tren khong.
        """
        if not hasattr(self, "btn_w"):
            return
        if amode == "2":        # HOLD — PID do cao dang lai throttle
            self.btn_w.configure(text=f"W\nlen +{self.WS_ALT_STEP_M*100:.0f}cm")
            self.btn_s.configure(text=f"S\nxuong -{self.WS_ALT_STEP_M*100:.0f}cm")
            self.ws_hint_var.set(f"HOLD: BAM = +/-{self.WS_ALT_STEP_M*100:.0f}cm")
            if hasattr(self, "alt_frame"):
                self.alt_frame.configure(text="Do cao (HOLD)")
        elif amode == "5":      # FLYING — PID do cao TAT, ga tay
            self.btn_w.configure(text=f"W\nga +{self.WS_THROTTLE_OFFSET_DUTY}")
            self.btn_s.configure(text=f"S\nga -{self.WS_THROTTLE_OFFSET_DUTY}")
            self.ws_hint_var.set(f"FLYING: GIU = +/-{self.WS_THROTTLE_OFFSET_DUTY} ga")
            if hasattr(self, "alt_frame"):
                self.alt_frame.configure(text="Ga tay (FLYING)")
        else:
            # Cac state con lai (DISARMED/ARMED/TAKEOFF/LANDING): W/S van gui
            # offset ga nhu truoc, nhung firmware phan lon se bo qua. Noi ro
            # thay vi de nhan cua state truoc dinh lai.
            self.btn_w.configure(text="W\nga +")
            self.btn_s.configure(text="S\nga -")
            self.ws_hint_var.set("W/S chi an o tab Manual")
            if hasattr(self, "alt_frame"):
                self.alt_frame.configure(text="Do cao / Ga")

    def _ws_mode(self):
        """Mode alt gan nhat tu STATUS ('2'=HOLD, '5'=FLYING, ...) hoac None."""
        return getattr(self, "_last_alt_mode", None)

    def _ws_press(self, sign):
        """Bat dau giu W/S. Hanh vi phu thuoc state — xem khoi comment tren."""
        self._stop_ws_repeat()

        if self._ws_mode() == "2":
            # HOLD: mot buoc do cao roi rac. KHONG dat _ws_repeat_key -> khong co
            # keepalive, khong lap lai khi giu phim.
            self._alt_step(sign)
            self._log(f"[GUI] HOLD: W/S -> alt target {sign:+d} x {self.WS_ALT_STEP_M:.2f}m")
            return

        # FLYING (va cac state con lai): offset momentary + keepalive.
        self._ws_repeat_key = sign
        self._send_thr_offset(sign * self.WS_THROTTLE_OFFSET_DUTY)
        self._ws_repeat_after = self.root.after(
            self.WS_OFFSET_KEEPALIVE_MS, self._ws_tick)

    def _ws_tick(self):
        if self._ws_repeat_key is None:
            return
        self._send_thr_offset(self._ws_repeat_key * self.WS_THROTTLE_OFFSET_DUTY)
        self._ws_repeat_after = self.root.after(
            self.WS_OFFSET_KEEPALIVE_MS, self._ws_tick)

    def _stop_ws_repeat(self):
        """Nha phim (hoac watchdog ket phim) -> BAT BUOC tra offset ve 0."""
        if self._ws_repeat_after is not None:
            self.root.after_cancel(self._ws_repeat_after)
            self._ws_repeat_after = None
        had_key = self._ws_repeat_key is not None
        self._ws_repeat_key = None
        # Gui 0 KE CA khi khong chac dang giu: goi thua vo hai, goi thieu de lai
        # throttle +100. Firmware cung co watchdog rieng — hai lop.
        if had_key:
            self._send_thr_offset(0)

    def _send_thr_offset(self, duty):
        self.send_raw_cmd(f"@THR OFFSET {int(duty)}")

    def _alt_step(self, sign):
        self.send_raw_cmd(">" if sign > 0 else "<")

    # ---- an toàn ----
    def _zero_cmd(self, send=False):
        self._held.clear()
        # BUG 1: phải xóa CẢ _keys_down và _release_after, nếu không sau panic/
        # focus-out/đổi-tab thì keysym còn kẹt trong _keys_down -> _on_manual_keypress
        # RETURN sớm -> phím chết giữa lúc bay tới khi có KeyRelease thật.
        self._keys_down.clear()
        for aid in self._release_after.values():
            self.root.after_cancel(aid)
        self._release_after.clear()
        # QUY TẮC 2/3: Space (panic) / mất focus / đổi tab -> hủy MỌI phanh đang chạy.
        self._cancel_all_brakes()
        self.cmd_roll = self.cmd_pitch = self.cmd_yaw = 0.0
        for name in self._manual_btns:
            self._highlight(name, False)
        self._stop_ws_repeat()
        if send:
            self._send_sp()
        else:
            self._update_manual_cmd_label()
        self._update_brake_label()

    def _panic_level(self):
        # Space: về thăng bằng, giữ độ cao. KHÁC KILL (không cắt motor).
        self._zero_cmd(send=True)

    def _focus_sink(self):
        try:
            self._focus_sink_w.focus_set()
        except Exception:
            pass

    def _start_heartbeat(self):
        # Gọi từ connect() — KHÔNG còn phụ thuộc việc người dùng đã từng bấm
        # sang tab Manual Control. LỖI CŨ: vòng này chỉ khởi động trong
        # _on_tab_changed(), mà notebook mở ở tab Config nên sự kiện đó không hề
        # bắn cho Manual -> nếu chưa đổi tab lần nào thì KHÔNG có @SP nào được
        # gửi, firmware thấy CMDAGE tăng vô hạn (log thật: 14720 -> 19756ms).
        # Chạy suốt phiên kết nối là an toàn: ngoài tab Manual thì cmd_* đều 0
        # nên nó chỉ gửi "@SP SET 0 0 0" (thăng bằng), đúng thứ firmware sẽ tự
        # đặt khi stale — nhưng nhờ vậy CMDAGE luôn tươi và có ý nghĩa để soi.
        self._stop_heartbeat()
        self._heartbeat()

    # Keepalive @SP PHẢI nhanh hơn SP_STALE_TIMEOUT_US của firmware (tuning.h
    # mục 6 = 400ms). ĐÂY LÀ LỖI ĐÃ SỬA: trước đây keepalive là 1.0s, nên khi
    # GIỮ một hướng (giá trị không đổi -> changed=False) firmware hết 400ms là
    # tự zero setpoint (flight_core.c bước 4b) rồi đứng 0 suốt 600ms còn lại
    # tới keepalive kế -> drone chỉ nhận lệnh ~40% thời gian, giật cục, cảm
    # giác "không điều khiển được". 200ms = 1/2 timeout, đủ margin cho 1 gói
    # UDP rớt mà vẫn không bao giờ để setpoint stale.
    SP_KEEPALIVE_S = 0.2

    def _heartbeat(self):
        # BUG 2: chỉ gửi khi GIÁ TRỊ ĐỔI, cộng keepalive định kỳ phòng rớt gói.
        # Trước đây gửi 10Hz vô điều kiện -> ESP32 parse 10 lệnh/giây vô ích + ACK
        # ngược -> nghẽn 2 chiều. Timer vẫn quay 100ms để bắt kịp thay đổi tức thì.
        if self.console is not None:
            cur = (self.cmd_roll, self.cmd_pitch, self.cmd_yaw)
            now = time.time()
            changed = (cur != self._last_sent_sp)
            stale = (now - self._last_sp_time) > self.SP_KEEPALIVE_S
            if changed or stale:
                self.console.send_command(f"@SP SET {cur[0]:.2f} {cur[1]:.2f} {cur[2]:.2f}")
                self._last_sent_sp = cur
                self._last_sp_time = now
        self._hb_after = self.root.after(100, self._heartbeat)

    def _stop_heartbeat(self):
        if self._hb_after is not None:
            self.root.after_cancel(self._hb_after)
            self._hb_after = None

    # ---- Commander watchdog heartbeat ('p', xem command_parser.c case 'p') ----
    # KHÁC HẲN _heartbeat() ở trên (cái đó gửi @SP cho watchdog SP_STALE_TIMEOUT_US
    # RIÊNG của setpoint bay tay, CHỈ chạy trong tab Manual Control). Vòng lặp
    # NÀY nuôi commander_heartbeat() (commander.h) — watchdog AN TOÀN của
    # Commander, mặc định heartbeat_timeout_ms=1000ms (xem panel "Commander /
    # Failsafe"). TRƯỚC BẢN NÀY, KHÔNG CÓ GÌ qua UDP ground-station từng gọi
    # commander_heartbeat() — bay bằng console này sẽ luôn bị Commander tự
    # SOFT FAULT (-> LANDING) ngay tick đầu tiên sau khi vào HOLDING/FLYING
    # (heartbeat_age đứng yên từ lúc ESP32 boot). Chạy NGAY KHI KẾT NỐI (không
    # đợi vào tab Manual Control) và suốt phiên kết nối, KHÔNG phụ thuộc người
    # dùng nhớ bấm tay — 400ms/lần, đủ margin dưới 1000ms mặc định.
    CMDR_HEARTBEAT_INTERVAL_MS = 400

    def _start_cmdr_heartbeat(self):
        self._stop_cmdr_heartbeat()
        if hasattr(self, "cmdr_hb_var"):
            self.cmdr_hb_var.set("Commander heartbeat: ON (tu dong, 'p' moi 400ms)")
        self._cmdr_heartbeat_tick()

    def _cmdr_heartbeat_tick(self):
        if self.console is not None:
            self.console.send_command("p")
        self._cmdr_hb_after = self.root.after(
            self.CMDR_HEARTBEAT_INTERVAL_MS, self._cmdr_heartbeat_tick)

    def _stop_cmdr_heartbeat(self):
        if self._cmdr_hb_after is not None:
            self.root.after_cancel(self._cmdr_hb_after)
            self._cmdr_hb_after = None
        if hasattr(self, "cmdr_hb_var"):
            self.cmdr_hb_var.set("Commander heartbeat: OFF (chua ket noi)")

    def _key_watchdog(self):
        # BUG 4: lớp phòng thủ THỨ HAI cho focus check (vốn không tin cậy). Nếu
        # đang giữ phím BÀN PHÍM (_keys_down khác rỗng) mà >0.6s KHÔNG có key event
        # nào -> KeyRelease đã mất (alt-tab / focus bị cướp) -> coi như kẹt phím ->
        # về thăng bằng. Giữ phím thật thì auto-repeat liên tục refresh
        # _last_key_event nên watchdog KHÔNG kích oan. Dùng _keys_down (không phải
        # _held) để KHÔNG đụng trường hợp giữ CHUỘT trên nút hướng (không sinh key
        # event nhưng ButtonRelease luôn tới -> không thể kẹt).
        if (self._manual_active() and self._keys_down
                and (time.time() - self._last_key_event) > 0.6):
            self._zero_cmd(send=True)
            self._log("[GUI] watchdog: mat KeyRelease -> ve thang bang (ket phim)")
        self._kw_after = self.root.after(200, self._key_watchdog)

    def _manual_active(self):
        try:
            return self.notebook.select() == self._manual_tab_id
        except Exception:
            return False

    def _on_tab_changed(self, _e=None):
        # Vòng keepalive @SP KHÔNG còn bị bật/tắt theo tab (xem _start_heartbeat
        # gọi từ connect()). Đổi tab chỉ về thăng bằng + trả focus cho phím bay.
        self._zero_cmd(send=True)
        if self._manual_active():
            self._focus_sink()

    def _on_focus_out(self, _e=None):
        # Cửa sổ mất focus (alt-tab / click app khác) trong lúc giữ phím -> release
        # có thể không tới -> cmd kẹt. Về thăng bằng NGAY.
        if self._manual_active() and not self.root.focus_displayof():
            self._zero_cmd(send=True)

    def _on_unmap(self, e):
        if e.widget is self.root and self._manual_active():
            self._zero_cmd(send=True)

    # ---- key handling ----
    def _bind_manual_keys(self):
        keys = ("Up", "Down", "Left", "Right", "a", "A", "d", "D",
                "w", "W", "s", "S")
        for ks in keys:
            self.root.bind(f"<KeyPress-{ks}>", self._on_manual_keypress, add="+")
            self.root.bind(f"<KeyRelease-{ks}>", self._on_manual_keyrelease, add="+")
        for ks in ("space", "t", "T", "l", "L"):
            self.root.bind(f"<KeyPress-{ks}>", self._on_manual_keypress, add="+")
        # Phím tắt trim: q/e (roll), 1/3 (pitch). Bind cả hoa/thường cho chữ.
        for ks in ("q", "Q", "e", "E", "1", "3"):
            self.root.bind(f"<KeyPress-{ks}>", self._on_trim_keypress, add="+")
            self.root.bind(f"<KeyRelease-{ks}>", self._on_trim_keyrelease, add="+")
        self.root.bind("<FocusOut>", self._on_focus_out, add="+")
        self.root.bind("<Unmap>", self._on_unmap, add="+")

    def _focus_is_entry(self):
        w = self.root.focus_get()
        return isinstance(w, (tk.Entry, ttk.Entry, tk.Spinbox))

    def _on_manual_keypress(self, e):
        if not self._manual_active():
            return
        ks = e.keysym.lower()
        self._last_key_event = time.time()   # BUG 4: refresh cả khi auto-repeat
        if self._focus_is_entry():
            return   # đang gõ spinbox -> không điều khiển drone
        if ks in self._release_after:                 # auto-repeat X11: hủy release chờ
            self.root.after_cancel(self._release_after.pop(ks))
            return
        if ks in self._keys_down:                     # auto-repeat Windows
            return
        self._keys_down.add(ks)
        self._key_press_action(ks)

    def _on_manual_keyrelease(self, e):
        if not self._manual_active():
            return
        ks = e.keysym.lower()
        self._last_key_event = time.time()   # BUG 4
        if ks in self._release_after:
            return
        self._release_after[ks] = self.root.after_idle(
            lambda k=ks: self._confirm_release(k))

    def _confirm_release(self, ks):
        self._release_after.pop(ks, None)
        if ks in self._keys_down:
            self._keys_down.discard(ks)
            self._key_release_action(ks)

    # ---- phím tắt trim (q/e roll, 1/3 pitch) — độc lập hệ momentary bay ----
    def _on_trim_keypress(self, e):
        if not self._manual_active() or self._focus_is_entry():
            return   # chỉ ở tab Manual; đang gõ spinbox thì để số 1/3 vào ô
        ks = e.keysym.lower()
        spec = self._TRIM_KEYS.get(ks)
        if spec is None:
            return
        # PHIM TAT phai chiu CUNG mot khoa voi NUT (xem _set_trim_enabled).
        # Khoa nut ma de phim an la khoa nua voi: nguoi dung thay nut xam roi
        # tuong minh khong the chinh, trong khi q/e/1/3 van dich setpoint.
        if getattr(self, "_trim_locked_state", False):
            self._log("[GUI] TRIM bi khoa khi dang bay -- LAND truoc roi chinh")
            return
        if ks in self._trim_keys_down:      # auto-repeat -> bỏ, 1 nhấn = 1 bước
            return
        self._trim_keys_down.add(ks)
        self._trim_nudge(spec[0], spec[1])

    def _on_trim_keyrelease(self, e):
        self._trim_keys_down.discard(e.keysym.lower())

    def _key_press_action(self, ks):
        if ks in self.KEY_TO_DIR:
            self._dir_press(self.KEY_TO_DIR[ks], True)
        elif ks == "w":
            self._ws_press(+1)
        elif ks == "s":
            self._ws_press(-1)
        elif ks == "space":
            self._panic_level()
        elif ks == "t":
            self.send_flight_takeoff()
        elif ks == "l":
            self.send_flight_landing()

    def _key_release_action(self, ks):
        if ks in self.KEY_TO_DIR:
            self._dir_press(self.KEY_TO_DIR[ks], False)
        elif ks in ("w", "s"):
            self._stop_ws_repeat()

    # ---------------- connection ----------------

    def toggle_connect(self):
        if self.console is None:
            self.connect()
        else:
            self.disconnect()

    def connect(self):
        host = self.host_var.get().strip()
        if not host:
            self._log("[GUI] Nhap IP ESP32 truoc.")
            return

        try:
            port = int(self.port_var.get().strip())
        except ValueError:
            self._log("[GUI] Port khong hop le.")
            return

        self.console = UavUdpConsole(
            host=host, port=port, bind_port=self.bind_port, debug=self.debug,
            line_callback=self._on_line_from_rx_thread,
        )
        self.console.start()

        self.conn_status_var.set(f"Connected -> {host}:{port}")
        self.conn_status_label.configure(foreground="green")
        self.connect_btn.configure(text="Disconnect")
        self._log(f"[GUI] Da mo UDP toi {host}:{port}")

        # ---- BAT STATUS STREAMING NGAY, KHONG doi nguoi dung bam "Flight (f)" ----
        # command_parser.c: s_telemetry_enabled MAC DINH = false, va chi bat bang
        # phim 'f'. Chua bat thi firmware KHONG gui dong STATUS nao -> GUI MU
        # HOAN TOAN: khong co TKOREJ (vi sao TAKEOFF bi tu choi), khong co pha
        # cat canh, khong co suc khoe ToF, khong co ly do ARM bi tu choi.
        #
        # Trieu chung nhin thay: bam TAKEOFF -> "khong co gi xay ra", va khong
        # cach nao biet vi sao. Lenh VAN duoc gui va firmware VAN tra loi, chi la
        # khong ai nghe. Toan bo GUI nay duoc xay quanh dong STATUS, nen bat no
        # luc connect moi la mac dinh dung.
        self.root.after(200, lambda: self.send_raw_cmd("f"))
        self.root.after(300, self.get_all_pid)
        self.root.after(400, self.get_mahony)
        self.root.after(500, self.get_setpoint)
        self.root.after(550, self.get_flight_trim)
        self.root.after(600, self.get_flight_alt)
        self.root.after(700, self.get_flight_tko)
        self.root.after(800, self.get_flight_land)
        self.root.after(900, self.get_flight_cmdr)

        self._start_cmdr_heartbeat()
        self._start_heartbeat()      # keepalive @SP — xem _start_heartbeat()

    def disconnect(self):
        self._stop_cmdr_heartbeat()
        self._stop_heartbeat()

        if self.console is not None:
            self.console.stop()
            self.console = None

        self.conn_status_var.set("Disconnected")
        self.conn_status_label.configure(foreground="red")
        self.connect_btn.configure(text="Connect")
        self._log("[GUI] Da ngat ket noi.")

    def _on_close(self):
        self.disconnect()
        self.root.destroy()

    # ---------------- sending ----------------

    def send_raw_cmd(self, cmd: str):
        if self.console is None:
            self._log("[GUI] Chua ket noi.")
            return
        self.console.send_command(translate_command(cmd))

    def send_arm(self):
        if self.console is None:
            self._log("[GUI] Chua ket noi.")
            return

        # ARM ('r'): cho phep stabilizer cam lai dong co. Bam trong flight-balance
        # (sau khi da vao bang 'f') hoac o ban bay. Xac nhan vi dong co se quay khi
        # tang throttle. (Muon VAO flight-balance thi bam nut "Flight (f)" truoc.)
        confirmed = messagebox.askyesno(
            "ARM",
            "Dong co se san sang quay (self-level roll/pitch, yaw-rate).\n"
            "Da thao canh quat / co dinh khung chua?\n\n"
            "Xac nhan ARM (r)?",
        )
        if not confirmed:
            self._log("[GUI] Da huy ARM.")
            return

        self.send_raw_cmd("r")

    def send_kill(self):
        # KILL ('k') = disarm ngay -> stabilizer cat dong co, van o trong mode de
        # co the re-arm. Muon roi han flight-balance thi bam "Exit (q)".
        self.send_raw_cmd("k")

    def send_bench_start(self):
        # 'b': vào FSM_BENCH_RAMP để tune PID bằng throttle tay (+/-/]/[/0) —
        # CHỈ hợp lệ từ ARMED. Xác nhận riêng, KHÁC cảnh báo của "ARM": ở đây
        # PHẢI nhấn mạnh drone bị GIỮ CHẶT/KẸP TRÊN GIÁ ĐỠ vì throttle sẽ tăng
        # thật (không phải chỉ "sẵn sàng quay" như ARM).
        confirmed = messagebox.askyesno(
            "Bench-test throttle",
            "Drone se TANG GA THAT qua nut +/- de tune PID.\n"
            "Da GIU CHAT / KEP TREN GIA DO chua (day KHONG phai che do bay)?\n\n"
            "Xac nhan vao Bench-test (b)?",
        )
        if not confirmed:
            self._log("[GUI] Da huy Bench-test.")
            return
        self.send_raw_cmd("b")

    def send_disarm(self):
        # DISARM ('d') = disarm THƯỜNG, CHỈ hợp lệ từ state ARMED (đã ARM
        # nhưng chưa cất cánh) — xem command_parser.c case 'd'. KHÔNG confirm
        # dialog (cùng triết lý với KILL: đây là hành động "làm motor DỪNG",
        # không cần rào cản như ARM "làm motor CÓ THỂ quay"). Đang bay dở thì
        # lệnh này bị firmware từ chối — dùng LAND hoặc KILL thay vào đó.
        self.send_raw_cmd("d")

    def get_all_pid(self):
        if self.console is None:
            return
        self.console.send_command("@PID GET")

    def send_pid_set(self, loop, axis, values):
        if self.console is None:
            self._log("[GUI] Chua ket noi, khong the set PID.")
            return

        cmd = (
            f"@PID SET {loop} {axis} "
            f"{values['kp']:.4f} {values['ki']:.4f} {values['kd']:.4f} "
            f"{values['ilimit']:.4f} {values['outlimit']:.4f}"
        )
        self.console.send_command(cmd)

    def send_mahony_set(self):
        if self.console is None:
            self._log("[GUI] Chua ket noi, khong the set Mahony.")
            return
        kp = self.mah_kp.get_value()
        ki = self.mah_ki.get_value()
        self.console.send_command(f"@MAH SET {kp:.4f} {ki:.4f}")
        self.mah_status_var.set("sending...")

    def get_mahony(self):
        if self.console is None:
            return
        self.console.send_command("@MAH GET")

    def get_setpoint(self):
        if self.console is None:
            return
        self.console.send_command("@SP GET")

    # ---- Trim roll/pitch (@TRIM) ----
    def send_flight_trim_set(self):
        if self.console is None:
            self._log("[GUI] Chua ket noi, khong the set @TRIM.")
            return
        r = self.trim_roll.get_value()
        p = self.trim_pitch.get_value()
        self.console.send_command(f"@TRIM SET {r:.2f} {p:.2f}")
        self.trim_status_var.set("sending...")

    def get_flight_trim(self):
        if self.console is None:
            return
        self.console.send_command("@TRIM GET")

    # ---- Altitude bản BAY (flight_control) ----
    def send_flight_alt_set(self):
        if self.console is None:
            self._log("[GUI] Chua ket noi, khong the set altitude (ban bay).")
            return
        v = self.flight_alt.get_values()
        # @ALT SET <alt_kp> <vz_kp> <vz_ki> <vz_ilimit>  (hover đã sang @TKO)
        self.console.send_command(
            f"@ALT SET {v['kp']:.4f} {v['vzkp']:.2f} {v['vzki']:.2f} {v['vzilim']:.1f}")
        self.console.send_command(f"@ALT TGT {v['tgt']:.2f}")   # target theo MÉT
        self.flight_alt.status_var.set("sending...")

    def get_flight_alt(self):
        if self.console is None:
            return
        self.console.send_command("@ALT GET")

    # ---- Ground/takeoff params (@TKO) ----
    def send_flight_tko_set(self):
        if self.console is None:
            self._log("[GUI] Chua ket noi, khong the set @TKO.")
            return
        v = self.tko.get_values()
        # @TKO SET <hover> <prime_duty> <prime_ms> <max_climb_ms>
        self.console.send_command(
            f"@TKO SET {v['hover']:.0f} {v['prime']:.0f} {v['ms']:.0f} {v['climb']:.2f}")
        self.tko.status_var.set("sending...")

    def get_flight_tko(self):
        if self.console is None:
            return
        self.console.send_command("@TKO GET")

    # ---- Landing (@LAND) ----
    def send_flight_land_set(self):
        if self.console is None:
            self._log("[GUI] Chua ket noi, khong the set @LAND.")
            return
        v = self.land.get_values()
        # @LAND SET <descent_vz> <flare_alt> <flare_vz> <touchdown_alt>
        self.console.send_command(
            f"@LAND SET {v['dvz']:.2f} {v['flarealt']:.2f} "
            f"{v['fvz']:.2f} {v['tdalt']:.2f}")
        self.land.status_var.set("sending...")

    def get_flight_land(self):
        if self.console is None:
            return
        self.console.send_command("@LAND GET")

    def send_flight_landing(self):
        if self.console is None:
            self._log("[GUI] Chua ket noi, khong the LAND.")
            return
        # Kích landing tự động (firmware nhận khi armed + attitude valid).
        self.send_raw_cmd("l")
        self.land.status_var.set("LANDING...")

    # ---- Commander / Failsafe (@CMDR) ----
    def send_flight_cmdr_set(self):
        if self.console is None:
            self._log("[GUI] Chua ket noi, khong the set @CMDR.")
            return
        v = self.cmdr.get_values()
        # @CMDR SET <altmin> <altmax> <battfloor> <hbms> <tilt> <motorsatms>
        self.console.send_command(
            f"@CMDR SET {v['altmin']:.2f} {v['altmax']:.2f} {v['battfloor']:.2f} "
            f"{v['hbms']:.0f} {v['tilt']:.1f} {v['motorsatms']:.0f}")
        self.cmdr.status_var.set("sending...")

    def get_flight_cmdr(self):
        if self.console is None:
            return
        self.console.send_command("@CMDR GET")

    # ---- Test Motor (@TEST) ----
    def send_test_motor(self, motor_idx: int, duty_pct: int):
        if self.console is None:
            self._log("[GUI] Chua ket noi, khong the @TEST.")
            return
        self.console.send_command(f"@TEST {motor_idx} {duty_pct}")

    # ---- Flight Commands (@HOVER / @MOVE / @YAW) ----
    def send_flight_hover(self, sec: float):
        if self.console is None:
            self._log("[GUI] Chua ket noi, khong the @HOVER.")
            return
        self.console.send_command(f"@HOVER {sec:.1f}")
        self.flight_cmd.status_var.set(f"sending... HOVER {sec:.1f}s")

    def send_flight_move(self, direction: str, pct: int, sec: float):
        if self.console is None:
            self._log("[GUI] Chua ket noi, khong the @MOVE.")
            return
        self.console.send_command(f"@MOVE {direction} {pct} {sec:.1f}")
        self.flight_cmd.status_var.set(f"sending... MOVE {direction} {pct}% {sec:.1f}s")

    def send_flight_yaw(self, deg: float):
        if self.console is None:
            self._log("[GUI] Chua ket noi, khong the @YAW.")
            return
        self.console.send_command(f"@YAW {deg:.1f}")
        self.flight_cmd.status_var.set(f"sending... YAW {deg:.1f}deg")

    def send_flight_alt_mode(self, mode):
        if self.console is None:
            self._log("[GUI] Chua ket noi, khong the doi mode altitude.")
            return
        # 0=OFF, 1=LOG_ONLY, 2=HOLD. @ALT MODE đặt tuyệt đối (không toggle như phim z).
        self.console.send_command(f"@ALT MODE {int(mode)}")
        self.flight_alt.status_var.set(f"MODE -> {int(mode)}")

    def send_cal_cmd(self, cmd: str):
        if self.console is None:
            self._log("[GUI] Chua ket noi, khong the calib.")
            return
        self.console.send_command(cmd)

    def send_flight_takeoff(self):
        if self.console is None:
            self._log("[GUI] Chua ket noi, khong the TAKEOFF.")
            return
        # Độ cao mục tiêu ĐI KÈM LỆNH (@ALT TAKEOFF <m>) — chuỗi cất cánh giờ
        # leo tới đúng số này rồi settle rồi HOLD ở đó. MỘT lệnh là đủ, không
        # cần gửi @ALT SET sau khi liftoff.
        try:
            tgt = float(self.tko.target_var.get())
        except Exception:
            tgt = 0.30
            self._log("[GUI] tgt(m) khong doc duoc -> dung 0.30m")
        tgt = max(0.05, min(3.0, tgt))
        self.console.send_command(f"@ALT TAKEOFF {tgt:.2f}")
        self.tko.status_var.set(f"TAKEOFF tgt={tgt:.2f}m -> PRIME -> CLIMB -> HOLDING")

        # ---- Watchdog: co ai TRA LOI khong? ----
        # Phan biet hai kieu hong nhin giong het nhau tu ben ngoai:
        #   (a) firmware TU CHOI  -> co STATUS, TKOREJ != 0  (nhan da hien ly do)
        #   (b) GUI khong NGHE gi -> khong co STATUS nao ca (streaming tat / mat
        #       ket noi). Truoc day ca hai deu ra "bam nut khong co gi xay ra".
        self._tko_watch_seq = getattr(self, "_status_seen", 0)
        self.root.after(2000, self._check_takeoff_ack)

    def _check_takeoff_ack(self):
        """Kiem tra sau 2s: co dong STATUS nao toi khong ke tu luc bam TAKEOFF."""
        seen_now = getattr(self, "_status_seen", 0)
        if seen_now == getattr(self, "_tko_watch_seq", 0):
            msg = ("KHONG nhan duoc STATUS nao sau 2s -> GUI dang MU. "
                   "Bam nut 'Flight (f)' de bat streaming, hoac kiem tra ket noi.")
            self.tko.status_var.set(msg)
            self._log("[TAKEOFF] " + msg)
            if hasattr(self, "manual_tko_var"):
                self.manual_tko_var.set("TAKEOFF: khong co STATUS -> bam 'Flight (f)'")
                self.manual_tko_label.configure(fg="#c0392b")

    def send_target_set(self):
        if self.console is None:
            self._log("[GUI] Chua ket noi, khong the set target.")
            return
        roll = self.target_roll.get_value()
        pitch = self.target_pitch.get_value()
        yaw = self.target_yaw.get_value()
        # Pilot setpoint command handled by flight_control (writes s_setpoint):
        # roll/pitch = target angle (deg), yaw = target yaw-RATE (dps).
        self.console.send_command(f"@SP SET {roll:.3f} {pitch:.3f} {yaw:.3f}")
        self.target_status_var.set("sending...")

    def zero_target(self):
        self.target_roll.set_value(0.0)
        self.target_pitch.set_value(0.0)
        self.target_yaw.set_value(0.0)
        self.send_target_set()

    # ---------------- receiving ----------------

    def _on_line_from_rx_thread(self, line: str):
        # Called from UavUdpConsole's socket thread: never touch Tk widgets
        # here directly, just hand the line to the GUI thread via the queue.
        self.line_queue.put(line)

    def _poll_queue(self):
        # BUG 3: nếu backlog dồn quá lớn (telemetry chảy nhanh hơn ta xử lý),
        # drop bớt dòng cũ để không tụt hậu mãi -> main thread luôn kịp bắt phím.
        backlog = self.line_queue.qsize()
        if backlog > 500:
            dropped = 0
            while self.line_queue.qsize() > 250:
                try:
                    self.line_queue.get_nowait()
                    dropped += 1
                except queue.Empty:
                    break
            if dropped:
                self._log(f"[GUI] telemetry qua nhanh, dang drop {dropped} dong cu")

        # Giới hạn 20 dòng/lần poll, phần thừa để lần sau -> không ngốn main thread
        # (parse + push 4 do thi + redraw) khiến GUI "đơ" / key event xử lý chậm.
        for _ in range(20):
            try:
                self._handle_line(self.line_queue.get_nowait())
            except queue.Empty:
                break

        self.root.after(50, self._poll_queue)

    def _handle_line(self, line: str):
        self._log(f"[ESP32] {line}")

        # Feed the live plots from ANY line carrying R/P/Y + gyro (flight STATUS
        # line or maintenance live-Mahony line). Done first + independently of the
        # telemetry-label parsing so the chart works in every firmware mode.
        mp = PLOT_RE.search(line)
        if mp:
            try:
                sample = {
                    "roll": float(mp.group(1)),
                    "pitch": float(mp.group(2)),
                    "gx": float(mp.group(4)),
                    "gy": float(mp.group(5)),
                    "gz": float(mp.group(6)),
                }
                if mp.group(3) is not None:      # yaw present (not in flight-balance line)
                    sample["yaw"] = float(mp.group(3))
                self.plot_angle.push(sample)
                self.plot_gyro.push(sample)
            except (ValueError, TypeError):
                pass

        # Accel từng trục (body, g, đã LPF) từ cụm "| A=ax ay az |" của STATUS.
        mac = ACC_PLOT_RE.search(line)
        if mac:
            try:
                self.plot_acc.push({
                    "ax": float(mac.group(1)),
                    "ay": float(mac.group(2)),
                    "az": float(mac.group(3)),
                })
            except (ValueError, TypeError):
                pass

        # Độ cao: ALTm (estimator) / TOF (thô) / TGT (target) từ dòng STATUS.
        ma = ALT_PLOT_RE.search(line)
        if ma:
            try:
                alt_vals = {
                    "alt": float(ma.group(1)),
                    "tgt": float(ma.group(2)),
                    "tof": float(ma.group(3)),
                }
                mbf = BARO_FILT_RE.search(line)
                if mbf:
                    alt_vals["baro"] = float(mbf.group(1))
                mbr = BARO_RAW_RE.search(line)
                if mbr:
                    alt_vals["braw"] = float(mbr.group(1))
                self.plot_alt.push(alt_vals)
            except (ValueError, TypeError):
                pass

        m = PID_DUMP_RE.match(line)
        if m:
            loop, axis, kp, ki, kd, ilim, olim = m.groups()
            group = self.groups.get((loop, axis))
            if group is not None:
                group.set_values(float(kp), float(ki), float(kd), float(ilim), float(olim))
                if line.startswith("PID OK"):
                    group.mark_ok()
            return

        m = PID_ERR_RE.match(line)
        if m:
            for group in self.groups.values():
                if group.status_var.get() == "sending...":
                    group.mark_err(m.group(1))
            return

        m = MAH_RE.match(line)
        if m:
            kp, ki = m.groups()
            self.mah_kp.set_value(float(kp))
            self.mah_ki.set_value(float(ki))
            self.mah_status_var.set(f"OK  Kp={kp} Ki={ki}")
            return

        if line.startswith("MAH ERR"):
            self.mah_status_var.set(line)
            return

        # Pilot setpoint reply ("SP R=.. P=.. Y=.." / "SP OK R=.. P=.. Y=..").
        m = SP_RE.match(line)
        if m:
            roll, pitch, yaw = m.groups()
            self.target_roll.set_value(float(roll))
            self.target_pitch.set_value(float(pitch))
            self.target_yaw.set_value(float(yaw))
            self.target_status_var.set(f"OK  R={roll} P={pitch} Y={yaw}")
            return

        if line.startswith("SP ERR"):
            self.target_status_var.set(line)
            return

        # Trim reply @TRIM -> panel Trim.
        m = TRIM_RE.match(line)
        if m:
            r, p = m.groups()
            self.trim_roll.set_value(float(r))
            self.trim_pitch.set_value(float(p))
            self.trim_status_var.set(f"OK roll={r} pitch={p}")
            self._update_manual_trim_label()
            return

        if line.startswith("TRIM ERR"):
            self.trim_status_var.set(line)
            return

        # Altitude controller reply @ALT (chữ thường) -> panel "ALT controller".
        m = FLIGHT_ALT_RE.match(line)
        if m:
            kp, vzkp, vzki, vzilim, tgt, mode = m.groups()
            self.flight_alt.set_gains(
                float(kp), float(vzkp), float(vzki), float(vzilim),
                float(tgt) if tgt is not None else None)
            self.flight_alt.mark_ok()
            self.flight_alt.status_var.set(
                f"OK kp={kp}" + (f" mode={mode}" if mode is not None else ""))
            return

        # Ground/takeoff reply @TKO -> panel "Takeoff / Ground".
        m = TKO_RE.match(line)
        if m:
            hover, prime, ms, climb = m.groups()
            self.tko.set_values(float(hover), float(prime), float(ms),
                                float(climb) if climb is not None else None)
            self.tko.mark_ok()
            climb_txt = f" climb={climb}" if climb is not None else ""
            self.tko.status_var.set(f"OK hover={hover} prime={prime} ms={ms}{climb_txt}")
            return

        if line.startswith("TKO ERR"):
            self.tko.status_var.set(line)
            return

        # Landing reply @LAND -> panel "Landing".
        m = LAND_RE.match(line)
        if m:
            dvz, fa, fvz, td = m.groups()
            self.land.set_values(float(dvz), float(fa), float(fvz), float(td))
            self.land.mark_ok()
            self.land.status_var.set(f"OK dvz={dvz} fvz={fvz} tdalt={td}")
            return

        if line.startswith("LAND ERR") or line.startswith("LAND REJECTED") \
                or line.startswith("LAND started"):
            self.land.status_var.set(line)
            return

        # Commander reply @CMDR -> panel "Commander / Failsafe".
        m = CMDR_RE.match(line)
        if m:
            altmin, altmax, battfloor, hbms, tilt, motorsatms = m.groups()
            self.cmdr.set_values(float(altmin), float(altmax), float(battfloor),
                                 float(hbms), float(tilt), float(motorsatms))
            self.cmdr.mark_ok()
            self.cmdr.status_var.set(f"OK altmin={altmin} altmax={altmax} tilt={tilt}")
            return

        if line.startswith("CMDR ERR"):
            self.cmdr.status_var.set(line)
            return

        # Test Motor reply @TEST -> panel "Test Motor". Không parse số ra (chỉ
        # 1 xung, không có state để đồng bộ lại) — hiện nguyên dòng.
        if line.startswith("TEST OK") or line.startswith("TEST ERR"):
            self.test_motor.status_var.set(line)
            return

        # Flight Commands reply @HOVER/@MOVE/@YAW -> panel "Flight Commands".
        if (line.startswith("HOVER ") or line.startswith("MOVE ")
                or line.startswith("YAW ")):
            self.flight_cmd.status_var.set(line)
            return

        # BẢN BAY: reply @ALT TGT / @ALT MODE / TAKEOFF -> chỉ hiện trạng thái.
        if (line.startswith("ALT TGT=") or line.startswith("ALT MODE=")
                or line.startswith("ALT TAKEOFF")):
            self.flight_alt.status_var.set(line)
            return

        if line.startswith("ALT ERR"):
            self.flight_alt.status_var.set(line)
            return

        # @CAL ... (xem command_parser.c::handle_cal(), MỚI) — mọi reply CAL,
        # kể cả "CAL ERR", đều bắt đầu bằng "CAL " nên gộp chung 1 nhánh.
        if line.startswith("CAL "):
            self.calib.set_status(line)
            return

        # Reply phím 'f' (xem command_parser.c case 'f', MỚI) — chỉ cập nhật
        # label, KHÔNG cần parse gì thêm ngoài phân biệt ON/OFF.
        if line.startswith("FLIGHT MODE ON"):
            self.telem_stream_var.set("STATUS streaming: ON")
            return
        if line.startswith("FLIGHT MODE OFF"):
            self.telem_stream_var.set("STATUS streaming: OFF")
            return

        m = STATUS_RE.match(line)
        if m:
            g = m.groups()
            armed, thr, roll, pitch, yaw = g[0], g[1], g[2], g[3], g[4]
            valid = g[8]
            acc_norm, accel_used, yaw_rel = g[9], g[10], g[11]
            altm, vz, atgt, amode, av, tof, _tok, terr = g[12:20]
            tko, ki, land = g[20], g[21], g[22]   # TKO=spool, KI=I-term, LAND=pha landing
            # Đuôi mở rộng UAV-S3 (xem STATUS_RE) — có thể None nếu firmware cũ.
            magok, baraok, imuok, balt, loop_dt, cmdage, hbage, fault = g[23:31]
            batv, bcomp = g[31], g[32]
            bfilt, binnov, bacc, brej, bcal, bhlt, bstd, vacc = g[33:41]
            # Đuôi accel-primary estimator (xem STATUS_RE) — None nếu firmware
            # cũ hơn (trước refactor accel-primary/bỏ ToF khỏi alt_estimator).
            bdt, vaccr, abias, vzao, zine, vztgt = g[41:47]
            # Đuôi refactor an toàn/realtime — None nếu firmware cũ hơn.
            kill, tkop, lsc, ldt, lmx, dlm = g[47:53]
            iage, mage, bage, hdeg = g[53:57]
            msat, mrl, mpl, myl, mhr, bthr = g[57:63]
            # Đuôi estimator GROUND/CANDIDATE/AIRBORNE — None nếu firmware cũ hơn.
            airb, cand, azcorr, bcrej, breacq, bseq, bfi = g[63:70]
            # Đuôi battery debug — None nếu firmware cũ hơn.
            # ALTSRC -- nhom rieng, chen NGAY SAU bfi (xem STATUS_RE).
            # ⚠ Them nhom vao GIUA regex lam DICH moi chi so phia sau. Cac lat cat
            # duoi day da duoc danh so lai +1; sua regex ma quen cho nay se lam GUI
            # doc nham cot MA KHONG BAO LOI (dung kieu hong im lang da gap 1 lan).
            altsrc = g[70]
            batraw, batmv, batratio, batvraw, batvalid, batcali, batage = g[71:78]
            # Đuôi TAKEOFF (PID + slew) — None nếu firmware cũ hơn refactor này.
            (tkoact, tkotgt, zsp, vztgt, tkobase, tkocorr, tkoi, tkognd,
             tkolift, tkotilt, tkoel, tkoab, altsat) = g[78:91]
            # Lý do từ chối ARM — None nếu firmware cũ hơn.
            # Dem dong STATUS -> _check_takeoff_ack() dung de biet GUI co dang
            # NGHE duoc firmware khong (xem send_flight_takeoff).
            self._status_seen = getattr(self, "_status_seen", 0) + 1
            armrej, armrseq = g[91], g[92]
            self._update_arm_reject(armrej, armrseq)
            # Lý do từ chối TAKEOFF — None nếu firmware cũ hơn.
            tkorej, tkorseq = g[93], g[94]
            self._update_takeoff_reject(tkorej, tkorseq)
            # Duoi ToF surface-gated — None neu firmware cu hon.
            (tofv, tofinn, tofsurf, tofst, tofcor, tofgr,
             tofa, tofr, floorz, landz, landv, tofage) = g[95:107]
            # TOFEN o CUOI regex -> g[107]. None = firmware cu (truoc khi co truong nay).
            tofen = g[107]
            # Luong sua thuc te + hoc bias -- g[108:114], None neu firmware cu.
            (tofcorrz, tofcorrvz, barocorrz, barocorrvz,
             biasres, biasadp) = g[108:114]
            # Latch ga hover theo pin -- g[114:117], o CUOI regex. None = firmware
            # cu hon (truoc khi co hover_model.h) HOAC HOVER_LATCH_ENABLED=0.
            hovlk, hovlv, hovld = g[114:117]
            # TOFALIVE o CUOI regex -> g[117]. None = firmware cu chua gui.
            tofalive = g[117]
            self._update_tof_label(tofst, tofcor, tofsurf, tofinn, floorz,
                                    tofage, tof, _tok, altsrc, tofen, tofalive)

            # Nuôi 2 đồ thị Vz/Az world (xem _build_plots) — CÙNG khối `if m:`
            # vì các field này CHỈ có ở đuôi mở rộng UAV-S3, không tách được
            # bằng regex độc lập như PLOT_RE/ACC_PLOT_RE (không có mốc bắt đầu
            # rõ ràng để search() an toàn).
            try:
                if zine is not None:
                    self.plot_alt.push({"zine": float(zine)})
                if vz is not None and vzao is not None:
                    vz_vals = {"vz": float(vz), "vzao": float(vzao)}
                    if vztgt is not None:
                        vz_vals["vztgt"] = float(vztgt)
                    self.plot_vz.push(vz_vals)
                if vaccr is not None and vacc is not None:
                    self.plot_az.push({"azraw": float(vaccr), "azfilt": float(vacc)})
            except (ValueError, TypeError):
                pass

            text = (
                f"ARM={'YES' if armed == '1' else 'no'} THR={thr} | "
                f"R={roll} P={pitch} Y={yaw} | valid={valid}"
            )

            if acc_norm is not None:
                # ACCU=0 -> firmware da bo qua mau accel nay (rung dong co /
                # gia toc khac trong luc), pitch/roll dang chi dua vao gyro.
                # Neu pitch tang theo throttle ma ACCU van la 1 va ACC gan 1.0,
                # nhieu kha nang la mat can bang co khi/canh quat that, khong
                # phai loi UDP hay loi EKF.
                acc_flag = "OK" if accel_used == "1" else "GATED(vibration?)"
                text += f" | ACC={acc_norm}g accel={acc_flag}"

            if yaw_rel is not None:
                # Yaw tuong doi so voi huong luc ARM (heading-hold latch).
                text += f" | YAWREL={yaw_rel}"

            # Cụm altitude bản BAY (nếu có) -> label telemetry + panel "ALT bay".
            if altm is not None:
                # Khop telemetry_format_alt_mode() — 5=FLYING, xem mnames duoi.
                names = {"0": "OFF", "1": "LOG", "2": "HOLD",
                         "3": "TAKEOFF", "4": "LANDING", "5": "FLYING"}
                text += (f" | ALT={altm}m vz={vz} tgt={atgt} "
                         f"mode={names.get(amode, amode)} tof={tof} av={av}")
                try:
                    self.flight_alt.set_live(
                        float(altm), float(vz), float(atgt), int(amode),
                        float(tof), av, terr)
                except (ValueError, TypeError):
                    pass

            # Duty 4 motor (nếu có trong dòng) -> hiện gọn trên nhãn.
            mmo = MOTOR_RE.search(line)
            if mmo:
                text += f" | M={'/'.join(mmo.groups())}"

            # Ki (I-term angle/rate) đang DÙNG hay KHÓA + pha takeoff (spool).
            if ki is not None:
                text += f" | Ki={'DUNG' if ki == '1' else 'KHOA'}"
            if tko is not None and tko != "0":
                text += " | TKO=SPOOL"
            land_names = {"1": "DESCEND", "2": "FLARE", "3": "TOUCHDOWN", "4": "BLIND"}
            if land is not None and land != "0":
                text += f" | LAND={land_names.get(land, land)}"

            # Đuôi mở rộng UAV-S3 (driver ok flags + fault + watchdog age) —
            # chỉ hiện khi firmware có gửi (None nếu STATUS cũ/rút gọn).
            if fault is not None:
                sensor_bad = [
                    name for name, ok in (("MAG", magok), ("BARO", baraok), ("IMU", imuok))
                    if ok == "0"
                ]
                if sensor_bad:
                    text += f" | SENSOR_LOI={'/'.join(sensor_bad)}"
                if baraok == "1":
                    text += f" | BALT={balt}m"
                fault_names = {"0": "NONE", "1": "SOFT", "2": "HARD"}
                if fault != "0":
                    text += f" | FAULT={fault_names.get(fault, fault)}"
                text += f" | dt={loop_dt}ms cmdAge={cmdage}ms hbAge={hbage}ms"

            # BATV/BCOMP tách riêng khỏi khối "fault is not None" ở trên vì
            # firmware có thể gửi cặp này dù không gửi khối MAGOK..FAULT (regex
            # độc lập nhau) — không giả định luôn đi kèm.
            if batv is not None:
                comp_txt = f" comp={bcomp}x" if bcomp is not None and bcomp != "1.000" else ""
                text += f" | BAT={batv}V{comp_txt}"
            self._update_battery_label(batv, bcomp, batvalid, batvraw,
                                        hovlk, hovlv, hovld)

            # Baro estimator debug (median-of-3+LPF+innovation gate, xem
            # alt_estimator.h) — tách riêng khỏi khối BATV/BCOMP vì firmware có
            # thể gửi/không gửi độc lập nhau (regex riêng, không giả định đi kèm).
            if bfilt is not None:
                health_txt = "OK" if bhlt == "1" else ("UNCAL" if bcal == "0" else "UNHEALTHY")
                text += (f" | BARO filt={bfilt}m innov={binnov}m "
                         f"accept={bacc} reject={brej} std={bstd}Pa[{health_txt}]")
            if vacc is not None:
                text += f" | vAcc={vacc}m/s2"

            # Pha GROUND/AIRBORNE + reacquire (xem alt_estimator.h) — AIRB=0
            # là BÌNH THƯỜNG lúc chưa cất cánh (Z/Vz khoá cứng=0), KHÔNG phải
            # lỗi. BREACQ=1 nghĩa là estimator đang tự kéo lại sau một chuỗi
            # bất đồng dài với baro (xem "reacquire"). CẢNH BÁO riêng nếu reject
            # liên tục cao (>=20, ~vài giây ở baro ~25-50Hz) MÀ baro vẫn khoẻ —
            # dấu hiệu estimator kẹt, đúng yêu cầu "không cho silently mất anchor".
            if airb is not None:
                # 3 pha rõ ràng (xem alt_estimator.h "BA PHA"): GROUND nghĩa là
                # Z/Vz đang khoá 0 CÓ CHỦ ĐÍCH — KHÔNG phải estimator hỏng.
                if airb == "1":
                    phase_txt = "AIRBORNE"
                elif cand == "1":
                    phase_txt = "CANDIDATE"
                else:
                    phase_txt = "GROUND(Z/Vz khoa 0)"
                text += f" | ALT_PHASE={phase_txt}"
                if breacq == "1":
                    text += " REACQUIRE!"
                if bfi == "0" and airb == "1":
                    text += " [fusion CHUA init]"
                try:
                    if int(bcrej) >= 20 and bhlt == "1":
                        text += f" BARO_STUCK(reject_lientuc={bcrej})!"
                except (TypeError, ValueError):
                    pass

            # Battery debug — CHỈ hiện khi CÓ VẤN ĐỀ (invalid/thiếu calibration),
            # vì lúc bình thường label BAT riêng ở thanh trên đã đủ. Hiện số thô
            # đúng lúc nó sai là điểm mấu chốt: BATVRAW cho biết firmware quy đổi
            # ra bao nhiêu dù đã bị loại.
            if batvalid == "0":
                text += (f" | BAT_INVALID(raw={batraw} mv={batmv} ratio={batratio} "
                         f"v={batvraw} cali={batcali}) -> KHONG dung cho failsafe")

            # Accel-primary estimator debug (state gamma + shadow accel-only
            # tracker, xem alt_estimator.h) — so vzao/zine với vz/altm ở trên
            # để thấy baro đang kéo bao nhiêu (chênh lớn = baro correction
            # đang làm việc nhiều/estimator mới hội tụ; chênh nhỏ dần theo
            # thời gian là bình thường sau reacquire).
            if abias is not None:
                text += f" | bias={abias}m/s2 vzAO={vzao} zINE={zine}"

            # ---- An toàn/realtime (refactor kill-latch + sensor_hub) ----
            # KILL đứng ĐẦU và luôn hiện khi bật: đây là thứ người dùng cần
            # thấy trước mọi con số khác — motor đã bị cắt cứng, mọi lệnh bay
            # sẽ không có tác dụng cho tới khi ARM lại.
            if kill == "1":
                text = "*** KILL LATCH (can ARM lai) *** " + text
            if tkop is not None and tkop != "0":
                text += f" | TKO={TAKEOFF_PHASE_NAMES.get(tkop, tkop)}"
                if tkotgt is not None:
                    # zsp = target ĐANG TRƯỢT (phải bò đều, không nhảy bậc);
                    # I = phần vòng Vz tự học hover thật.
                    text += f" tgt={tkotgt}m zsp={zsp}"
                if tkoi is not None:
                    text += f" I={tkoi}"
                if tkolift is not None:
                    text += " LIFTED" if tkolift == "1" else " (chua roi dat)"
                if altsat == "1":
                    text += " ALT_PID_SAT"
            if tkoab is not None and tkoab != "0":
                text += f" | TKO_ABORT={TAKEOFF_ABORT_NAMES.get(tkoab, tkoab)}"
            self._update_takeoff_phase_label(tkop, airb, altm, tkoact, tkotgt,
                                              zsp, tkoi, tkognd, tkolift, tkoel, tkoab)
            if hdeg == "1":
                text += " | HEADING_DEGRADED(yaw chi con gyro)"
            # Tuổi mẫu: chỉ hiện khi CÓ VẤN ĐỀ (âm = chưa có mẫu, hoặc quá cũ).
            # Hiện mọi lúc thì dòng status dài vô ích và người đọc bỏ qua.
            try:
                stale_bits = []
                for nm, val, limit in (("IMU", iage, 20), ("MAG", mage, 300), ("BARO", bage, 500)):
                    if val is None:
                        continue
                    iv = int(val)
                    if iv < 0:
                        stale_bits.append(f"{nm}=chua-co-mau")
                    elif iv > limit:
                        stale_bits.append(f"{nm}={iv}ms")
                if stale_bits:
                    text += " | STALE: " + " ".join(stale_bits)
            except (ValueError, TypeError):
                pass
            if msat == "1":
                axes = "".join(a for a, f in (("R", mrl), ("P", mpl), ("Y", myl)) if f == "1")
                text += f" | MIXER_SAT[{axes}] headroom={mhr}"
            if dlm is not None and dlm != "0":
                text += f" | deadlineMiss={dlm} loopMax={lmx}us"

            self.telemetry_var.set(text)

            # ---- Nuôi tab Manual Control ----
            if altm is not None:
                self._last_alt_target = atgt
            # Mode alt gan nhat — _ws_press() doc de re nhanh W/S (HOLD = buoc do
            # cao, FLYING = offset ga giu-nut). Luu O DAY, NGOAI khoi
            # `if hasattr(manual_status_var)`: neu de ben trong thi khi tab Manual
            # chua duoc dung, mode ket o None va W/S se im lang chay nhanh sai.
            self._last_alt_mode = amode
            # Nhan nut W/S bam theo mode. Chi ve lai khi mode THAT SU doi —
            # configure() moi frame la lang phi va lam nut nhap nhay.
            if amode != getattr(self, "_ws_labels_mode", "__init__"):
                self._ws_labels_mode = amode
                self._update_ws_labels(amode)
            # TRIM chi cho chinh khi KHONG bay. "Dang bay" = ARM va mode alt la
            # mot trong TAKEOFF/HOLD/LANDING/FLYING (2..5). ARM ma van nam dat
            # (mode 0/1) thi van cho chinh — do la luc do bias hop le nhat.
            _flying = (armed == "1") and (amode in ("2", "3", "4", "5"))
            if _flying != getattr(self, "_trim_locked_state", None):
                self._trim_locked_state = _flying
                self._set_trim_enabled(not _flying)
            if hasattr(self, "manual_status_var"):
                # Khop telemetry_format_alt_mode() (src/telemetry_format.c).
                # 5=FLYING tach rieng khoi 2=HOLD: truoc day firmware tra CUNG
                # gia tri 2 cho ca HOLDING lan FLYING nen GUI luon hien "HOLD"
                # ke ca khi dang bay tien/lui/trai/phai.
                mnames = {"0": "OFF", "1": "LOG", "2": "HOLD",
                          "3": "TAKEOFF", "4": "LANDING", "5": "FLYING"}
                armed_yes = (armed == "1")
                # THR= chính là throttle base: khi mode HOLD/TAKEOFF/LANDING (2/3/4)
                # nó là output PID giữ độ cao (hover+dthr); còn lại là throttle tay.
                # CHI de hien thi nhan "(PID)" canh THR=. KHONG con dieu khien
                # hanh vi phim W/S nua (xem khoi comment tren _ws_press).
                # ⚠ FLYING (5) DA BI LOAI khoi danh sach nay.
                # Comment cu noi "FLYING van chay alt_hold y het HOLDING" — dieu
                # do TUNG dung, nhung KHONG con dung: firmware gio TAT HAN PID do
                # cao trong FLYING (hold_driving=false, throttle den tu
                # s_flying_throttle_latch). Giu 5 o day se dan nhan "(PID)" trong
                # khi khong co PID nao chay — telemetry noi doi dung luc nguoi lai
                # can biet ai dang cam ga.
                thr_pid = amode in ("2", "3", "4")
                thr_txt = f"{thr}{' (PID)' if thr_pid else ''}" if thr is not None else "?"
                mode_txt = mnames.get(amode, amode) if amode is not None else "?"
                # alt: hien ca do cao HIEN TAI va TARGET. Chi mot so thi khong
                # biet drone dang bam target hay dang troi - dung cai lech giua
                # hai so nay de thay ngay (log truoc: alt=1.32 nhung tgt=0.97).
                if altm is not None:
                    alt_txt = f"{altm}m"
                    if atgt is not None:
                        alt_txt += f" /tgt {atgt}m"
                    if vz is not None:
                        alt_txt += f"  vz={vz}"
                else:
                    alt_txt = "?"
                # W/S DANG LAM GI — hien ngay canh mode, vi cung mot phim co hai
                # y nghia khac han nhau tuy state (xem khoi comment tren
                # _ws_press). Khong hien thi thi nguoi lai phai tu nho, va bam
                # nham o do cao that la mot cach hong chuyen bay.
                if amode == "2":
                    ws_hint = f"W/S=alt +-{self.WS_ALT_STEP_M:.2f}m (bam)"
                elif amode == "5":
                    ws_hint = f"W/S=thr +-{self.WS_THROTTLE_OFFSET_DUTY} (GIU)"
                else:
                    ws_hint = "W/S=thr (giu)"
                self.manual_status_var.set(
                    f"ARM={'YES' if armed_yes else 'no'}   "
                    f"mode={mode_txt}   "
                    f"alt={alt_txt}   "
                    f"THR={thr_txt}   "
                    f"[{ws_hint}]")
                self._update_manual_cmd_label()
                # Takeoff chỉ enable khi đã ARM; Land luôn enable.
                if hasattr(self, "manual_takeoff_btn"):
                    self.manual_takeoff_btn.configure(
                        state=("normal" if armed_yes else "disabled"))
                # ---- Throttle 4 dong co (M= trong dong STATUS) ----
                # LUON cap nhat, ke ca khi dong STATUS khong co cum "M=":
                # de nguyen gia tri cu thi nhan hien so CU CUA LAN TRUOC ma
                # khong co dau hieu gi - nguoi doc tuong motor van dang o duty
                # do. Khong co du lieu thi phai NOI la khong co.
                if hasattr(self, "manual_motor_var"):
                    if mmo:
                        m1, m2, m3, m4 = mmo.groups()
                        # Them THR de so sanh: 4 motor xoay quanh THR, lech nhau
                        # chinh la correction cua PID. Nhin duoc do lech ngay
                        # tren mot dong la biet mixer co dang lam viec khong.
                        base = f"   (THR={thr})" if thr is not None else ""
                        self.manual_motor_var.set(
                            f"M1={m1:>5}  M2={m2:>5}  M3={m3:>5}  M4={m4:>5}{base}")
                    else:
                        self.manual_motor_var.set(
                            "M1=--  M2=--  M3=--  M4=--   (dong STATUS khong co cum M=)")
            return

    def _log(self, text: str):
        self.log_text.configure(state="normal")
        self.log_text.insert(tk.END, text + "\n")
        self.log_text.see(tk.END)
        self.log_text.configure(state="disabled")


def run_gui(args):
    if not _TK_AVAILABLE:
        print(
            "Tkinter khong co san trong Python nay. "
            "Cai dat lai Python tu python.org (mac dinh co Tkinter) "
            "hoac dung --cli de chay console dang text.",
            file=sys.stderr,
        )
        sys.exit(1)

    root = tk.Tk()

    # Size to fit the actual screen, never taller than it — otherwise the plots
    # (bottom of the window) end up below the screen edge / behind the taskbar
    # and look "missing".
    sw = root.winfo_screenwidth()
    sh = root.winfo_screenheight()
    # Nới từ 1240 -> 1760: từ bản này Log chiếm CỨNG 50% bề rộng (xem
    # PidTunerApp `body`), nên nửa còn lại phải đủ chỗ cho tab Config.
    # 1240 cũ sẽ chỉ cho notebook 620px và cắt mất cột PID bên phải.
    win_w = min(1760, sw - 40)
    win_h = min(980, sh - 80)
    root.geometry(f"{win_w}x{win_h}+20+10")
    root.minsize(900, 600)

    PidTunerApp(root, args.host, args.port, args.bind_port, args.debug)
    root.mainloop()


def main():
    parser = argparse.ArgumentParser(
        description="UAV-Mini UDP console + PID tuner",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    parser.add_argument(
        "host", nargs="?", default=None,
        help="IP cua ESP32, vi du 192.168.1.19 (GUI: co the de trong roi nhap trong app)",
    )
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="UDP port cua ESP32, mac dinh 4210")
    parser.add_argument("--bind-port", type=int, default=None, help="Local UDP port cua PC, thuong khong can")
    parser.add_argument("--debug", action="store_true", help="In packet TX de debug")
    parser.add_argument("--cli", action="store_true", help="Dung console dang go lenh text, khong mo GUI")

    args = parser.parse_args()

    if args.cli:
        if not args.host:
            parser.error("host la bat buoc khi dung --cli")
        run_cli(args)
    else:
        run_gui(args)


if __name__ == "__main__":
    main()
