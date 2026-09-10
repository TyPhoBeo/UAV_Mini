"""Client MJPEG cho UAV-S3 -- worker doc /stream tren thread RIENG.

MjpegParser khong import gi ngoai stdlib nen test duoc offline, khong can
mang/cv2 (python/test_mjpeg_parser_offline.py). GUI chi goi latest_frame(),
khong bao gio doc socket.
"""
from __future__ import annotations

import threading
import time

SOI = b"\xff\xd8"      # Start Of Image
EOI = b"\xff\xd9"      # End Of Image

# ~100 frame QVGA. Vuot nguong = dang tich rac -> vut het, bat lai tu SOI.
MAX_BUFFER_BYTES = 2 * 1024 * 1024

# JPEG nho hon nguong nay gan nhu chac chan la rac (header + vai byte).
MIN_JPEG_BYTES = 128


class MjpegParser:
    """Bytes -> tung anh JPEG, theo marker SOI/EOI chu khong theo boundary.

    Khong parse multipart header vi moi server dat mot kieu, va mot header le
    lam hong ca stream; marker nam trong chinh du lieu JPEG nen khong sai duoc.

    Danh doi: 0xFFD9 co the xuat hien trong du lieu nen -> anh bi cat som ->
    imdecode tra None -> bo mot khung. Khong mat dong bo.
    """

    def __init__(self, max_buffer=MAX_BUFFER_BYTES):
        self._buf = bytearray()
        self._max_buffer = max_buffer
        self.resyncs = 0        # so lan phai vut buffer
        self.frames_found = 0

    def feed(self, chunk):
        """Nap chunk, tra ve list cac JPEG hoan chinh (co the rong)."""
        if chunk:
            self._buf.extend(chunk)

        out = []
        while True:
            start = self._buf.find(SOI)
            if start < 0:
                # Giu 1 byte cuoi: 0xFF co the vat qua ranh gioi chunk.
                if len(self._buf) > 1:
                    del self._buf[:-1]
                break

            end = self._buf.find(EOI, start + 2)
            if end < 0:
                # Co dau, chua co duoi. Bo rac truoc SOI roi doi them.
                if start > 0:
                    del self._buf[:start]
                break

            jpeg = bytes(self._buf[start:end + 2])
            del self._buf[:end + 2]
            if len(jpeg) >= MIN_JPEG_BYTES:
                out.append(jpeg)
                self.frames_found += 1

        # Chan buffer phinh vo han.
        if len(self._buf) > self._max_buffer:
            self._buf.clear()
            self.resyncs += 1
        return out

    @property
    def buffered(self):
        return len(self._buf)

    def reset(self):
        self._buf.clear()


# ---------------------------------------------------------------- trang thai
STATE_DISCONNECTED = "DISCONNECTED"
STATE_CONNECTING = "CONNECTING"
STATE_STREAMING = "STREAMING"
STATE_RECONNECTING = "RECONNECTING"


class CameraStreamWorker:
    """Doc MJPEG tren thread rieng, giu DUY NHAT frame moi nhat.

    Khong dung Queue: GUI ve cham hon mang thi hang doi chi lam do tre tang dan.
    """

    def __init__(self, url, retry_s=1.0, timeout_s=5.0, read_timeout_s=15.0,
                 on_state=None, processor=None):
        # read_timeout TACH KHOI connect timeout, va dai hon HAN.
        # Do tren bo: khi buffer JPEG cua esp32-camera qua nho, cam_hal bo cac
        # khung thieu EOI va CHO khung sau -- khoang lang co the vai giay. Voi
        # read timeout 5s, requests nem loi -> worker dong ket noi -> ESP log
        # "httpd_sock_err: error in send : 104" (ECONNRESET) va stream chet.
        # Ngat roi ket noi lai TON kem hon la doi.
        self.read_timeout_s = read_timeout_s
        # processor: callable(frame_bgr), chay ngay sau giai ma TRONG thread
        # nay. Khong tach thread thu ba: se them mot ban sao khung hinh va mot
        # cho de anh lech pha voi ket qua.
        self.processor = processor
        self.proc_error = ""
        self.url = url
        self.retry_s = retry_s
        self.timeout_s = timeout_s
        self._on_state = on_state

        self._lock = threading.Lock()
        self._frame = None          # numpy.ndarray BGR, hoac None
        self._result = None         # ket qua cua processor cho DUNG khung do
        self._frame_seq = 0
        self._frame_t = 0.0

        self._stop = threading.Event()
        self._thread = None

        self.state = STATE_DISCONNECTED
        self.last_error = ""
        self.fps = 0.0
        self.frames = 0
        self.decode_fails = 0

    # ---- dieu khien ----
    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="cam_stream",
                                        daemon=True)
        self._thread.start()

    def stop(self, join_s=2.0):
        self._stop.set()
        t = self._thread
        if t and t.is_alive():
            t.join(timeout=join_s)
        self._set_state(STATE_DISCONNECTED)

    def latest_frame(self):
        """(frame, seq, age_s). frame=None neu chua co. Goi tu GUI thread."""
        with self._lock:
            if self._frame is None:
                return None, 0, 0.0
            return self._frame, self._frame_seq, time.time() - self._frame_t

    def latest_result(self):
        """(frame, ket_qua_processor, seq) — luon la CUNG MOT khung hinh.

        Diem noi cho phan dieu khien: doc o day thi ket qua va anh khong lech.
        """
        with self._lock:
            return self._frame, self._result, self._frame_seq

    # ---- noi bo ----
    def _set_state(self, st):
        if st == self.state:
            return
        self.state = st
        if self._on_state:
            try:
                self._on_state(st)
            except Exception:
                # Callback cua GUI khong duoc phep giet thread mang.
                pass

    def _publish(self, frame, result=None):
        with self._lock:
            self._frame = frame
            self._result = result
            self._frame_seq += 1
            self._frame_t = time.time()

    def _run(self):
        import numpy as np
        import cv2
        import requests

        parser = MjpegParser()
        first = True

        while not self._stop.is_set():
            self._set_state(STATE_CONNECTING if first else STATE_RECONNECTING)
            first = False
            parser.reset()

            try:
                with requests.get(self.url, stream=True,
                                  timeout=(self.timeout_s, self.read_timeout_s)) as r:
                    r.raise_for_status()
                    self._set_state(STATE_STREAMING)
                    self.last_error = ""
                    win_t, win_n = time.time(), 0

                    for chunk in r.iter_content(chunk_size=4096):
                        if self._stop.is_set():
                            break
                        if not chunk:
                            continue

                        for jpeg in parser.feed(chunk):
                            arr = np.frombuffer(jpeg, dtype=np.uint8)
                            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                            if frame is None:
                                # Anh cat som -> bo, khong ngat ket noi.
                                self.decode_fails += 1
                                continue
                            result = None
                            if self.processor is not None:
                                try:
                                    result = self.processor(frame)
                                    self.proc_error = ""
                                except Exception as pexc:     # noqa: BLE001
                                    # Loi xu ly anh khong duoc giet stream.
                                    self.proc_error = "%s: %s" % (
                                        type(pexc).__name__, pexc)
                            self._publish(frame, result)
                            self.frames += 1
                            win_n += 1

                        now = time.time()
                        if now - win_t >= 1.0:
                            self.fps = win_n / (now - win_t)
                            win_t, win_n = now, 0

            except Exception as exc:                      # noqa: BLE001
                # Bat rieng tung loai se bo sot loai chua nghi toi -> thread
                # chet im lang, GUI dung hinh ma khong bao gi.
                self.last_error = "%s: %s" % (type(exc).__name__, exc)

            if self._stop.is_set():
                break
            self._set_state(STATE_RECONNECTING)
            self.fps = 0.0
            # wait() chu khong sleep(): stop() cat duoc ngay.
            self._stop.wait(self.retry_s)

        self._set_state(STATE_DISCONNECTED)
