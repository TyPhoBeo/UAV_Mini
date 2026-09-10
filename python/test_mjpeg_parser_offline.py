"""MjpegParser: tach JPEG tu byte stream MJPEG.

VI SAO CAN TEST NAY. Parser nay chay tren MOT byte stream tuy y do mang cat
khuc -- chunk KHONG bao gio trung ranh gioi frame. Moi loi o day deu la loi
IM LANG: khong crash, chi la video dung hinh hoac tre dan, va nguoi dung se
di do loi cho Wi-Fi / ESP32 chu khong nghi toi parser.

Bon cai bay THAT su cua bai nay:
  1. Marker SOI/EOI bi CAT DOI giua hai chunk (0xFF o cuoi chunk n, 0xD8 o dau
     chunk n+1). Vut het chunk khi khong thay SOI se an mat frame do.
  2. Header multipart nam GIUA cac anh -> phai bo, khong duoc tinh vao JPEG.
  3. Ket noi lai giua chung frame -> buffer con nua anh cu, phai resync chu
     khong duoc ghep nua anh cu voi nua anh moi.
  4. Server gui thu khong phai JPEG -> buffer phinh vo han -> het RAM tren PC.

Test chay KHONG can cv2/requests/numpy/mang: do la ly do MjpegParser duoc tach
ra khoi CameraStreamWorker.
"""
import io
import os
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tools"))

from camera_stream import (MjpegParser, MIN_JPEG_BYTES, SOI, EOI,  # noqa: E402
                           STATE_DISCONNECTED, STATE_CONNECTING,
                           STATE_STREAMING, STATE_RECONNECTING)

FAILED = []


def check(name, cond, detail=""):
    print("  %s  %s" % ("PASS" if cond else "FAIL", name))
    if not cond:
        if detail:
            print("        %s" % detail)
        FAILED.append(name)


def fake_jpeg(marker, size=400):
    """Mot 'anh' JPEG gia: SOI + than nhan dang duoc + EOI.

    Than KHONG duoc chua 0xFFD9, neu khong ta tu tao ra chinh ca kho ma test
    (5) dang co tinh dung -- va se khong phan biet duoc lo hong that voi du
    lieu test cau tha.
    """
    body = bytes([marker]) * (size - 4)
    assert EOI not in body and SOI not in body
    return SOI + body + EOI


def part_header(n):
    return b"\r\n--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % n


def mjpeg_stream(images):
    """Dung dong giong het camera_stream.c phat ra."""
    out = b""
    for img in images:
        out += part_header(len(img)) + img
    return out


def feed_in_chunks(parser, data, size):
    got = []
    for i in range(0, len(data), size):
        got.extend(parser.feed(data[i:i + size]))
    return got


print("== (1) Stream sach, chunk lon hon frame ==")
imgs = [fake_jpeg(0x41), fake_jpeg(0x42), fake_jpeg(0x43)]
p = MjpegParser()
got = feed_in_chunks(p, mjpeg_stream(imgs), 4096)
check("lay du 3 frame", len(got) == 3, "duoc %d" % len(got))
check("noi dung khop tung frame", got == imgs)
check("header multipart bi loai het",
      all(b"Content-Length" not in g for g in got))

print()
print("== (2) BAY CHINH: chunk cat NAT, ke ca giua marker ==")
data = mjpeg_stream(imgs)
for size in (1, 2, 3, 7, 13, 64, 511):
    p = MjpegParser()
    got = feed_in_chunks(p, data, size)
    ok = (got == imgs)
    print("     chunk=%-4d -> %d frame  %s" % (size, len(got), "OK" if ok else "SAI"))
    check("chunk %d byte van ra dung 3 frame" % size, ok,
          "chunk=1 la truong hop khac nghiet nhat: MOI marker deu bi cat doi")

print()
print("== (3) Buffer khong duoc phinh sau khi da nha frame ==")
p = MjpegParser()
feed_in_chunks(p, mjpeg_stream(imgs * 20), 100)
check("buffer con lai nho hon mot frame", p.buffered < len(imgs[0]),
      "con %d byte -- dang giu lai du lieu da tieu thu" % p.buffered)

print()
print("== (4) Frame chua hoan chinh thi CHO, khong nha nua anh ==")
p = MjpegParser()
half = mjpeg_stream(imgs)[:len(imgs[0]) // 2]
check("chua du -> chua nha frame nao", p.feed(half) == [])
rest = mjpeg_stream(imgs)[len(imgs[0]) // 2:]
check("nap not -> ra du 3 frame", len(p.feed(rest)) == 3)

print()
print("== (5) Rac truoc SOI bi bo, khong lam lech frame ==")
p = MjpegParser()
got = p.feed(b"HTTP/1.1 200 OK\r\nrac rac rac\r\n" + mjpeg_stream(imgs))
check("van ra dung 3 frame du co rac dau dong", got == imgs, "duoc %d" % len(got))

# Rac phai bi CAT NGAY, khong duoc nam cho toi khi frame hoan chinh. Khac biet
# chi lo ra khi dang DOI mot frame do dang: neu khong cat, buffer = rac + nua
# anh va phinh dan theo moi lan reconnect.
p = MjpegParser()
rac = b"HTTP/1.1 200 OK\r\n" + b"x" * 5000 + b"\r\n"
# PHAI cat SAU khi da co SOI, neu khong nhanh can kiem khong chay: part_header
# dai dung 60 byte, nen [:60] la header thuan -- parser roi vao nhanh "chua co
# SOI" va test se PASS ke ca khi code hong. Da vap dung loi nay.
partial = mjpeg_stream(imgs)[:len(part_header(len(imgs[0]))) + 20]
assert SOI in partial and EOI not in partial, "du lieu test khong dung nhanh can kiem"
p.feed(rac + partial)
check("rac truoc SOI bi cat ngay khi dang doi frame",
      p.buffered <= 32,
      "buffer %d byte -- dang om lai ~%d byte rac" % (p.buffered, p.buffered - 20))

print()
print("== (6) Ket noi lai giua chung frame -> reset phai cat duoc nua anh cu ==")
p = MjpegParser()
p.feed(mjpeg_stream(imgs)[:len(imgs[0]) - 10])   # dut ngay truoc EOI
check("dang giu du lieu do dang", p.buffered > 0)
p.reset()
check("reset xoa sach", p.buffered == 0)
got = p.feed(mjpeg_stream(imgs))
check("sau reset khong ghep nham nua anh cu", got == imgs)

print()
print("== (7) Server gui thu khong phai JPEG -> resync, khong het RAM ==")
p = MjpegParser(max_buffer=8192)
# Co SOI de parser bat dau giu lai, nhung KHONG BAO GIO co EOI.
p.feed(SOI + b"Z" * 20000)
check("buffer bi vut khi vuot tran", p.buffered <= 8192,
      "con %d byte" % p.buffered)
check("co dem lai so lan resync", p.resyncs >= 1)
got = p.feed(mjpeg_stream(imgs))
check("sau resync van bat lai duoc stream", len(got) == 3, "duoc %d" % len(got))

print()
print("== (8) Manh qua nho bi bo (rac, khong phai anh) ==")
p = MjpegParser()
tiny = SOI + b"\x00" * 4 + EOI          # ngan hon MIN_JPEG_BYTES
got = p.feed(tiny + mjpeg_stream([imgs[0]]))
check("bo manh < %d byte" % MIN_JPEG_BYTES, got == [imgs[0]],
      "nha ra %d manh" % len(got))

print()
print("== (9) Bon trang thai theo yeu cau muc 18 deu ton tai ==")
for st in (STATE_DISCONNECTED, STATE_CONNECTING, STATE_STREAMING,
           STATE_RECONNECTING):
    check("co trang thai %s" % st, isinstance(st, str) and st != "")

print()
print("== (10) Worker KHONG import cv2/requests o muc module ==")
# Neu import o dau file thi chi mo GUI khong co camera cung crash. Chung phai
# nam TRONG _run(), tuc trong thread nen.
src = io.open(os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "tools", "camera_stream.py"),
    encoding="utf-8").read()
head = src[:src.index("class MjpegParser")]
check("khong 'import cv2' o dau file", "import cv2" not in head)
check("khong 'import requests' o dau file", "import requests" not in head)
check("cv2 duoc import ben trong _run()",
      "import cv2" in src[src.index("def _run"):])

print()
if FAILED:
    print("KET QUA: %d FAIL -- %s" % (len(FAILED), FAILED[0]))
    sys.exit(1)
print("KET QUA: TAT CA PASS")
