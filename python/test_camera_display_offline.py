"""Duong dua anh OpenCV vao Tkinter: PPM, KHONG Pillow.

BOI CANH -- toi da noi SAI va day la test de dieu do khong lap lai.
Toi bao "duong PPM cham hon, giat thi pip install pillow". Do tren chinh may
nay (Tk 8.6.13, frame 320x240, 50 lan):

    PPM bytes tho     0.30 ms/frame     tran ~3300 fps
    PNG base64       13.36 ms/frame     tran ~75 fps
    PPM base64       KHONG CHAY -- TclError "couldn't recognize image data"

Stream 12 fps co ngan sach 83 ms/frame. 0.30 ms la 0.4% cua no. Pillow khong
mua duoc gi, ma them mot dependency + mot nhanh code thu hai khong ai chay.

Hai thu test nay khoa lai:
  1. PPM bytes THO nap duoc vao tk.PhotoImage (base64 thi KHONG -- neu ai do
     "sua cho gon" bang base64 se hong ngay).
  2. KENH MAU dung. cv2 giu BGR, PPM la RGB. Doi kenh do/lam la loi IM LANG:
     anh van hien, van muot, chi la mau sai -- va no se pha dung phan color
     detection sap lam.

Test tu bo qua neu may khong co cv2/Tk (vd CI khong man hinh).
"""
import sys

FAILED = []
SKIP = False


def check(name, cond, detail=""):
    print("  %s  %s" % ("PASS" if cond else "FAIL", name))
    if not cond:
        if detail:
            print("        %s" % detail)
        FAILED.append(name)


try:
    import cv2
    import numpy as np
    import tkinter as tk
    _root = tk.Tk()
    _root.withdraw()
except Exception as exc:                              # noqa: BLE001
    print("BO QUA: khong co cv2/numpy/Tk kha dung (%s: %s)"
          % (type(exc).__name__, exc))
    SKIP = True

if not SKIP:
    def to_photo(frame):
        ok, buf = cv2.imencode(".ppm", frame)
        assert ok
        return tk.PhotoImage(data=buf.tobytes())

    print("Tk %s" % _root.tk.call("info", "patchlevel"))
    print()
    print("== (1) PPM bytes THO nap duoc ==")
    frame = (np.random.rand(240, 320, 3) * 255).astype(np.uint8)
    img = to_photo(frame)
    check("tao duoc PhotoImage 320x240",
          (img.width(), img.height()) == (320, 240),
          "ra %dx%d" % (img.width(), img.height()))

    print()
    print("== (2) base64 KHONG chay -- dung 'toi uu' sang no ==")
    import base64
    ok, buf = cv2.imencode(".ppm", frame)
    try:
        tk.PhotoImage(data=base64.b64encode(buf.tobytes()))
        b64_ok = True
    except Exception:                                 # noqa: BLE001
        b64_ok = False
    check("PPM base64 bi Tcl tu choi (nhu da do)", not b64_ok,
          "neu ban Tk nay CHAP NHAN base64 thi ghi chu trong _to_photo() can "
          "cap nhat -- khong phai loi, nhung dung de ghi chu sai")

    print()
    print("== (3) KENH MAU: BGR vao -> RGB ra, khong doi do/lam ==")
    cases = [
        ("DO",  (0, 0, 255), (255, 0, 0)),
        ("LAM", (255, 0, 0), (0, 0, 255)),
        ("LA",  (0, 255, 0), (0, 255, 0)),
    ]
    for ten, bgr, mong in cases:
        f = np.zeros((8, 8, 3), np.uint8)
        f[:, :] = bgr
        got = to_photo(f).get(4, 4)
        # Tk 8.6 tra tuple; ban cu tra chuoi "r g b".
        if isinstance(got, str):
            got = tuple(int(x) for x in got.split())
        print("     %-4s BGR%-12s -> Tk %s" % (ten, bgr, got))
        check("%s khong bi doi kenh" % ten, tuple(got) == mong,
              "mong %s, duoc %s -- do/lam dang bi hoan doi" % (mong, got))

    print()
    print("== (4) zoom nguyen giu dung kich thuoc ==")
    z = to_photo(frame).zoom(2)
    check("zoom(2) -> 640x480", (z.width(), z.height()) == (640, 480),
          "ra %dx%d" % (z.width(), z.height()))

    print()
    print("== (5) Console KHONG con phu thuoc Pillow ==")
    import io
    import os
    src = io.open(os.path.join(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))), "tools", "uav_udp_console.py"),
        encoding="utf-8").read()
    check("khong con 'from PIL import'", "from PIL import" not in src)
    check("khong con 'import PIL'", "import PIL" not in src)
    check("van dung imencode('.ppm')", '.ppm' in src)

    _root.destroy()

print()
if FAILED:
    print("KET QUA: %d FAIL -- %s" % (len(FAILED), FAILED[0]))
    sys.exit(1)
print("KET QUA: TAT CA PASS" if not SKIP else "KET QUA: BO QUA (thieu cv2/Tk)")
