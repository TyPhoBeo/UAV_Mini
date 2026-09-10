"""Nhan dien mau: kiem tren anh TU SINH, khong can camera/drone/mang.

VI SAO CAN TEST NAY. Ket qua cua no se duoc dung de DIEU KHIEN drone. Mot loi
o day khong lam gi crash ca -- no chi tra ve toa do sai, va drone se bay theo
toa do sai do. Khong co cach nao phat hien bang mat khi dang bay.

Ba thu de sai nhat, va deu im lang:
  1. MAU DO vat qua moc 0 cua vong Hue (OpenCV: H = 0..179). Mot khoang don
     bat duoc dung mot nua sac do. Anh van co vet, van co toa do -- chi la mat
     nua so lan.
  2. DAU cua toa do chuan hoa. nx duong = ben PHAI. Doi dau = drone bay nguoc
     huong, va no van chay muot ma.
  3. Nguong dien tich theo DIEM ANH thay vi ty le -> doi do phan giai la moi
     nguong sai het, khong ai bao gi.

Test tu bo qua neu may khong co cv2/numpy.
"""
import os
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tools"))

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
    from color_detect import (ColorDetector, ColorProfile, DEFAULT_PROFILES,
                              HUE_MAX)
except ImportError as exc:
    print("BO QUA: thieu %s" % exc.name)
    SKIP = True


def canvas(w=320, h=240, bgr=(20, 20, 20)):
    img = np.zeros((h, w, 3), np.uint8)
    img[:, :] = bgr
    return img


def put_rect(img, cx, cy, w, h, bgr):
    cv2.rectangle(img, (cx - w // 2, cy - h // 2), (cx + w // 2, cy + h // 2),
                  bgr, -1)
    return img


if not SKIP:
    det = ColorDetector()

    print("== (1) Bat duoc tung mau co ban ==")
    # BGR thuan cho bon mau mac dinh.
    # Dung mau BAO HOA VUA PHAI, khong bao hoa tuyet doi: bang HSV_RANGES_UAV
    # dat san S 70..100 tuy mau, ma vat that hiem khi bao hoa het co.
    for ten, bgr in (("red", (38, 52, 185)), ("green", (60, 165, 70)),
                     ("blue", (190, 110, 45)), ("yellow", (45, 200, 215)),
                     ("orange", (30, 110, 215)), ("purple", (150, 55, 130))):
        img = put_rect(canvas(), 160, 120, 60, 60, bgr)
        got = det.detect(img)
        labels = [d.label for d in got]
        check("thay '%s'" % ten, ten in labels, "chi thay %s" % labels)

    print()
    print("== (2) BAY DO: mau do vat qua moc 0 cua Hue ==")
    # Do "tuoi" (H~0) va do "tham nga tim" (H~175) deu la DO. Mot khoang don
    # chi bat duoc mot trong hai.
    for ten, hsv in (("H~2   (dau duoi vong)", (2, 200, 200)),
                     ("H~170 (dau tren vong)", (170, 200, 200))):
        px = np.zeros((1, 1, 3), np.uint8)
        px[0, 0] = hsv
        bgr = tuple(int(v) for v in cv2.cvtColor(px, cv2.COLOR_HSV2BGR)[0, 0])
        img = put_rect(canvas(), 160, 120, 60, 60, bgr)
        got = [d for d in det.detect(img) if d.label == "red"]
        check("red o %s van bat duoc" % ten, len(got) == 1,
              "profile 'red' phai co HAI khoang HSV, khong phai mot")

    prof_do = next(p for p in DEFAULT_PROFILES if p.name == "red")
    check("profile 'red' khai dung 2 khoang", len(prof_do.hsv_ranges) == 2,
          "co %d khoang" % len(prof_do.hsv_ranges))

    print()
    print("== (3) BAY DAU: toa do chuan hoa ==")
    # nx < 0 = TRAI, nx > 0 = PHAI, ny < 0 = TREN, ny > 0 = DUOI.
    cases = [("trai-tren",  (60, 50),   -1, -1),
             ("phai-tren",  (260, 50),  +1, -1),
             ("trai-duoi",  (60, 190),  -1, +1),
             ("phai-duoi",  (260, 190), +1, +1)]
    for ten, (cx, cy), sx, sy in cases:
        img = put_rect(canvas(), cx, cy, 40, 40, (38, 52, 185))
        got = det.detect(img)
        if not got:
            check("%s: tim thay vet" % ten, False)
            continue
        d = got[0]
        print("     %-11s tam anh (%3d,%3d) -> n=(%+.2f,%+.2f)"
              % (ten, cx, cy, d.nx, d.ny))
        check("%s: dau nx dung" % ten, (d.nx > 0) == (sx > 0))
        check("%s: dau ny dung" % ten, (d.ny > 0) == (sy > 0))

    img = put_rect(canvas(), 160, 120, 40, 40, (38, 52, 185))
    d = det.detect(img)[0]
    check("vat o CHINH GIUA -> n ~ (0,0)", abs(d.nx) < 0.05 and abs(d.ny) < 0.05,
          "ra (%+.3f,%+.3f)" % (d.nx, d.ny))

    print()
    print("== (4) Nguong dien tich theo TY LE, khong theo diem anh ==")
    # Cung mot vat chiem cung ty le khung -> phai bat duoc o CA HAI do phan giai.
    for w, h in ((320, 240), (640, 480)):
        img = canvas(w, h)
        put_rect(img, w // 2, h // 2, w // 8, h // 8, (38, 52, 185))
        got = [x for x in det.detect(img) if x.label == "red"]
        frac = got[0].area_frac if got else 0.0
        print("     %dx%d -> %d vet, area_frac=%.4f" % (w, h, len(got), frac))
        check("%dx%d van bat duoc" % (w, h), len(got) == 1)
    # area_frac phai GIONG NHAU o hai do phan giai (day la diem cua ty le).
    a = [x for x in det.detect(
        put_rect(canvas(320, 240), 160, 120, 40, 30, (38, 52, 185))) if x.label == "red"][0]
    b = [x for x in det.detect(
        put_rect(canvas(640, 480), 320, 240, 80, 60, (38, 52, 185))) if x.label == "red"][0]
    check("area_frac khong doi theo do phan giai",
          abs(a.area_frac - b.area_frac) < 0.005,
          "%.4f vs %.4f" % (a.area_frac, b.area_frac))

    # Cai tren MOI chi chung minh vet TO van bat duoc. Con phai chung minh vet
    # NHO bi loai DUNG theo ty le -- neu khong, doi min_area_frac thanh nguong
    # diem anh se khong ai phat hien (da vap: test cu chi co cham 1 diem anh,
    # ma contourArea cua no bang 0.0 nen loc kieu gi cung lot).
    #
    # min_area_frac mac dinh 0.002. Tai 320x240 = 76800 px -> nguong 154 px.
    prof = next(p for p in DEFAULT_PROFILES if p.name == "red")
    n_px = 320 * 240 * prof.min_area_frac
    print("     nguong tai 320x240 = %.0f px" % n_px)
    nho = put_rect(canvas(320, 240), 160, 120, 10, 10, (38, 52, 185))   # 100 px
    to = put_rect(canvas(320, 240), 160, 120, 20, 20, (38, 52, 185))    # 400 px
    check("vet 100 px (duoi nguong ty le) bi LOAI",
          len([x for x in det.detect(nho) if x.label == "red"]) == 0,
          "nguong dang tinh theo diem anh chu khong theo ty le?")
    check("vet 400 px (tren nguong) duoc GIU",
          len([x for x in det.detect(to) if x.label == "red"]) == 1)

    print()
    print("== (5) Nhieu hat le bi loai, khong bao vet gia ==")
    img = canvas()
    rng = np.random.default_rng(0)
    for _ in range(300):
        x, y = int(rng.integers(0, 320)), int(rng.integers(0, 240))
        img[y, x] = (38, 52, 185)          # cham do 1 diem anh
    check("300 cham le -> khong vet nao", len(det.detect(img)) == 0,
          "bao %d vet" % len(det.detect(img)))

    print()
    print("== (6) Anh trong -> list rong, khong no ==")
    check("anh nen tron", det.detect(canvas()) == [])
    check("None -> []", det.detect(None) == [])

    print()
    print("== (7) draw() KHONG sua anh goc ==")
    img = put_rect(canvas(), 160, 120, 60, 60, (38, 52, 185))
    before = img.copy()
    det.draw(img, det.detect(img))
    check("anh goc nguyen ven sau khi ve overlay",
          bool(np.array_equal(img, before)),
          "draw() dang ve de len anh goc -- lan detect sau se sai mau")

    print()
    print("== (8) Tat/bat tung mau ==")
    img = put_rect(canvas(), 100, 120, 50, 50, (38, 52, 185))
    put_rect(img, 230, 120, 50, 50, (60, 165, 70))
    check("bat ca hai -> 2 vet", len(det.detect(img)) == 2)
    det.enabled["green"] = False
    got = det.detect(img)
    check("tat 'green' -> con 1 vet red", len(got) == 1 and got[0].label == "red")
    det.enabled["green"] = True

    print()
    print("== (9) H toi da la 179, khong phai 255/359 ==")
    check("HUE_MAX = 179", HUE_MAX == 179, "dang la %s" % HUE_MAX)
    for p in DEFAULT_PROFILES:
        for lo, hi in p.hsv_ranges:
            check("profile '%s': H trong 0..179" % p.name,
                  0 <= lo[0] <= HUE_MAX and 0 <= hi[0] <= HUE_MAX,
                  "khoang (%s,%s)" % (lo, hi))

print()
if FAILED:
    print("KET QUA: %d FAIL -- %s" % (len(FAILED), FAILED[0]))
    sys.exit(1)
print("KET QUA: TAT CA PASS" if not SKIP else "KET QUA: BO QUA")
