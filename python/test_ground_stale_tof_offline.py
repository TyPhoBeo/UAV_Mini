"""Trang thai ToF DAN XUAT phai bi xoa khi nam dat.

LOI THAT, do duoc tren log cat canh:

    Nam sat san -> driver ngung cap mau HOP LE (TOFAGE 8415->9158ms trong khi
                   TOFALIVE van dao 3..43: chip VAN tra loi I2C, chi la khong
                   con range dung duoc. KHONG phai cong hinh hoc chan --
                   ALT_EST_TOF_MIN_RANGE_M = 0.00 nen 6mm van qua.)
                -> tof_vz_lpf_ms / tof_vz_valid / prev_* DONG BANG

    Do tren log: dung yen 8.4 GIAY (TOFAGE 8415..9158, TOFA=0) ma
        tof_vz_lpf_ms = -1.414 m/s   tof_vz_valid = 1
    van con nguyen tu luc dat drone xuong san.

    alt_estimator_confirm_liftoff():
        vz_ms = tof_vz_valid ? tof_vz_lpf_ms : 0   ->  -1.414
        vz_accel_only_ms = vz_ms                   ->  -1.414

DAY CHUYEN HAU QUA -- vi sao no khong dung lai o do:
    vz_accel_only_ms CO Y khong bao gio duoc ToF sua (no la du doan DOC LAP
    cho residual terrain, xem khoi B4). Nen sai so nay o LAI ca chuyen bay.

    Do tren log:  VZAO ~ -1.2 m/s  trong khi VZ that ~ +0.4  ->  lech +1.28 m/s
    Terrain dung chinh no:  expected = vz_accel_only_ms * tof_dt_s
    dt=45ms -> expected lech 0.057m = 48% cua TERR_JUMP_THRESH_M (0.12)

    Do lai TRES tren SAN PHANG (TOFF=0, khong he co bac dia hinh):
        trung binh +0.078m, dinh +0.108m,  nguong 0.12
    -> chi con 0.012m nua la BAO DONG DIA HINH GIA tren san phang.

Cung khoi sua: refresh last_tof_geom_ok_us khi nam dat, neu khong thi nam cho
8.4s roi cat canh se SOFT FAULT ngay tick dau vi moc hinh hoc da qua han 300ms.
"""
import io
import os
import re
import sys

FAILED = []


def check(name, cond, detail=""):
    print("  %s  %s" % ("PASS" if cond else "FAIL", name))
    if not cond:
        if detail:
            print("        %s" % detail)
        FAILED.append(name)


def read(rel):
    p = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), rel)
    return io.open(p, encoding="utf-8", errors="replace").read()


def strip_c_comments(s):
    """Bo comment truoc khi kiem 'code co lam X khong'.

    Khoi giai thich o day trich dan chinh cac ten bien dang kiem, khong bo thi
    test doc trung comment va bao PASS gia.
    """
    s = re.sub(r"/\*.*?\*/", " ", s, flags=re.S)
    s = re.sub(r"//[^\n]*", " ", s)
    return s


SRC = strip_c_comments(read("components/flight_core/src/alt_estimator.c"))
HDR = read("components/flight_core/include/flight_core/alt_estimator.h")


def const_of(name):
    m = re.search(r"#define\s+" + re.escape(name) + r"\s+([0-9.]+)f?\b", HDR)
    return float(m.group(1)) if m else None


# Nhanh NAM DAT: tu `if (!airborne)` co gan alt_m = 0 cho toi `return`.
i = SRC.find("if (!airborne)")
ground = ""
while i != -1:
    seg = SRC[i:i + 4000]
    if "terrain_off_m" in seg and "return" in seg:
        ground = seg[:seg.find("return") + 8]
        break
    i = SRC.find("if (!airborne)", i + 1)

print("== (1) Tim duoc nhanh nam dat ==")
check("tim thay khoi !airborne co reset terrain", bool(ground),
      "khong dinh vi duoc -> cac kiem tra duoi vo nghia")

if ground:
    print()
    print("== (2) Van toc ToF bi xoa khi nam dat ==")
    check("tof_vz_valid = false",
          re.search(r"tof_vz_valid\s*=\s*false", ground) is not None,
          "khong xoa -> confirm_liftoff tiem van toc cu vao vz_ms")
    check("tof_vz_lpf_ms bi xoa ve 0",
          re.search(r"tof_vz_lpf_ms\s*=\s*0\.0f", ground) is not None,
          "gia tri -1.414 m/s tu 8s truoc van con dung duoc neu khong xoa")

    print()
    print("== (3) Dao ham khong duoc bac qua ranh gioi dat/khong ==")
    check("prev_tof_vertical_valid = false",
          re.search(r"prev_tof_vertical_valid\s*=\s*false", ground) is not None,
          "residual terrain se tinh d_range qua mot khoang cach ca giay")
    check("prev_tof_z_valid = false",
          re.search(r"prev_tof_z_valid\s*=\s*false", ground) is not None)

    print()
    print("== (4) Moc hinh hoc duoc refresh khi nam dat ==")
    check("last_tof_geom_ok_us = now_us trong nhanh nam dat",
          re.search(r"last_tof_geom_ok_us\s*=\s*now_us", ground) is not None,
          "khong refresh -> nam cho 8.4s roi cat canh = SOFT FAULT tick dau")

    print()
    print("== (5) KHONG duoc dung toi tof_fusable o nhanh nay ==")
    # tof_fusable giu ground_tof_alive -> valid -> cong cat canh. Xoa no o day
    # la tu choi cat canh vinh vien.
    assigns = re.findall(r"tof_fusable\s*=\s*(\w+)", ground)
    check("khong gan tof_fusable = false khi nam dat",
          "false" not in assigns,
          "xoa tof_fusable -> ground_tof_alive=0 -> valid=0 -> khong cat canh duoc")

print()
print("== (6) confirm_liftoff doc dung hai co da xoa ==")
m = re.search(r"void\s+alt_estimator_confirm_liftoff\s*\([^)]*\)\s*\{(.*?)\n\}",
              SRC, re.S)
check("tim thay confirm_liftoff", m is not None)
if m:
    body = m.group(1)
    check("vz seed di qua tof_vz_valid",
          "tof_vz_valid" in body,
          "neu doc thang tof_vz_lpf_ms thi co xoa cung vo nghia")

print()
print("== (7) Mo phong day chuyen hau qua ==")

JUMP = const_of("TERR_JUMP_THRESH_M")
check("doc duoc TERR_JUMP_THRESH_M", JUMP is not None, str(JUMP))

if JUMP:
    STALE_VZ = -1.414          # tof_vz_lpf_ms dong bang tren san (log)
    VZ_THAT = 0.40             # toc do leo that (log)

    def expected_bias(seed, dt):
        """Sai so cua `expected = vz_accel_only_ms * dt` do seed sai."""
        return (VZ_THAT - seed) * dt

    for dt in (0.040, 0.045):
        bias_cu = abs(expected_bias(STALE_VZ, dt))
        bias_moi = abs(expected_bias(0.0, dt))
        print("  dt=%.0fms  seed cu %.3f -> lech %.4fm (%.0f%% nguong) | "
              "seed 0 -> lech %.4fm (%.0f%%)"
              % (dt * 1000, STALE_VZ, bias_cu, bias_cu / JUMP * 100,
                 bias_moi, bias_moi / JUMP * 100))

    b_cu = abs(expected_bias(STALE_VZ, 0.045))
    b_moi = abs(expected_bias(0.0, 0.045))
    check("seed cu day residual qua nua nguong (tai hien loi)",
          b_cu > 0.4 * JUMP,
          "khong tai hien duoc thi mo phong sai, khong phai da het loi")
    check("seed 0 giam sai so residual it nhat 3 lan",
          b_moi * 3 <= b_cu,
          "cu=%.4f moi=%.4f" % (b_cu, b_moi))

    # TRES do duoc tren SAN PHANG voi seed cu VA cong thuc cu (co tru vz).
    # ⚠ So voi nguong CU (0.12) chu khong phai nguong hien tai: day la du kien
    # LICH SU giai thich vi sao phai sua, no khong doi khi hang so doi.
    JUMP_CU = 0.12
    tres_do_duoc = [0.076, 0.077, 0.047, 0.071, 0.108, 0.070, 0.088, 0.073]
    dinh = max(tres_do_duoc)
    print("  TRES do tren san phang (seed cu, cong thuc cu): dinh %.3f"
          % dinh)
    print("    voi nguong CU  %.2f -> bien %.3f" % (JUMP_CU, JUMP_CU - dinh))
    print("    voi nguong MOI %.2f -> bien %.3f" % (JUMP, JUMP - dinh))
    check("bien voi nguong CU qua hep (< 2cm) -- ly do phai sua",
          JUMP_CU - dinh < 0.02)
    # Hai thay doi phai di cung nhau: bo so hang accel (het bias 0.082m) VA noi
    # nguong. Chi lam mot trong hai la doi mot loi lay mot loi khac.
    check("nguong hien tai da noi len >= 0.18",
          JUMP >= 0.18,
          "dang la %.2f" % JUMP)

print()
if FAILED:
    print("KET QUA: %d FAIL -- %s" % (len(FAILED), FAILED[0]))
    sys.exit(1)
print("KET QUA: TAT CA PASS")
