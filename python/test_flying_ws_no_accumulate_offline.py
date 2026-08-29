"""W/S trong FLYING = offset TAM THOI, KHONG duoc cong don.

VI SAO CAN TEST NAY -- day la loi da do duoc tren log bay THAT:

    THR=999  -> 999 -> 1799 -> 2000 -> 2000 -> 2000   (MHR=0, kich tran)
    THRCORR=-1 -> -1 -> 799  -> 1000 -> 1000

Nguyen nhan: GUI gui '@THR OFFSET 100' LAP LAI moi 100ms khi nguoi dung GIU
phim -- bat buoc phai lap, vi firmware co watchdog BENCH_OFFSET_STALE_US se tu
ve 0 neu khong nghe thay gi. Ban cu doc moi goi keepalive nhu mot lenh MOI va
CONG DON vao s_flying_throttle_latch:

    s_flying_throttle_latch += ws_duty;     // SAI

5 lan keepalive = +500 duty. Va vi latch la trang thai BEN VUNG, nha phim ra
throttle KHONG tu tra ve -- no o nguyen tren tran. Drone vot len khong phanh.

Ban chat cua loi: keepalive la co che GIU LENH SONG, khong phai mot lenh moi.
Dem so lan lap lai cua cung mot y dinh la dem nham.

HOP DONG DUNG:
    giu phim -> throttle = ga_nen + 100
    nha phim -> throttle = ga_nen
    giu lau  -> VAN LA ga_nen + 100 (khong doi theo thoi gian)
"""
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CORE_C = (ROOT / "components/flight_core/src/flight_core.c").read_text(encoding="utf-8")
GUI_PY = (ROOT / "tools/uav_udp_console.py").read_text(encoding="utf-8")

fails = []


def check(name, cond, detail=""):
    if cond:
        print("  PASS  %s" % name)
    else:
        print("  FAIL  %s%s" % (name, ("  -- " + detail) if detail else ""))
        fails.append(name)


def strip_c_comments(src):
    """Bo comment truoc khi kiem 'code co lam X khong'.

    Khoi giai thich o flight_core.c CO NHAC LAI dong sai cu de canh bao nguoi
    doc, nen kiem thang tren van ban se bao dong gia.
    """
    src = re.sub(r"/\*.*?\*/", "", src, flags=re.S)
    return re.sub(r"//[^\n]*", "", src)


CODE = strip_c_comments(CORE_C)

# =============================================================================
# (1) FIRMWARE: khong duoc cong don vao latch
# =============================================================================
print("== (1) Firmware: W/S FLYING khong cong don ==")

# Trich rieng nhanh FLYING de khong lan voi nhanh HOLDING (cho do W/S di duong
# khac han -- lenh Vz, khong phai offset duty).
i_fly = CODE.find("case FSM_FLYING:")
check("tim thay nhanh FSM_FLYING", i_fly != -1)
if i_fly != -1:
    # Cat toi cuoi case (case ke tiep hoac het switch).
    j = CODE.find("case FSM_", i_fly + 10)
    fly = CODE[i_fly:j] if j != -1 else CODE[i_fly:i_fly + 6000]

    # ⚠ DONG SAI CU. Neu no quay lai thi test do ngay.
    # Nham vao PHEP GAN VAO LATCH, khong phai moi lan chuoi do xuat hien:
    # `throttle_cmd = clampi(latch + ws_duty, ...)` la dong DUNG (doc latch lam
    # ga nen). Chi `latch = ... latch + ws_duty ...` moi la loi.
    m_accum = re.search(
        r"s_flying_throttle_latch\s*=\s*clampi\(\s*s_flying_throttle_latch\s*\+", fly)
    check("KHONG con `latch = latch + ws_duty` (dong da gay loi tren log)",
          m_accum is None,
          "keepalive moi 100ms se cong don -> kich tran -> khong tra ve")

    # Offset phai di vao throttle cua TICK NAY.
    check("offset cong vao throttle_cmd cua tick hien tai",
          "throttle_cmd = clampi(s_flying_throttle_latch + ws_duty" in fly,
          "phai doc latch lam GA NEN, khong ghi de no")

    # Va latch phai duoc GIU NGUYEN trong nhanh W/S.
    i_ws = fly.find("if (s_bench_throttle_offset != 0)")
    check("tim thay khoi W/S trong FLYING", i_ws != -1)
    if i_ws != -1:
        ws_block = fly[i_ws:i_ws + 700]
        check("khoi W/S KHONG ghi vao s_flying_throttle_latch",
              "s_flying_throttle_latch =" not in ws_block,
              "ghi vao latch = bien offset tam thoi thanh trang thai ben vung")

    # Geofence van phai con: giu W len toi tran do cao thi phai ngung an.
    check("van chan khi cham tran do cao", "fly_alt_max" in fly)
    check("van chan khi cham san do cao", "alt_min_m" in fly)

# =============================================================================
# (2) FIRMWARE: PID do cao KHONG chay trong FLYING
# =============================================================================
# Nguoi dung nhan manh nhieu lan. Neu alt_hold lai lai throttle thi offset W/S
# se danh nhau voi PID.
print("\n== (2) FLYING: PID do cao khong chay ==")
if i_fly != -1:
    check("hold_driving = false trong FLYING",
          "hr.hold_driving = false;" in fly,
          "alt_hold KHONG duoc lai throttle o FLYING")

# =============================================================================
# (3) GUI: keepalive + tra ve 0 khi nha phim
# =============================================================================
print("\n== (3) GUI: giu/nha phim ==")

# Khong ghim con so: bien do nay la num nguoi dung chinh theo cam giac bay
# (da qua 100 -> 200 -> 100 -> 150). Chi chan hai dau vo ly.
m_ws = re.search(r"WS_THROTTLE_OFFSET_DUTY\s*=\s*(\d+)", GUI_PY)
check("tim thay WS_THROTTLE_OFFSET_DUTY", m_ws is not None)
if m_ws:
    ws_duty = int(m_ws.group(1))
    check("bien do offset nam trong khoang hop ly (50..400)",
          50 <= ws_duty <= 400, str(ws_duty))
check("GUI co keepalive khi giu phim", "_ws_tick" in GUI_PY)
check("nha phim -> gui 0", "_send_thr_offset(0)" in GUI_PY)

# Keepalive PHAI nhanh hon watchdog stale ben firmware, neu khong giu phim se
# bi ngat quang.
m_ka = re.search(r"WS_OFFSET_KEEPALIVE_MS\s*=\s*(\d+)", GUI_PY)
check("tim thay WS_OFFSET_KEEPALIVE_MS", m_ka is not None)
m_stale = re.search(r"#define\s+BENCH_OFFSET_STALE_US\s+\(?\(?int64_t\)?\)?\s*(\d+)",
                    (ROOT / "components/flight_core/include/flight_core/tuning.h")
                    .read_text(encoding="utf-8"))
if m_ka and m_stale:
    ka_ms = int(m_ka.group(1))
    stale_ms = int(m_stale.group(1)) / 1000.0
    check("keepalive nhanh hon watchdog stale",
          ka_ms * 2 <= stale_ms,
          "keepalive=%dms stale=%.0fms" % (ka_ms, stale_ms))

# =============================================================================
# (4) MO PHONG chuoi keepalive da gay loi
# =============================================================================
print("\n== (4) Mo phong: giu phim 1s (10 lan keepalive) ==")

GA_NEN = 1000
OFFSET = ws_duty if m_ws else 150
MAX = 2000
MIN = 400


def tick_dung(latch, offset):
    """Cach DUNG: latch la ga nen, offset chi anh huong tick nay."""
    thr = latch + offset
    return latch, max(MIN, min(MAX, thr))


def tick_sai(latch, offset):
    """Cach SAI (ban cu): cong don vao latch."""
    latch = max(MIN, min(MAX, latch + offset))
    return latch, latch


# Giu phim 1s = 10 lan keepalive (100ms/lan).
latch_d = latch_s = GA_NEN
for _ in range(10):
    latch_d, thr_d = tick_dung(latch_d, OFFSET)
    latch_s, thr_s = tick_sai(latch_s, OFFSET)

check("DUNG: giu 1s -> throttle van la ga_nen + offset", thr_d == GA_NEN + OFFSET,
      "duoc %d, ky vong %d" % (thr_d, GA_NEN + OFFSET))
check("DUNG: ga nen KHONG bi doi", latch_d == GA_NEN, str(latch_d))
check("(chung minh loi cu) SAI: giu 1s -> kich tran", thr_s == MAX, str(thr_s))

# Nha phim: offset ve 0.
latch_d, thr_d = tick_dung(latch_d, 0)
latch_s, thr_s = tick_sai(latch_s, 0)
check("DUNG: nha phim -> tra ve dung ga nen", thr_d == GA_NEN, str(thr_d))
check("(chung minh loi cu) SAI: nha phim -> VAN kich tran", thr_s == MAX, str(thr_s))

# Giu lau hon nua cung khong doi gi.
latch_d2 = GA_NEN
for _ in range(100):        # 10s
    latch_d2, thr_d2 = tick_dung(latch_d2, OFFSET)
check("DUNG: giu 10s van la ga_nen + offset (khong troi theo thoi gian)",
      thr_d2 == GA_NEN + OFFSET, str(thr_d2))

# Chieu am.
latch_d3 = GA_NEN
for _ in range(10):
    latch_d3, thr_d3 = tick_dung(latch_d3, -OFFSET)
check("DUNG: giu S 1s -> ga_nen - offset", thr_d3 == GA_NEN - OFFSET, str(thr_d3))

# =============================================================================
# (5) TRAN TOC DO TANG GA (150 duty/s) -- guard khong duoc nhay bac
# =============================================================================
# Log bay THAT: bay o ~23cm, duoi TERR_MIN_CLEARANCE_M (25cm) nen guard khoang
# ho kich hoat MOI TICK va moi lan ghi de latch:
#     THR=995 -> 995 -> 1095 -> 1995 -> 2000 -> 2000   (MHR=0)
#     CLR=0.224  0.226  0.229  0.232  0.238  0.246     (luon < 0.25)
# +900 duty trong MOT tick 5ms. PID khong sinh ra so do -- VZP=2.80 VZI=-6.97
# dung yen ca doan -> nguon la guard ghi de latch, khong phai W/S.
print("\n== (5) Tran toc do tang ga ==")

TUNING = (ROOT / 'components/flight_core/include/flight_core/tuning.h').read_text(encoding='utf-8')
check('co hang so THROTTLE_MAX_RISE_DUTY_PER_S',
      'THROTTLE_MAX_RISE_DUTY_PER_S' in TUNING)
m_rise = re.search(r'THROTTLE_MAX_RISE_DUTY_PER_S' + chr(92) + 's+([0-9.]+)f', TUNING)
check('doc duoc gia tri', m_rise is not None)
if m_rise:
    rise = float(m_rise.group(1))
    # Khong ghim mot con so: day la num nguoi dung chinh theo cam giac bay.
    # Chi chan hai dau vo ly. Chan tren 800: cao hon the thi tu hover (~1000)
    # len tran mat < 1.3s -- gan nhu khong con la mot tran toc do.
    check('gioi han nam trong khoang hop ly (100..800 duty/s)',
          100.0 <= rise <= 800.0, str(rise))

if i_fly != -1:
    check('guard co ap tran toc do tang', 'rise_cap' in fly,
          'khong co tran -> guard lai nhay bac len tran nhu log')
    check('tran tinh theo dt (khong theo so tick)',
          'THROTTLE_MAX_RISE_DUTY_PER_S * dt' in fly,
          'tinh theo tick se cho toc do khac nhau khi dt thay doi')
    # CHI chan chieu tang. Ha ga la duong thoat an toan.
    check('chi chan chieu TANG (khong chan giam)',
          'if (guard_duty > rise_cap)' in fly,
          'chan chieu giam = tao che do hong moi')

# --- Cach tich luy credit PHAI duoc dung (khong (int) thang) ---
if i_fly != -1:
    check('co bo tich luy credit (khong lam tron nguoi moi tick)',
          's_flying_rise_credit' in fly,
          '150*0.004 = 0.6 -> (int) = 0 -> ga KHONG BAO GIO tang duoc')
    check('credit bi tru sau khi tieu', 's_flying_rise_credit -=' in fly,
          'khong tru thi credit tich mai roi cho mot buoc nhay lon')

RISE = rise if m_rise else 500.0
# --- Mo phong dung co che credit ---
DT = 0.004
latch = 995
credit = 0.0
n = 0
while latch < 2000 and n < 200000:
    credit += RISE * DT
    cap = latch + int(credit)
    new_latch = min(2000, cap)          # guard doi kich tran
    used = new_latch - latch
    if used > 0:
        credit -= used
    latch = new_latch
    n += 1
secs = n * DT
# 995 -> 2000 la 1005 duty. Thoi gian ky vong = 1005 / RISE.
want = 1005.0 / RISE
check('thoi gian len tran khop tran toc do (%.2fs)' % want,
      want * 0.8 <= secs <= want * 1.25, '%.2fs (ky vong %.2fs)' % (secs, want))

# Bang chung tran THAT SU chan: mot tick khong duoc nhay qua 1 duty.
latch2, credit2 = 995, 0.0
credit2 += RISE * DT
step = min(2000, latch2 + int(credit2)) - latch2
max_step = int(RISE * DT) + 1
check('mot tick 4ms khong nhay qua %d duty' % max_step, step <= max_step, str(step))

print("")
if fails:
    print("KET QUA: %d FAIL -- %s" % (len(fails), ", ".join(fails)))
    sys.exit(1)
print("KET QUA: ALL PASS")
print("         giu phim = ga_nen +/- %d, nha phim = ga_nen. Khong cong don." % OFFSET)
