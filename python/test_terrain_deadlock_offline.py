"""PHASE B -- terr_pending PHAI co loi thoat. Day la loi da lam terrain bi TAT.

LOI GOC (do duoc tren bo, ghi trong app_config.h):
    TOFF=-0.315  TPEND=1  TCMT=1  TOFFUSE=0  TOFTRACK=0
    TOFR 83->95 (tang deu)   TOFA=177..178 (DUNG YEN)

VONG LUAN QUAN:
    terr_pending = true  -> tof_fusable = false      (dung theo thiet ke)
    tof_fusable = false  -> khong mau nao qua nhanh confirm
    khong co mau         -> terr_confirm_cnt KHONG tang, cung KHONG reset
    => terr_pending KET O 1 VINH VIEN
    => sau ALT_EST_TOF_LOST_MS khong correction -> valid=false -> ALTSRC=4
    => commander soft-fault -> LANDING GIUA CHUYEN BAY

BA DIEU KIEN PHAI DUNG DONG THOI, thieu mot la loi quay lai:
  (1) co timeout huy nghi ngo
  (2) timeout NGAN HON ALT_EST_TOF_LOST_MS -- neu khong, soft-fault no TRUOC
      khi loi thoat kip chay, va ban va thanh vo dung
  (3) khoi loi thoat nam NGOAI if(tof_new) -- nam trong thi no cung chi chay
      khi co mau moi, dung cai dieu kien no sinh ra de sua
"""
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
EST_C = (ROOT / "components/flight_core/src/alt_estimator.c").read_text(encoding="utf-8")
EST_H = (ROOT / "components/flight_core/include/flight_core/alt_estimator.h").read_text(encoding="utf-8")

fails = []


def check(name, cond, detail=""):
    if cond:
        print("  PASS  %s" % name)
    else:
        print("  FAIL  %s%s" % (name, ("  -- " + detail) if detail else ""))
        fails.append(name)


def strip_c_comments(src):
    src = re.sub(r"/\*.*?\*/", "", src, flags=re.S)
    return re.sub(r"//[^\n]*", "", src)


CODE = strip_c_comments(EST_C)
HDR = strip_c_comments(EST_H)


def const_of(src, name):
    m = re.search(r"#define\s+%s\s+([0-9.]+)f?" % re.escape(name), src)
    return float(m.group(1)) if m else None


print("== (1) Timeout NGAN HON MOI nguong bien 'khong correction' thanh fault ==")
tmo = const_of(HDR, "TERR_PENDING_TIMEOUT_MS")
lost = const_of(HDR, "ALT_EST_TOF_LOST_MS")
# ⚠ ALT_EST_NO_CORRECTION_DEGRADED_MS moi la nguong chan TRUOC. Ban dau test
# nay chi so voi ALT_EST_TOF_LOST_MS -- so NHAM, va da bo lot dung lo hong
# no sinh ra de bat (xem muc (9)).
degr = const_of(HDR, "ALT_EST_NO_CORRECTION_DEGRADED_MS")
if degr is None:
    # dinh nghia kieu alias: #define A  B
    m = re.search(r"#define\s+ALT_EST_NO_CORRECTION_DEGRADED_MS\s+(\w+)", HDR)
    if m and m.group(1) == "ALT_EST_TOF_LOST_MS":
        degr = lost
check("TERR_PENDING_TIMEOUT_MS co dinh nghia", tmo is not None)
check("ALT_EST_TOF_LOST_MS co dinh nghia", lost is not None)
check("ALT_EST_NO_CORRECTION_DEGRADED_MS giai duoc", degr is not None)
if tmo is not None and lost is not None:
    check("timeout < ALT_EST_TOF_LOST_MS", tmo < lost,
          "timeout=%s >= lost=%s" % (tmo, lost))
if tmo is not None and degr is not None:
    check("timeout < ALT_EST_NO_CORRECTION_DEGRADED_MS (nguong THAT SU)",
          tmo < degr,
          "timeout=%s >= degraded=%s: degraded=true -> commander SOFT FAULT -> "
          "LANDING, TRUOC khi loi thoat kip chay" % (tmo, degr))
if tmo is not None:
    check("timeout du dai cho TERR_CONFIRM_N mau (>= 120ms)",
          tmo >= 120,
          "qua ngan thi khong bao gio confirm kip, moi bac deu bi huy")

print()
print("== (2) Co _Static_assert bao ve rang buoc do ==")
check("_Static_assert(TERR_PENDING_TIMEOUT_MS < ALT_EST_TOF_LOST_MS)",
      re.search(r"_Static_assert\s*\(\s*TERR_PENDING_TIMEOUT_MS\s*<\s*ALT_EST_TOF_LOST_MS",
                CODE) is not None,
      "khong co assert = ai do doi hang so sau nay se lam song lai loi cu, im lang")

print()
print("== (3) Khoi loi thoat NAM NGOAI if(tof_new) -- diem mau chot ==")
mesc = re.search(r"if\s*\(\s*e->terr_pending\s*&&\s*e->terr_pending_since_us", CODE)
check("tim thay khoi loi thoat", mesc is not None)
if mesc:
    # dem do sau ngoac tu dau ham update() toi khoi loi thoat:
    # neu no nam trong if(tof_new) thi phai co mot '{' chua dong truoc do
    mfun = re.search(r"void alt_estimator_update\s*\(", CODE)
    check("tim thay alt_estimator_update()", mfun is not None)
    if mfun:
        body = CODE[mfun.end():mesc.start()]
        depth = body.count("{") - body.count("}")
        # depth 1 = dang o than ham, KHONG nam trong khoi con nao
        check("do sau ngoac = 1 (than ham, khong nam trong if(tof_new))",
              depth == 1,
              "depth=%d: khoi loi thoat dang nam trong mot khoi con -- neu do la "
              "if(tof_new) thi no chi chay khi CO mau moi, dung cai dieu kien "
              "no sinh ra de sua" % depth)

print()
print("== (4) Het han thi HUY, KHONG duoc COMMIT ==")
if mesc:
    seg = CODE[mesc.start():mesc.start() + 1200]
    check("dat terr_pending = false", "terr_pending = false" in seg)
    check("vut ung vien (cand_offset = terrain_off)",
          re.search(r"terr_cand_offset_m\s*=\s*e->terrain_off_m", seg) is not None)
    check("KHONG gan terrain_off_m = terr_cand_offset_m (khong commit)",
          re.search(r"terrain_off_m\s*=\s*e->terr_cand_offset_m", seg) is None,
          "commit mot ung vien CHUA xac nhan = ghi offset co the sai vao trang "
          "thai ben vung -- dung cai da tao ra TOFF=-0.315")
    check("tang terr_timeout_count de theo doi duoc",
          "terr_timeout_count" in seg)

print()
print("== (5) Moc thoi gian dat o CANH LEN, khong dat lai moi tick ==")
check("stamp nam trong if (!e->terr_pending_since_us)",
      re.search(r"if\s*\(!e->terr_pending_since_us\)\s*e->terr_pending_since_us\s*=\s*now_us",
                CODE) is not None,
      "dat lai moi tick = timeout khong bao gio het han = deadlock quay lai")

print()
print("== (6) Moi cho clear terr_pending deu clear ca moc ==")
n_pending = len(re.findall(r"terr_pending\s*=\s*false", CODE))
n_stamp = len(re.findall(r"terr_pending_since_us\s*=\s*0", CODE))
check("so lan clear moc >= so lan clear pending (%d vs %d)" % (n_stamp, n_pending),
      n_stamp >= n_pending,
      "sot mot cho = moc cu con lai = lan nghi ngo sau het han ngay lap tuc")

print()
print("== (7) B3 sanity check chan commit vo ly ==")
maxstep = const_of(HDR, "TERR_MAX_STEP_M")
check("TERR_MAX_STEP_M co dinh nghia", maxstep is not None)
if maxstep is not None:
    check("TERR_MAX_STEP_M trong khoang hop ly (0.5..3.0m)",
          0.5 <= maxstep <= 3.0, "duoc %s" % maxstep)
check("commit duoc bao ve boi fabsf(...) <= TERR_MAX_STEP_M",
      re.search(r"fabsf\s*\(\s*e->terr_cand_offset_m\s*-\s*e->terrain_off_m\s*\)\s*\n?\s*<=\s*TERR_MAX_STEP_M",
                CODE) is not None,
      "khong co sanity = mot mau rac co the ghi offset khong lo")

print()
print("== (8) Residual dung du doan DOC LAP, khong dung vz da fuse ToF ==")
check("expected tinh tu vz_accel_only_ms",
      re.search(r"expected\s*=\s*e->vz_accel_only_ms\s*\*\s*e->tof_dt_s", CODE) is not None,
      "dung vz_ms la vong hoi tiep kin: cu nhay range bom vao vz_ms -> expected "
      "phinh theo dung huong cu nhay -> residual bi triet tieu -> bac that "
      "khong bao gio duoc phat hien")

print()
print("== (9) terr_pending KHONG duoc bien thanh SOFT FAULT ngay tick dau ==")
# CHUOI DAY DU phai lan theo, khong duoc dung o hai hang so:
#   terr_pending -> tof_fusable=false -> update_age() nhanh BRIDGE
#     -> degraded -> cin.alt_estimator_degraded -> commander FAULT_SOFT -> LANDING
# Commander KHONG co debounce o nhanh do, nen neu update_age() dat
# degraded=true ngay o mau dau khong fusable thi MOT mau ToF xau = ha canh.
mbridge = re.search(r"ALT_TOF_BRIDGE(.*?)\n    \}", CODE, re.S)
check("tim thay nhanh BRIDGE trong update_age()", mbridge is not None)
if mbridge:
    seg = mbridge.group(1)
    check("BRIDGE KHONG gan degraded = true vo dieu kien",
          re.search(r"degraded\s*=\s*true\s*;", seg) is None,
          "gan cung true = mot mau ToF khong fusable (ngoai tam / hap thu / "
          "terr_pending) se fault NGAY -> LANDING giua chuyen")
    check("degraded tinh theo thoi gian tu correction cuoi",
          "last_tof_accept_us" in seg and
          "ALT_EST_NO_CORRECTION_DEGRADED_MS" in seg,
          "phai dem tu moc correction cuoi, dung hop dong o commander.h")
    check("van giu valid = true (coast bang IMU khong phai loi)",
          re.search(r"valid\s*=\s*true", seg) is not None)

print()
if fails:
    print("KET QUA: %d FAIL -- %s" % (len(fails), ", ".join(fails)))
    sys.exit(1)
print("KET QUA: ALL PASS -- terr_pending co loi thoat va khong gay fault tuc thi")
