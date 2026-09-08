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
# ⚠ Kiem Y DINH, khong kiem CACH VIET.
# Truoc day dieu kien la `if (!e->terr_pending_since_us)`. Sau khi sua loi
# "cand_offset phai cong don" (xem test_terrain_accumulate_offline.py), canh len
# duoc nhan bang `if (!e->terr_pending)` va moc dat trong do. Ca hai deu dat
# moc DUNG MOT LAN o canh len -- yeu cau that su la vay.
_m = re.search(r"if\s*\(!e->terr_pending(?:_since_us)?\)\s*\{?[^}]*?"
               r"terr_pending_since_us\s*=\s*now_us", CODE, re.S)
check("moc thoi gian chi dat o CANH LEN cua nghi ngo", _m is not None,
      "dat lai moi mau = timeout khong bao gio het han = deadlock quay lai")
# Va phai KHONG co duong nao dat moc ngoai canh len.
_all = re.findall(r"terr_pending_since_us\s*=\s*now_us", CODE)
check("chi co DUNG MOT cho dat moc = now_us", len(_all) == 1,
      "co %d cho -- them duong nao khac la co the dat lai giua chung" % len(_all))

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
print("== (8) Residual la BUOC NHAY THO, khong tru chuyen dong cua drone ==")
# ⚠ YEU CAU DA DAO NGUOC so voi ban dau cua test nay.
#
# Ban dau doi `expected = vz_accel_only_ms * tof_dt_s` (tru phan range doi do
# chinh drone leo/ha). Can do lai tren log bay cho thay so hang do LAM HAI:
#     thu no SUA duoc (drone tu leo, 1 mau) :  0.018 m
#     thu no BOM VAO  (sai so cua VZAO)     :  0.082 m
#     bac ban that                          :  0.75 .. 1.20 m
# Chiem 2.4% tin hieu ma sai so gap 5 lan thu no sua. Do TRES tren SAN PHANG
# voi cong thuc cu: dinh +0.108m tren nguong 0.12 -> con 12mm la bao dong gia.
#
# Doi lai: TERR_JUMP_THRESH_M phai noi 0.12 -> 0.20 (bien 2.6x so voi
# |d_range| lon nhat do duoc luc leo, 0.076m). Hai thay doi nay di CUNG NHAU --
# bo so hang accel ma khong noi nguong la doi mot loi lay mot loi khac.
check("residual = d_range (khong tru vz)",
      re.search(r"residual\s*=\s*d_range\s*;", CODE) is not None,
      "phai la buoc nhay THO")
check("KHONG con tru vz_accel_only_ms * tof_dt_s",
      re.search(r"=\s*e->vz_accel_only_ms\s*\*\s*e->tof_dt_s", CODE) is None,
      "so hang nay bom 0.082m sai so vao mot phep do can 0.018m")
# Nguong KHONG can noi len. |d_range| dinh 0.076 chi xay ra o pha LEO ngay sau
# cat canh -- luc do drone duoi 0.3m, khong the o tren vat the nao -- va pha do
# da bi TERR_ARM_AFTER_LIFTOFF_MS khoa. Khi bay bang, dinh chi 0.015..0.026.
_jump = float(const_of(HDR, "TERR_JUMP_THRESH_M"))
check("giu nguong 0.12 (noi len 0.20 lam mat vat the < 0.4m)",
      abs(_jump - 0.12) < 1e-6, "dang la %s" % _jump)
check("co khoa terrain sau cat canh de loai pha leo",
      re.search(r"#define\s+TERR_ARM_AFTER_LIFTOFF_MS\s+[1-9]", HDR) is not None)

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
    # Moc phai la mot TIMESTAMP bat ky, so voi NO_CORRECTION_DEGRADED_MS.
    # KHONG khoa ten bien: ban dau dem tu last_tof_accept_us, nhung moc do bi
    # terr_pending/terr_offset_stale lam DUNG YEN (ca hai ep tof_fusable=false),
    # nen moi lan phat hien dia hinh lai tu dem nguoc toi SOFT FAULT du ToF van
    # khoe. Gio dem tu last_tof_geom_ok_us -- "chip con tra mau hop le" -- doc
    # lap voi trang thai terrain.
    check("degraded tinh theo thoi gian tu mot moc _us",
          re.search(r"last_\w*_us", seg) is not None and
          "ALT_EST_NO_CORRECTION_DEGRADED_MS" in seg,
          "phai dem theo thoi gian, dung hop dong o commander.h")
    check("moc degraded KHONG phai last_tof_accept_us (bi terrain lam dung)",
          "last_tof_accept_us" not in seg,
          "last_tof_accept_us chi refresh khi tof_fusable; terr_pending ep no "
          "false -> dong ho dung -> roi ban la fault sau 300ms")
    check("van giu valid = true (coast bang IMU khong phai loi)",
          re.search(r"valid\s*=\s*true", seg) is not None)

print()
if fails:
    print("KET QUA: %d FAIL -- %s" % (len(fails), ", ".join(fails)))
    sys.exit(1)
print("KET QUA: ALL PASS -- terr_pending co loi thoat va khong gay fault tuc thi")
