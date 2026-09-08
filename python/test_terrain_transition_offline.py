"""NHOM A -- terrain transition fail-safe, thu tu commit, clearance doc lap.

BUG GOC (log bay: terrain -> ground):
    TOFF_cu = 0.693,  TRES: +0.365 +0.194 -0.043 -0.029 -0.039
    Residual DA hoi tu ve ~0 (ung vien dung), nhung TTMO=1 -> huy.
    Sau do ToF duoc fuse LAI bang TOFF cu:
        raw 1.01 + 0.693 = 1.70   vs   alt_m coast 1.09
    -> estimator bi keo nhay 0.5m -> FAULT=1 -> LANDING.

BA LOI DOC LAP, sua rieng tung cai:

  (1) TIMEOUT KHONG FAIL-SAFE
      Het han -> quay lai fuse bang offset CU. Nhung "het han" nghia la ta
      BIET be mat da doi ma KHONG BIET doi bao nhieu -- offset cu la RAC.

  (2) TIMEOUT DEM BANG DONG HO, COMMIT DEM BANG MAU
      timeout kiem moi tick dieu khien (250Hz = 65 lan/260ms);
      confirm chi chay khi co MAU ToF MOI (25Hz = 6-7 lan).
      Ung vien HOP LE bi vut chi vi ToF truot vai mau.

  (3) CLEARANCE THUA HUONG SAI SO CUA Z TUYET DOI
      agl = alt_m - terrain_off. Ca hai ve cung mang mot sai so nen no KHONG
      trieu tieu. Log: drone cach mat ban 0.019m, cong thuc cu cho 0.712m
      -> touchdown khong bao gio kich hoat.
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

print("== (1) TIMEOUT FAIL-SAFE: khong tai dung offset cu ==")
check("co co terr_offset_stale", "terr_offset_stale" in HDR)
m = re.search(r"TERR_PENDING_TIMEOUT_MS \* 1000\) \{(.*?)\n    \}", CODE, re.S)
check("tim thay than khoi timeout", m is not None)
if m:
    body = m.group(1)
    check("timeout SET terr_offset_stale = true",
          re.search(r"terr_offset_stale\s*=\s*true", body) is not None,
          "khong set -> ToF fuse lai bang offset cu -> nhay gia 0.5m")
    check("timeout KHONG dung terrain_off_m",
          "terrain_off_m = " not in body.replace("terr_cand_offset_m = e->terrain_off_m", ""),
          "commit mot offset chua xac nhan con te hon giu offset cu")

check("stale CHAN fusion (tof_fusable = false)",
      re.search(r"terr_offset_stale\)\s*\{[^}]*tof_fusable\s*=\s*false", CODE, re.S) is not None,
      "khong chan thi co nay vo nghia")

print()
print("== (2) COMMIT duoc danh gia TRUOC khi timeout huy ==")
check("co bo dem terr_samples_seen", "terr_samples_seen" in HDR)
check("hang so TERR_MIN_SAMPLES_BEFORE_TIMEOUT ton tai",
      "TERR_MIN_SAMPLES_BEFORE_TIMEOUT" in HDR)
check("timeout doi them dieu kien so mau",
      re.search(r"terr_samples_seen\s*>=\s*TERR_MIN_SAMPLES_BEFORE_TIMEOUT", CODE) is not None,
      "chi doi dong ho -> ung vien hop le bi vut khi ToF truot mau")
check("bo dem tang trong khoi confirm",
      re.search(r"if\s*\(e->terr_pending\)\s*\{[^}]*terr_samples_seen\+\+", CODE, re.S) is not None)
# thu tu trong file: confirm phai NAM TRUOC timeout
i_confirm = CODE.find("terr_confirm_cnt >= TERR_CONFIRM_N")
i_timeout = CODE.find("TERR_PENDING_TIMEOUT_MS * 1000")
check("confirm nam TRUOC timeout trong luong thuc thi",
      i_confirm != -1 and i_timeout != -1 and i_confirm < i_timeout,
      "timeout chay truoc = ung vien hop le mat oan")

print()
print("== (3) CLEARANCE doc lap voi terrain_offset ==")
for fn in ("alt_estimator_agl_m", "alt_estimator_tof_agl_m"):
    mm = re.search(r"float %s\(const alt_estimator_t \*e\)\s*\{(.*?)\n\}" % fn, CODE, re.S)
    check("tim thay %s()" % fn, mm is not None)
    if mm:
        b = mm.group(1)
        check("%s uu tien so do THO (tof_vertical_m)" % fn,
              "tof_vertical_m" in b,
              "van tru terrain_off -> clearance thua huong sai so cua Z")
        check("%s van co duong rot ve khi ToF ngoai tam" % fn,
              "terrain_off_m" in b,
              "mat duong rot ve = khong co so nao khi ToF hong")

print()
print("== (4) Co duoc XOA khi offset dang tin tro lai ==")
check("commit thanh cong -> stale = false",
      re.search(r"terr_commit_count\+\+;\s*\n\s*(?://[^\n]*\n\s*)*e->terr_offset_stale\s*=\s*false",
                EST_C) is not None,
      "khong xoa thi mot lan timeout se khoa ToF vinh vien")
n_clear = len(re.findall(r"terr_offset_stale\s*=\s*false", CODE))
check("co it nhat 3 cho xoa (commit / rebase / reset)", n_clear >= 3,
      "chi co %d" % n_clear)

print()
print("== (5) MO PHONG so lieu THAT tu log ==")
JUMP, TOL, N, DT, TMO = 0.12, 0.10, 3, 0.040, 0.260
MINS = N + 1


def sim(res_seq, raw_seq, alt_m, terr0, sample_gate):
    """Tra (ket_qua, offset). sample_gate=True = ban moi (doi du mau)."""
    terr, pend, cand, cnt, seen, t, t0 = terr0, False, 0.0, 0, 0, 0.0, None
    for res, raw in zip(res_seq, raw_seq):
        if abs(res) > JUMP:
            if not pend:
                pend, cand, t0, seen = True, terr, t, 0
            cand -= res
            cnt = 0
        if pend:
            seen += 1
            cnt = cnt + 1 if abs(raw + cand - alt_m) < TOL else 0
            if cnt >= N:
                return ("COMMIT", cand)
            expired = (t - t0) > TMO
            if expired and (not sample_gate or seen >= MINS):
                return ("TIMEOUT", terr)
        t += DT
    return ("PENDING", cand)


RES = [0.365, 0.194, -0.043, -0.029, -0.039, -0.020, -0.015]
RAW = [0.50, 0.75, 1.01, 1.01, 1.01, 1.01, 1.01]
ALT, TERR0 = 1.09, 0.693

r_new, off_new = sim(RES, RAW, ALT, TERR0, sample_gate=True)
check("ban MOI: commit duoc tren du lieu log", r_new == "COMMIT",
      "ket qua = %s" % r_new)
if r_new == "COMMIT":
    zt = RAW[-1] + off_new
    check("Z lien tuc: |tof_z - alt_m| < %.2f (duoc %.3f)" % (TOL, abs(zt - ALT)),
          abs(zt - ALT) < TOL,
          "offset=%.3f -> tof_z=%.3f vs alt_m=%.3f" % (off_new, zt, ALT))

print()
print("== (6) TEST D: timeout that su -> KHONG tai dung offset cu ==")
# residual khong bao gio hoi tu -> phai timeout
BAD_RES = [0.365, 0.30, 0.28, 0.31, 0.29, 0.30, 0.32, 0.31]
BAD_RAW = [0.50, 0.80, 1.10, 1.40, 1.70, 2.00, 2.30, 2.60]
r_bad, off_bad = sim(BAD_RES, BAD_RAW, ALT, TERR0, sample_gate=True)
check("residual khong hoi tu -> TIMEOUT (dung)", r_bad == "TIMEOUT",
      "ket qua = %s" % r_bad)
check("sau timeout, code danh dau stale thay vi fuse tiep",
      re.search(r"terr_offset_stale\s*=\s*true", CODE) is not None)

print()
if fails:
    print("KET QUA: %d FAIL -- %s" % (len(fails), ", ".join(fails)))
    sys.exit(1)
print("KET QUA: ALL PASS -- transition fail-safe, commit uu tien, clearance doc lap")
