"""Guard khoang ho phai REBASE duoc -- co chet khong duoc dung lam dieu kien chan.

LOI THAT DA SUA
---------------
alt_estimator_terrain_rebase() tung mo dau bang:

    if (!e || !e->floor_locked || !e->tof_ground_ref_valid) return false;

`tof_ground_ref_valid` duoc set boi MAY DO SAN (floor_add + Welford). May do san
DA BI XOA HAN khoi repo. Grep toan bo source chi con DUNG MOT cho cham vao co do:

    clear_runtime():  e->tof_ground_ref_valid = old.tof_ground_ref_valid;

Tuc no tu copy chinh minh. KHONG AI set true bao gio -> co vinh vien false ->
ham LUON return false.

HAU QUA: guard khoang ho toi thieu (flight_core.c, B8) van ep leo duoc, nhung
nhanh REBASE cua no chet cung. Moi lan guard ban, log in:

    "(khong rebase duoc: ToF khong dung duoc)"

dung nhu the cam bien hong -- trong khi ToF hoan toan binh thuong. Nguoi doc log
se di tim loi o phan cung.

Day la loai loi TE NHAT de tu tim ra: khong crash, khong warning luc build,
test cu van xanh, va thong bao loi lai TRO SAI HUONG.

Cung ly do do: `tof_ground_range_m` gio chi con duoc gan 0.0f, nen phep tru no
trong rebase la vo nghia -- ToF do TUYET DOI.
"""
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
EST_C = (ROOT / "components/flight_core/src/alt_estimator.c").read_text(encoding="utf-8")
CORE_C = (ROOT / "components/flight_core/src/flight_core.c").read_text(encoding="utf-8")

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
CORE = strip_c_comments(CORE_C)

print("== (1) Khong duoc CHAN bang mot co ma khong ai set true ==")
# Voi moi co dang bi dung lam dieu kien chan trong rebase, doi hoi phai ton tai
# it nhat mot cho GAN THAT (khong phai self-copy tu `old.`).
m = re.search(r"bool alt_estimator_terrain_rebase[^{]*\{(.*?)\n\}", CODE, re.S)
check("tim thay alt_estimator_terrain_rebase()", m is not None)
body = m.group(1) if m else ""

gates = set(re.findall(r"!e->(\w+)", body))
for flag in sorted(gates):
    writes = re.findall(r"%s\s*=\s*([^;]+);" % re.escape(flag), CODE)
    real = [w for w in writes if "old." not in w and w.strip() != "0.0f"]
    check("gate '%s' co nguon set that" % flag,
          len(real) > 0,
          "khong ai set -> nhanh rebase chet cung, va log bao nham la ToF hong")

print()
print("== (2) Khong dung field da chet trong phep tinh ==")
check("khong tru tof_ground_range_m (chi con duoc gan 0.0f)",
      "tof_ground_range_m" not in body,
      "may do san da xoa -> field nay luon 0 -> phep tru vo nghia")

print()
print("== (3) Van giu cac gate CON Y NGHIA ==")
check("giu floor_locked", "floor_locked" in body)
check("giu gate hinh hoc MIN_RANGE", "ALT_EST_TOF_MIN_RANGE_M" in body)
check("giu gate hinh hoc MAX_RANGE", "ALT_EST_TOF_MAX_RANGE_M" in body)
check("KHONG doi tof_fusable",
      "tof_fusable" not in body,
      "dung luc guard ban thi fusable thuong DANG bi tat -- doi no la tu khoa minh")

print()
print("== (4) Guard khoang ho van goi rebase ==")
check("flight_core goi alt_estimator_terrain_rebase()",
      "alt_estimator_terrain_rebase" in CORE)
check("goi o CANH LEN cua guard (khong moi tick)",
      re.search(r"if\s*\(!s_terr_guard_active\)\s*\{[^}]*terrain_rebase", CORE, re.S) is not None,
      "rebase moi tick se lam terr_commit_count chay loan")

print()
if fails:
    print("KET QUA: %d FAIL -- %s" % (len(fails), ", ".join(fails)))
    sys.exit(1)
print("KET QUA: ALL PASS -- rebase chay duoc, khong bi co chet chan")
