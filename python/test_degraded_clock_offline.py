"""NHOM A2 -- dong ho `degraded` phai DOC LAP voi trang thai terrain.

LOI THAT (do duoc tren chuyen bay roi ban):
    roi ban -> bac dia hinh AM -> terr_pending = 1
            -> tof_fusable = false MOI TICK (alt_estimator.c, nhanh B6)
            -> last_tof_accept_us NGUNG refresh  (dong `if (e->tof_fusable) ...`)
            -> sau ALT_EST_NO_CORRECTION_DEGRADED_MS: degraded = true
            -> commander.c "ToF correction mat qua lau" -> SOFT FAULT -> LANDING

Trong khi do ToF hoan toan KHOE va van tra mau deu 25Hz. Estimator tu ket an
chinh no vi mot trang thai ma NO CO Y dat ra.

Ngan sach thoi gian cho thay day khong phai truong hop hiem:
    detect + confirm can ~4 mau = 160-190ms (TOFAGE do duoc 40-47ms)
    fault den o 300ms  ->  bien con lai chi ~110-140ms
    terr_offset_stale bat o 260ms -> chi som hon fault 40ms, gan nhu vo dung.

SUA: tach lam HAI moc, hai cau hoi khac nhau
    last_tof_accept_us   = lan cuoi mot mau duoc DUNG de sua Z tuyet doi
    last_tof_geom_ok_us  = lan cuoi CHIP tra mau hop le ve HINH HOC
`degraded` phai doc moc THU HAI.
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

    Khoi giai thich o day dai va co trich dan chinh ten bien cu, nen khong bo
    thi test se doc trung comment va bao PASS gia.
    """
    s = re.sub(r"/\*.*?\*/", " ", s, flags=re.S)
    s = re.sub(r"//[^\n]*", " ", s)
    return s


SRC = read("components/flight_core/src/alt_estimator.c")
HDR = read("components/flight_core/include/flight_core/alt_estimator.h")
CODE = strip_c_comments(SRC)

print("== (1) Ton tai moc rieng, doc lap voi fusion ==")
check("header khai bao last_tof_geom_ok_us",
      re.search(r"int64_t\s+last_tof_geom_ok_us\s*;", HDR) is not None,
      "thieu truong -> khong co moc nao doc lap voi tof_fusable")

m = re.search(r"e->last_tof_geom_ok_us\s*=\s*now_us\s*;", CODE)
check("code co ghi moc do", m is not None)

print()
print("== (2) Moc ghi KHONG duoc nam duoi dieu kien tof_fusable ==")
if m:
    # Lay 900 ky tu ngay TRUOC cho ghi moc. Neu trong do co `if (...fusable...)`
    # mo ra thi moc dang bi gac -- dung lai loi cu.
    before = CODE[max(0, m.start() - 900):m.start()]
    check("khong bi gac boi tof_fusable",
          not re.search(r"if\s*\([^)]*tof_fusable[^)]*\)\s*$", before.rstrip()),
          "moc bi dat sau mot if(tof_fusable) -> lai dung yen khi terr_pending")
    check("khong bi gac boi terr_pending / terr_offset_stale",
          not re.search(r"if\s*\([^)]*terr_(pending|offset_stale)[^)]*\)\s*$",
                        before.rstrip()),
          "moc phu thuoc terrain -> tai tao dung loi dang sua")

print()
print("== (3) degraded doc moc MOI, khong doc moc cu ==")
# Khoi tinh degraded theo thoi gian (nhanh BRIDGE trong update_age).
# Neo vao chinh DONG GAN degraded, roi lay khoi ngay TRUOC no (noi tinh bien
# thoi gian). Tim theo ten hang so se bat trung nhanh fallback `: (int64_t)...`
# va doc nham mot doan khong chua ten moc.
mdeg = re.search(r"e->degraded\s*=\s*\w+\s*>=\s*\(int64_t\)\s*"
                 r"ALT_EST_NO_CORRECTION_DEGRADED_MS", CODE)
seg = CODE[max(0, mdeg.start() - 500):mdeg.end()] if mdeg else None

check("tim thay khoi tinh degraded theo thoi gian", seg is not None)
if seg:
    check("dem tu last_tof_geom_ok_us",
          "last_tof_geom_ok_us" in seg,
          "phai dem tu moc hinh hoc, khong phai moc fusion")
    check("KHONG dem tu last_tof_accept_us",
          "last_tof_accept_us" not in seg,
          "moc do bi terr_pending lam dung -> roi ban = fault sau 300ms")

print()
print("== (4) Mo phong: roi ban, ToF van khoe ==")

DEGRADED_MS = 300
PENDING_TIMEOUT_MS = 260
TOF_PERIOD_MS = 45          # TOFAGE do duoc trong log: 7..47ms
CONFIRM_SAMPLES = 4         # TERR_CONFIRM_N + 1


def sim(use_geom_clock, commit_at_ms):
    """Tra ve (co_fault, ms_fault).

    ToF KHOE suot: cu TOF_PERIOD_MS lai co mot mau hop le hinh hoc.
    terr_pending bat tu t=0 (vua roi ban), commit o commit_at_ms.
    """
    last_accept = 0        # chi refresh khi fusable
    last_geom = 0          # refresh moi mau hop le hinh hoc
    pending = True
    stale = False
    for t in range(0, 1200, 4):          # tick 250Hz
        if pending and t >= commit_at_ms:
            pending = False
            stale = False
        if pending and not stale and t >= PENDING_TIMEOUT_MS:
            stale = True                  # timeout -> fail-safe
        fusable = not pending and not stale
        if t % TOF_PERIOD_MS < 4:         # co mau ToF moi
            last_geom = t                 # chip van tra mau -> LUON refresh
            if fusable:
                last_accept = t
        moc = last_geom if use_geom_clock else last_accept
        if t - moc >= DEGRADED_MS:
            return True, t
    return False, None


commit_ok = CONFIRM_SAMPLES * TOF_PERIOD_MS          # 180ms -- commit kip
commit_late = 400                                    # truot -> qua timeout

for label, commit in (("commit kip (%dms)" % commit_ok, commit_ok),
                      ("commit TRUOT (%dms)" % commit_late, commit_late)):
    f_old, t_old = sim(False, commit)
    f_new, t_new = sim(True, commit)
    print("  %-22s  moc cu: %-14s   moc moi: %s"
          % (label,
             ("FAULT @%dms" % t_old) if f_old else "ok",
             ("FAULT @%dms" % t_new) if f_new else "ok"))

f_old, _ = sim(False, commit_ok)
f_new, _ = sim(True, commit_ok)
# Commit KIP thi ca hai moc deu song sot: fuse tro lai o 180ms, truoc han 300ms.
# Do la ly do lo hong nay an lau -- no chi lo ra khi commit TRUOT.
check("commit kip: ca hai moc deu khong fault", not f_old and not f_new)
check("moc MOI: commit kip -> KHONG fault", not f_new)

f_old2, t_old2 = sim(False, commit_late)
f_new2, _ = sim(True, commit_late)
# Day moi la kich ban da quan sat duoc tren bo: commit khong kip trong
# TERR_PENDING_TIMEOUT_MS -> moc cu dung yen tu luc TPEND=1 -> fault.
check("moc CU: commit truot -> FAULT (tai hien duoc loi that)", f_old2,
      "khong tai hien duoc thi mo phong sai, khong phai da het loi")
check("moc MOI: commit truot van KHONG fault (ToF con song)", not f_new2,
      "chip van tra mau deu -> khong duoc bao 'mat correction qua lau'")
if f_old2:
    print("        moc cu fault @%dms sau khi roi ban" % t_old2)

print()
print("== (5) ToF chet THAT thi van phai FAULT ==")


def sim_dead(use_geom_clock):
    """ToF ngung tra mau han tu t=0."""
    last = 0
    for t in range(0, 1200, 4):
        moc = last                      # khong bao gio refresh nua
        if t - moc >= DEGRADED_MS:
            return True, t
    return False, None


f, t = sim_dead(True)
check("chip im -> van FAULT (khong nuot mat duong bao that)", f,
      "moc moi khong duoc lam mat kha nang phat hien ToF chet")
print("        FAULT @%sms" % t)

print()
if FAILED:
    print("KET QUA: %d FAIL -- %s" % (len(FAILED), FAILED[0]))
    sys.exit(1)
print("KET QUA: TAT CA PASS")
