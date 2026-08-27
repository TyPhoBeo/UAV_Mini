"""Kiem tra STATUS_RE (tools/uav_udp_console.py) parse DUNG dong STATUS ma
firmware (src/telemetry_format.c) thuc su phat ra, va cac group index dung
trong _handle_line() tro dung field.

Day la test THAT: no import chinh module GUI dang ship va dung chinh chuoi
dinh dang lay tu telemetry_format.c.
"""
import importlib.util
import pathlib
import sys

# File nay nam trong python/ nen repo root la thu muc cha.
REPO = pathlib.Path(__file__).resolve().parents[1]

spec = importlib.util.spec_from_file_location(
    "uav_udp_console", REPO / "tools" / "uav_udp_console.py")
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)

# Dung dinh dang cua telemetry_format.c, moi field mot gia tri PHAN BIET duoc
# de bat loi lech index (neu lech 1 o thi gia tri se khong khop).
line = (
    "ARM=1 THR=1234 | R=1.10 P=2.20 Y=3.30 | G=4.40 5.50 6.60 | A=0.110 0.220 0.330 | "
    "M = 11 22 33 44 | Val=1 | ACC=0.999 ACCU=1 | YAWREL=7.70 "
    "| ALTm=8.80 VZ=9.90 TGT=10.10 MODE=2 AV=1 TOF=11.11 TOK=0 TERR=12 TKO=3 KI=1 LAND=4"
    " MAGOK=0 BAROOK=1 IMUOK=1 BALT=13.13 DT=4.20 CMDAGE=14 HBAGE=15 FAULT=0"
    " BATV=3.95 BCOMP=1.063"
    " BFILT=16.16 BINNOV=0.17 BACC=18 BREJ=19 BCAL=1 BHLT=1 BSTD=20.20 VACC=0.21"
    " BDT=0.020 VACCR=0.220 ABIAS=-0.031 VZAO=0.240 ZINE=25.25 VZTGT=0.26"
    " KILL=0 TKOP=2 LSC=3 LDT=4001 LMX=4002 DLM=27"
    " IAGE=28 MAGE=29 BAGE=30 HDEG=1"
    " MSAT=1 MRPY=101 MHR=310 BTHR=1300"
    " AIRB=0 CAND=1 AZCORR=0.345 BCREJ=36 BREACQ=1 BSEQ=3700 BFI=1"
    " ALTSRC=2"
    " BATRAW=1500 BATMV=1190 BATRATIO=3.20 BATVRAW=3.808 BATVALID=1 BATCALI=1 BATAGE=95"
    " TKOACT=1 TKOTGT=0.80 ZSP=0.42 VZTGT=0.38 TKOBASE=1300 TKOCORR=-45"
    " TKOI=-120 TKOGND=0.02 TKOLIFT=1 TKOTILT=3.4 TKOEL=1.7 TKOAB=0 ALTSAT=0"
    " ARMREJ=11 ARMRSEQ=4"
    " TKOREJ=2 TKORSEQ=1"
    " TOFV=0.430 TOFINN=-0.400 TOFSURF=0.400 TOFST=2 TOFCOR=0 TOFGR=0.030"
    " TOFA=7 TOFR=93 FLOORZ=0.00 LANDZ=0.40 LANDV=1 TOFAGE=31"
    " TOFEN=1"
    " TOFCORRZ=0.0120 TOFCORRVZ=0.0450 BAROCORRZ=0.0031 BAROCORRVZ=0.0090"
    " BIASRES=-0.0210 BIASADP=1234"
    " HOVLK=1 HOVLV=3.87 HOVLD=1085"
)

m = mod.STATUS_RE.match(line)
assert m is not None, "STATUS_RE KHONG match dong STATUS cua firmware!"
g = m.groups()

fails = []


def check(name, got, want):
    if got != want:
        fails.append(f"  {name}: got={got!r} want={want!r}")


# --- Cac index dung trong _handle_line() ---
check("armed g[0]", g[0], "1")
check("thr g[1]", g[1], "1234")
check("batv g[31]", g[31], "3.95")
check("bcomp g[32]", g[32], "1.063")
# bfilt..vacc = g[33:41]
check("bfilt g[33]", g[33], "16.16")
check("bhlt g[38]", g[38], "1")
check("vacc g[40]", g[40], "0.21")
# bdt,vaccr,abias,vzao,zine,vztgt = g[41:47]
check("bdt g[41]", g[41], "0.020")
check("abias g[43]", g[43], "-0.031")
check("zine g[45]", g[45], "25.25")
# kill..dlm = g[47:53]
check("kill g[47]", g[47], "0")
check("tkop g[48]", g[48], "2")
check("lsc g[49]", g[49], "3")
# iage..hdeg = g[53:57]
check("iage g[53]", g[53], "28")
check("hdeg g[56]", g[56], "1")
# msat..bthr = g[57:63]
check("msat g[57]", g[57], "1")
check("bthr g[62]", g[62], "1300")
# airb,cand,azcorr,bcrej,breacq,bseq,bfi = g[63:70]   <-- MOI
check("airb g[63]", g[63], "0")
check("cand g[64]", g[64], "1")
check("azcorr g[65]", g[65], "0.345")
check("bcrej g[66]", g[66], "36")
check("breacq g[67]", g[67], "1")
check("bseq g[68]", g[68], "3700")
check("bfi g[69]", g[69], "1")
# batraw..batage = g[70:77]   <-- MOI
# altsrc = g[70]  <-- MOI: nguon dang giu Z (0=NONE 1=TOF 2=BARO 3=TOF+BARO)
check("altsrc g[70]", g[70], "2")
# batraw..batage DICH +1 vi ALTSRC chen vao TRUOC chung
check("batraw g[71]", g[71], "1500")
check("batmv g[72]", g[72], "1190")
check("batratio g[73]", g[73], "3.20")
check("batvraw g[74]", g[74], "3.808")
check("batvalid g[75]", g[75], "1")
check("batcali g[76]", g[76], "1")
check("batage g[77]", g[77], "95")
# tkoact..altsat = g[77:90]   <-- takeoff PID + slew
check("tkoact g[78]", g[78], "1")
check("tkotgt g[79]", g[79], "0.80")
check("zsp g[80]", g[80], "0.42")
check("vztgt g[81]", g[81], "0.38")
check("tkobase g[82]", g[82], "1300")
check("tkocorr g[83]", g[83], "-45")
check("tkoi g[84]", g[84], "-120")
check("tkognd g[85]", g[85], "0.02")
check("tkolift g[86]", g[86], "1")
check("tkotilt g[87]", g[87], "3.4")
check("tkoel g[88]", g[88], "1.7")
check("tkoab g[89]", g[89], "0")
check("altsat g[90]", g[90], "0")
# armrej,armrseq = g[90:92]   <-- MOI (ly do tu choi ARM)
check("armrej g[91]", g[91], "11")
check("armrseq g[92]", g[92], "4")
# tofv..landv = g[92:103]   <-- MOI (ToF surface-gated)
check("tofv g[95]", g[95], "0.430")
check("tofinn g[96]", g[96], "-0.400")
check("tofsurf g[97]", g[97], "0.400")
check("tofst g[98]", g[98], "2")
check("tofcor g[99]", g[99], "0")
check("tofgr g[100]", g[100], "0.030")
check("tofa g[101]", g[101], "7")
check("tofr g[102]", g[102], "93")
check("floorz g[103]", g[103], "0.00")
check("landz g[104]", g[104], "0.40")
check("landv g[105]", g[105], "1")
check("tofage g[106]", g[106], "31")
check("tkorej g[93]", g[93], "2")
check("tkorseq g[94]", g[94], "1")
# Moi ma surface PHAI co ten
for code in ("0", "1", "2"):
    if code not in mod.TOF_SURFACE_NAMES:
        fails.append(f"  TOF_SURFACE_NAMES thieu ma {code}")
# Moi ma trong enum PHAI co text huong dan — thieu la nguoi dung thay "ma N" vo nghia
for code in [str(i) for i in range(1, 16)]:
    if code not in mod.ARM_REJECT_NAMES or not mod.ARM_REJECT_NAMES[code]:
        fails.append(f"  ARM_REJECT_NAMES thieu/rong ma {code}")

# --- Tuong thich nguoc: firmware CU (khong co cac duoi moi) van phai parse ---
old_line = line.split(" AIRB=")[0]
m_old = mod.STATUS_RE.match(old_line)
assert m_old is not None, "STATUS_RE khong con tuong thich voi firmware CU!"
g_old = m_old.groups()
check("old: bthr g[62]", g_old[62], "1300")
check("old: airb g[63] phai None", g_old[63], None)
check("old: batraw g[71] phai None", g_old[71], None)
check("old: tkoact g[78] phai None", g_old[78], None)

# --- Firmware TRUOC refactor takeoff (co battery debug, KHONG co duoi takeoff) ---
mid_line = line.split(" TKOACT=")[0]
m_mid = mod.STATUS_RE.match(mid_line)
assert m_mid is not None, "STATUS_RE khong parse duoc firmware truoc refactor takeoff!"
g_mid = m_mid.groups()
check("mid: batage g[77]", g_mid[77], "95")
check("mid: tkoact g[78] phai None", g_mid[78], None)
check("mid: altsat g[90] phai None", g_mid[90], None)

# --- Firmware CO duoi takeoff nhung CHUA co ARMREJ ---
tko_line = line.split(" ARMREJ=")[0]
m_tko = mod.STATUS_RE.match(tko_line)
assert m_tko is not None, "STATUS_RE khong parse duoc firmware truoc khi them ARMREJ!"
g_tko = m_tko.groups()
check("tko: altsat g[90]", g_tko[90], "0")
check("tko: armrej g[91] phai None", g_tko[91], None)

# --- Kiem tra so group tong (bat viec them group ma quen cap nhat index) ---
# Firmware CO ARMREJ nhung CHUA co duoi ToF
tof_line = line.split(" TOFV=")[0]
m_tof = mod.STATUS_RE.match(tof_line)
assert m_tof is not None, "STATUS_RE khong parse duoc firmware truoc khi them ToF!"
check("pre-tof: armrseq g[92]", m_tof.groups()[92], "4")
check("pre-tof: tkorej g[93]", m_tof.groups()[93], "2")
check("pre-tof: tkorseq g[94]", m_tof.groups()[94], "1")
check("pre-tof: tofv g[95] phai None", m_tof.groups()[95], None)

# --- Firmware CO ARMREJ nhung CHUA co TKOREJ (ban truoc khi them) ---
# Quan trong: TKOREJ nam GIUA ARMREJ va duoi ToF, nen phai chac chan no
# optional that su -- neu khong, moi GUI cu se ngung parse duoc STATUS moi
# VA GUI moi ngung parse duoc firmware cu.
arm_only = line.split(" TKOREJ=")[0]
m_arm = mod.STATUS_RE.match(arm_only)
assert m_arm is not None, "STATUS_RE khong parse duoc firmware truoc khi them TKOREJ!"
check("arm-only: armrseq g[92]", m_arm.groups()[92], "4")
check("arm-only: tkorej g[93] phai None", m_arm.groups()[93], None)

# tofen = g[107] <-- MOI, them o CUOI regex nen KHONG dich chi so nao
check("tofen g[107]", g[107], "1")
# Firmware baro-only (TOFEN=0) phai parse duoc va cho dung gia tri
m_off = mod.STATUS_RE.match(line.replace(" TOFEN=1", " TOFEN=0"))
check("tofen=0 parse duoc", m_off is not None, True)
if m_off is not None:
    check("tofen=0 g[107]", m_off.groups()[107], "0")
    check("tofen=0 khong pha tofage", m_off.groups()[106], "31")
# Firmware CU (khong co TOFEN) van parse duoc, g[107] = None
m_no = mod.STATUS_RE.match(line.split(" TOFEN=")[0])
check("firmware cu khong co TOFEN van parse", m_no is not None, True)
if m_no is not None:
    check("firmware cu: tofen None", m_no.groups()[107], None)
    check("firmware cu: tofage van dung", m_no.groups()[106], "31")
# g[108:114] <-- MOI: luong sua thuc te + hoc bias tu residual (them o CUOI)
check("tofcorrz g[108]",  g[108],  "0.0120")
check("tofcorrvz g[109]", g[109],  "0.0450")
check("barocorrz g[110]", g[110],  "0.0031")
check("barocorrvz g[111]",g[111],  "0.0090")
check("biasres g[112]",   g[112],  "-0.0210")
check("biasadp g[113]",   g[113],  "1234")
# Firmware cu (khong co duoi nay) van phai parse va KHONG pha field cu
m_old2 = mod.STATUS_RE.match(line.split(" TOFCORRZ=")[0])
check("firmware cu parse duoc", m_old2 is not None, True)
if m_old2 is not None:
    check("firmware cu: tofcorrz None", m_old2.groups()[108], None)
    check("firmware cu: tofen van dung", m_old2.groups()[107], "1")
    check("firmware cu: tofage van dung", m_old2.groups()[106], "31")
# g[114:117] <-- MOI: latch ga hover theo pin (them o CUOI, khong dich gi)
check("hovlk g[114]", g[114], "1")
check("hovlv g[115]", g[115], "3.87")
check("hovld g[116]", g[116], "1085")
# Firmware cu / HOVER_LATCH_ENABLED=0 -> khong co duoi nay, van phai parse
m_old3 = mod.STATUS_RE.match(line.split(" HOVLK=")[0])
check("khong co HOVLK van parse", m_old3 is not None, True)
if m_old3 is not None:
    check("khong co HOVLK: g[114] None", m_old3.groups()[114], None)
    # Bang chung nhom moi KHONG dich chi so nao phia truoc:
    check("khong co HOVLK: biasadp van dung", m_old3.groups()[113], "1234")
    check("khong co HOVLK: tofen van dung", m_old3.groups()[107], "1")
    check("khong co HOVLK: tofage van dung", m_old3.groups()[106], "31")
# HOVLK=0 (chua ARM lan nao) phai parse va cho dung gia tri
m_nolatch = mod.STATUS_RE.match(line.replace(" HOVLK=1", " HOVLK=0"))
check("hovlk=0 parse duoc", m_nolatch is not None, True)
if m_nolatch is not None:
    check("hovlk=0 g[114]", m_nolatch.groups()[114], "0")
check("tong so group", len(g), 117)

# --- TELEMETRY_LEVEL=1 (MINIMAL): dong cat NGAY SAU HOVLD ---------------------
# Phase 2 refactor gate phan duoi HOVLD sau #if TELEMETRY_LEVEL >= 2. Test nay
# la BANG CHUNG cho tuyen bo "MINIMAL khong lam mat gi tren GUI": neu ranh gioi
# cat sai (cat nham vao giua mot cum ma GUI dang doc), so group co gia tri se
# TUT XUONG va check duoi day fail.
#
# Dung split(" TOFZ=") vi TOFZ la field DAU TIEN cua khoi FULL trong
# src/telemetry_format.c (ngay sau HOVLK/HOVLV/HOVLD).
minimal_line = line.split(" TOFZ=")[0]
m_min = mod.STATUS_RE.match(minimal_line)
check("MINIMAL parse duoc", m_min is not None, True)
if m_min is not None:
    g_min = m_min.groups()
    # Toan bo 117 group van co mat -> khong mat gi so voi FULL.
    check("MINIMAL: van du 117 group", len(g_min), 117)
    check("MINIMAL: hovlk g[114]", g_min[114], "1")
    check("MINIMAL: hovlv g[115]", g_min[115], "3.87")
    check("MINIMAL: hovld g[116]", g_min[116], "1085")
    # Khong mot group nao bi None hoa so voi FULL: chung minh phan bi cat
    # KHONG nam trong vung STATUS_RE doc.
    lost = [i for i in range(117) if g[i] is not None and g_min[i] is None]
    check("MINIMAL: khong group nao bi mat", lost, [])

if fails:
    print("FAIL:")
    print("\n".join(fails))
    sys.exit(1)
print(f"PASS: STATUS_RE parse dung {len(g)} group, ca dinh dang MOI lan CU")
