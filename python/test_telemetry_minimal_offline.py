"""TELEMETRY_LEVEL = 1 (MINIMAL): dong gon 6 truong.

MUC DICH. Telemetry di CHUNG duong Wi-Fi voi MJPEG. Dong FULL ~1010 byte o
20Hz = ~19.7 KB/s; dong gon 47 byte = ~0.9 KB/s. Muc nay tra ~18.8 KB/s lai
cho video.

BAY DA KIEM: hai dinh dang phai phan biet duoc. Toi tung DOAN rang dong FULL
bat dau y het dong gon nen re.match() se nuot no -- DO LA SAI, FULL co dau '|'
ngan cach ("ARM=1 THR=1107 | R=..."). Test kiem bang so thay vi tin tri nho.
fullmatch() van la lua chon dung, nhung vi ly do khac: no tu choi moi phan du
phia sau, nen them truong ma quen sua regex se bao ngay.

Bay thu hai: khi doi sang MINIMAL, cac o GUI khac (ToF, pin, terrain) khong duoc
xoa trang. Firmware khong gui KHONG co nghia gia tri do da thanh 0.
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


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def read(rel):
    return io.open(os.path.join(ROOT, rel), encoding="utf-8",
                   errors="replace").read()


GUI = read("tools/uav_udp_console.py")
FMT = read("src/telemetry_format.c")
CFG = read("main/app_config.h")

# Lay COMPACT_RE THAT tu GUI thay vi chep lai -- chep lai thi test kiem chinh
# ban sao cua no, khong kiem cai dang chay.
m = re.search(r"COMPACT_RE = re\.compile\(\s*\n\s*(r\"[^\n]*\")\s*\n\s*\)", GUI)
COMPACT_RE = re.compile(eval(m.group(1))) if m else None

GON = "ARM=1 THR=1107 R=1.48 P=0.35 ALTm=1.03 TGT=1.00"
FULL = ("ARM=1 THR=1107 | R=1.48 P=0.35 Y=2.98 | G=0.98 6.72 4.17 | "
        "A=-0.061 0.021 1.017 | M = 1031 1210 966 1219 | Val=1 | "
        "ACC=1.019 ACCU=1 | YAWREL=-90.69 | ALTm=1.03 VZ=0.05 TGT=1.00 "
        "MODE=5 AV=1 TOF=0.48 TOK=1 TERR=0 TKO=0 KI=1 LAND=0")

print("== (1) Firmware co nhanh MINIMAL rieng ==")
check("telemetry_format.c co '#elif TELEMETRY_LEVEL == 1'",
      "#elif TELEMETRY_LEVEL == 1" in FMT,
      "muc 1 van dang dung chung format voi muc 2")
check("khong con '#if TELEMETRY_LEVEL >= 2' (dieu kien chet)",
      "TELEMETRY_LEVEL >= 2" not in FMT,
      "nhanh FULL gio chi chay o muc 2 -> guard do luon dung")

print()
print("== (2) Dung SAU truong, khong hon khong kem ==")
i = FMT.index("#elif TELEMETRY_LEVEL == 1")
j = FMT.index("#else", i)
blk = FMT[i:j]
for f in ("ARM=%d", "THR=%d", "R=%.2f", "P=%.2f", "ALTm=%.2f", "TGT=%.2f"):
    check("co truong %s" % f.split("=")[0], f in blk)
# Khong duoc lot them truong nao khac vao dong gon.
thua = [t for t in ("VZ=", "MODE=", "TOF=", "YAWREL=", "BATV=", "TOFF=")
        if t in blk]
check("khong co truong thua", not thua, "lot them: %s" % thua)

print()
print("== (3) GUI co COMPACT_RE va no khop dong gon ==")
check("tim thay COMPACT_RE trong GUI", COMPACT_RE is not None)
if COMPACT_RE:
    g = COMPACT_RE.fullmatch(GON)
    check("khop dong gon", g is not None)
    if g:
        check("lay dung 6 gia tri",
              g.groups() == ("1", "1107", "1.48", "0.35", "1.03", "1.00"),
              "duoc %s" % (g.groups(),))

print()
print("== (4) Dong FULL KHONG duoc lot vao parser gon ==")
# ⚠ TOI DA DOAN SAI o ban dau: tuong dong FULL bat dau y het dong gon nen
# re.match() se nuot no. Do lai thi KHONG phai -- FULL co dau '|' ngan cach:
#     FULL : "ARM=1 THR=1107 | R=1.48 ..."
#     GON  : "ARM=1 THR=1107 R=1.48 ..."
# Nen ngay ca match() cung khong khop. Kiem tra lai bang so, khong bang tri nho:
check("hai dinh dang khac nhau ngay o ky tu ngan cach",
      FULL.startswith("ARM=1 THR=1107 |") and GON.startswith("ARM=1 THR=1107 R"))
if COMPACT_RE:
    check("fullmatch dong FULL -> None", COMPACT_RE.fullmatch(FULL) is None,
          "dong FULL bi parser gon nuot -> GUI tut ve 6 truong im lang")
    # fullmatch van la lua chon dung, nhung LY DO khac cai toi tuong: no tu
    # choi MOI phan du phia sau. Neu sau nay ai them truong vao dong gon ma
    # quen sua regex, fullmatch bao ngay; match() thi lang le bo qua phan moi.
    check("fullmatch tu choi phan du phia sau",
          COMPACT_RE.fullmatch(GON + " EXTRA=1") is None)
check("GUI goi fullmatch, khong phai match",
      "COMPACT_RE.fullmatch(" in GUI)

print()
print("== (5) GUI thu dong GON TRUOC dong FULL, va thoat han ==")
i_c = GUI.find("COMPACT_RE.fullmatch(")
i_s = GUI.find("m = STATUS_RE.match(line)")
check("kiem dong gon truoc STATUS_RE", i_c != -1 and i_s != -1 and i_c < i_s,
      "compact=%d status=%d" % (i_c, i_s))
seg = GUI[i_c:i_s]
check("khop xong thi return, khong chay tiep khoi FULL", "return" in seg)

print()
print("== (6) Muc MINIMAL khong xoa trang cac o khac ==")
k = GUI.find("def _handle_compact_status")
body = GUI[k:k + 1600] if k != -1 else ""
check("co _handle_compact_status", bool(body))
check("bao ro dang o che do MINIMAL tren GUI", "MINIMAL" in body,
      "khong bao thi nguoi dung tuong GUI hong")
check("boc try/except quanh phan doi so",
      "except" in body,
      "mot dong loi khong duoc lam chet vong nhan")

print()
print("== (7) Bang thong: MINIMAL phai nho hon FULL it nhat 10 lan ==")
# Chuoi FULL o tren la ban CAT NGAN cho de doc (228 byte). Do dai THAT lay tu
# so da do trong repo: src/main.c ghi "Dong STATUS thuc te ~1010 byte (113
# field - do bang chinh chuoi mau trong test_status_parse_offline.py)".
FULL_REAL_BYTES = 1010
check("so 1010 van con trong main.c (neu doi, sua ca day)",
      "~1010 byte" in read("src/main.c"))
for ten, n in (("FULL (that)", FULL_REAL_BYTES), ("GON", len(GON))):
    print("     %-11s %4d byte -> %5.1f KB/s @20Hz" % (ten, n, n * 20 / 1024.0))
ratio = FULL_REAL_BYTES / float(len(GON))
print("     -> gon %.1f lan, tra lai %.1f KB/s cho MJPEG"
      % (ratio, (FULL_REAL_BYTES - len(GON)) * 20 / 1024.0))
check("gon nho hon full >= 10 lan", ratio >= 10.0, "chi %.1f lan" % ratio)

print()
print("== (8) Tai lieu khong con noi sai ve muc 1 ==")
check("app_config.h khong con khang dinh muc 1 giu nguyen GUI",
      "KHÔNG làm mất bất kỳ thứ gì GUI đang hiển thị" not in CFG,
      "mo ta cu da sai tu khi muc 1 thanh dong 6 truong")
check("co canh bao MINIMAL lam mat GUI", "MINIMAL LAM MAT GAN HET GUI" in CFG)

print()
if FAILED:
    print("KET QUA: %d FAIL -- %s" % (len(FAILED), FAILED[0]))
    sys.exit(1)
print("KET QUA: TAT CA PASS")
