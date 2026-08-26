# -*- coding: utf-8 -*-
"""
test_tof_bringup_offline.py -- kiem tra logic bring-up VL53L0X (MOT sensor).

VI SAO TEST OFFLINE BANG PYTHON: repo khong co trinh bien dich C cho may host,
nen moi test la BAN CHEP thuat toan C sang Python (giong cac test_*_offline khac).

⚠ DA VIET LAI cho cau hinh MOT sensor. Ban truoc mo hinh hoa
tof_driver_init_dual() (2 chip, XSHUT tuan tu, doi dia chi bat buoc) -- duong do
DA BI XOA khoi firmware vi con ToF thu hai chua bao gio duoc fuse vao control
loop nhung bat driver phai mang nhung rang buoc chi co nghia khi co 2 chip.
Test cu VAN PASS sau khi xoa (no la mo hinh Python thuan, khong coupled voi
source) -- tuc no dang bao ve code KHONG CON TON TAI. Do la kieu test te nhat:
xanh nhung vo nghia.

Bai test nay bao ve:
  (A) HANG SO + cach lai XSHUT doc THANG tu tof_driver.c (rang buoc DATASHEET).
  (B) May trang thai DIA CHI cho 1 sensor.
  (C) Cac nhanh loi + bat bien "khong con duong dual nao trong code".
"""

import io
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
DRIVER_C = os.path.join(HERE, "..", "components", "flight_core", "src",
                        "drivers", "tof_driver.c")
DRIVER_H = os.path.join(HERE, "..", "components", "flight_core", "include",
                        "flight_core", "drivers", "tof_driver.h")

ROOT = os.path.normpath(os.path.join(HERE, ".."))

DEFAULT_ADDR = 0x29

fails = []


def check(name, cond, detail=""):
    if cond:
        print("  PASS  " + name)
    else:
        print("  FAIL  " + name + ("  -- " + detail if detail else ""))
        fails.append(name)


src = io.open(DRIVER_C, encoding="utf-8").read()
hdr = io.open(DRIVER_H, encoding="utf-8").read()


def strip_comments(text):
    """Bo comment truoc khi tim dinh danh cu.

    Comment CO QUYEN nhac ten cu (giai thich vi sao duong dual bi xoa) -- do la
    lich su can giu. Chi CODE moi khong duoc con tham chieu toi chung.
    """
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    return re.sub(r"//[^\n]*", "", text)


code = strip_comments(src)
hdr_code = strip_comments(hdr)


# =============================================================================
# (A) Rang buoc datasheet + cach lai XSHUT
# =============================================================================
print("== (A) Rang buoc datasheet trong tof_driver.c ==")


# Hai nguon: hang so cua FACADE nam trong tof_driver.c, hang so DUNG CHUNG cho
# moi backend nam trong tof_backend.h. Tim ca hai -- ten da bo tien to VL53L0X_
# khi tach chip, vi chung khong con thuoc ve mot dong chip nao.
BACKEND_H = os.path.join(ROOT, "components", "flight_core", "src", "drivers",
                         "tof_backend.h")
backend_src = io.open(BACKEND_H, encoding="utf-8").read()


def define_int(name):
    for blob in (src, backend_src):
        m = re.search(r"^#define\s+" + name + r"\s+(\d+)", blob, re.M)
        if m:
            return int(m.group(1))
    return None


boot_ms = define_int("TOF_BOOT_DELAY_MS")
hold_ms = define_int("TOF_XSHUT_HOLD_MS")
probe_tries = define_int("TOF_PROBE_ATTEMPTS")

# vTaskDelay(pdMS_TO_TICKS(n)) chi bao dam (n-1) tick TRON VEN: tick dau tien bi
# cat cut theo pha. Voi tick 1000Hz -> thoi gian THAT toi thieu = (n-1) ms.
T_BOOT_MAX_MS = 1.2
check("TOF_BOOT_DELAY_MS ton tai", boot_ms is not None)
check("boot delay du cho t_boot=1.2ms ke ca khi tick bi cat cut",
      boot_ms is not None and (boot_ms - 1) >= T_BOOT_MAX_MS,
      "boot_ms=%s -> toi thieu that %.1fms" % (boot_ms, (boot_ms - 1) if boot_ms else -1))
check("gia tri cu (2ms) DA bi bat la vi pham", (2 - 1) < T_BOOT_MAX_MS)
check("TOF_XSHUT_HOLD_MS ton tai va >= boot delay",
      hold_ms is not None and boot_ms is not None and hold_ms >= boot_ms)
check("co thu lai khi probe (>=3 lan)", probe_tries is not None and probe_tries >= 3)

m = re.search(r"static esp_err_t configure_xshut_gpio\(.*?\n\}", src, re.S)
xshut_fn = m.group(0) if m else ""
check("configure_xshut_gpio() dung GPIO_MODE_OUTPUT_OD",
      "GPIO_MODE_OUTPUT_OD" in xshut_fn,
      "XSHUT la chan 2.8V khong level-shift -- push-pull 3.3V la ngoai spec")
check("configure_xshut_gpio() KHONG dung push-pull",
      not re.search(r"cfg\.mode\s*=\s*GPIO_MODE_OUTPUT\s*;", xshut_fn))
check("configure_xshut_gpio() chan gpio am truoc khi dich bit",
      re.search(r"if\s*\(\s*gpio_num\s*<\s*0\s*\)", xshut_fn) is not None,
      "1ULL << (so am) la undefined behaviour")
check("khong bat pull-up noi bo tren XSHUT", "GPIO_PULLUP_DISABLE" in xshut_fn)

check("co dung i2c_master_probe() de do bus", "i2c_master_probe(" in code)
check("co do CA dia chi mong muon LAN dia chi mac dinh",
      "TOF_DEFAULT_I2C_ADDR" in code and "wanted_addr" in code)

# Sau khi nha open-drain, phai XAC NHAN pull-up keo len that. Voi GPIO0 (chan
# strapping BOOT) day con la canh bao "lan reset sau se vao download mode".
check("co xac nhan XSHUT len HIGH sau khi nha",
      "xshut_check_released_high" in code)
check("co canh bao rieng cho GPIO0 (chan strapping BOOT)",
      "gpio_num == 0" in code or "xshut_gpio == 0" in code,
      "GPIO0 LOW luc reset -> bo vao DOWNLOAD MODE thay vi boot firmware")


# =============================================================================
# (B) May trang thai dia chi -- MOT sensor
# =============================================================================
print("\n== (B) May trang thai dia chi (1 sensor) ==")

ERR_OK = "ESP_OK"
ERR_NOT_FOUND = "ESP_ERR_NOT_FOUND"
ERR_INVALID_ARG = "ESP_ERR_INVALID_ARG"
ERR_FAIL = "ESP_FAIL"


class Chip(object):
    """VL53L0X mo phong.

    addr:          dia chi HIEN TAI, nam trong RAM chip.
    xshut_wired:   day XSHUT co that su noi toi MCU khong.
    powered:       con dien khong.
    addr_write_ok: gia lap truong hop ghi register 0x8A that bai.
    """

    def __init__(self, xshut_wired=True, powered=True, addr=DEFAULT_ADDR,
                 addr_write_ok=True):
        self.addr = addr
        self.xshut_wired = xshut_wired
        self.powered = powered
        self.addr_write_ok = addr_write_ok
        self.in_reset = False
        self.damaged = False       # bi day 3.3V vao XSHUT
        self.init_count = 0

    def xshut_drive_low(self):
        if not self.xshut_wired:
            return
        self.in_reset = True
        self.addr = DEFAULT_ADDR   # reset xoa dia chi trong RAM

    def xshut_release_highz(self):
        if not self.xshut_wired:
            return
        self.in_reset = False

    def xshut_drive_high_3v3(self):
        """Cach lai SAI. Ngoai spec 2.8V -> chip khong con tin duoc."""
        if not self.xshut_wired:
            return
        self.in_reset = False
        self.damaged = True

    def responds_at(self, addr):
        return (self.powered and not self.in_reset and not self.damaged
                and self.addr == addr)

    def write_addr_reg(self, new_addr):
        if not self.addr_write_ok:
            return ERR_FAIL
        self.addr = new_addr
        return ERR_OK


class Bus(object):
    def __init__(self, chips):
        self.chips = chips

    def probe(self, addr):
        return any(c.responds_at(addr) for c in self.chips)

    def chip_at(self, addr):
        for c in self.chips:
            if c.responds_at(addr):
                return c
        return None


def find_sensor_addr(bus, wanted_addr):
    for _ in range(probe_tries):
        if bus.probe(DEFAULT_ADDR):
            return DEFAULT_ADDR
        if wanted_addr != DEFAULT_ADDR and bus.probe(wanted_addr):
            return wanted_addr
    return None


def tof_driver_init(bus, chip, xshut_gpio, i2c_addr):
    """Ban chep tof_driver_init() single-sensor. Tra (err, final_addr, log)."""
    log = []
    if i2c_addr == 0 or i2c_addr >= 0x80:
        return ERR_INVALID_ARG, None, log

    # XSHUT la BEST-EFFORT: gpio < 0 = khong noi, VAN init duoc.
    xshut_ok = xshut_gpio is not None and xshut_gpio >= 0
    if xshut_ok:
        chip.xshut_drive_low()
        chip.xshut_release_highz()
    else:
        log.append("NO_XSHUT")

    found_at = find_sensor_addr(bus, i2c_addr)
    if found_at is None:
        log.append("NOT_FOUND")
        return ERR_NOT_FOUND, None, log

    final_addr = found_at
    if found_at != i2c_addr:
        # Doi dia chi hong KHONG phai loi cung: chi 1 sensor, khong ai tranh.
        if chip.write_addr_reg(i2c_addr) == ERR_OK:
            final_addr = i2c_addr
        else:
            log.append("ADDR_CHANGE_SOFT_FAIL")
    elif found_at != DEFAULT_ADDR:
        log.append("ALREADY_AT_TARGET")

    chip.init_count += 1
    return ERR_OK, final_addr, log


# ---- T1: dia chi mac dinh 0x29 (cau hinh THAT hien tai) ----
c = Chip(xshut_wired=True)
bus = Bus([c])
err, addr, log = tof_driver_init(bus, c, 0, 0x29)
check("T1 addr mac dinh 0x29 + XSHUT GPIO0 -> OK, khong doi dia chi",
      err == ERR_OK and addr == 0x29 and c.init_count == 1,
      "err=%s addr=%s log=%s" % (err, addr, log))

# ---- T2: XSHUT KHONG noi (gpio < 0) -- PHAI van init duoc ----
# Day la khac biet then chot so voi cau hinh 2 chip (noi XSHUT la DIEU KIEN CAN).
c = Chip(xshut_wired=False)
bus = Bus([c])
err, addr, log = tof_driver_init(bus, c, -1, 0x29)
check("T2 XSHUT khong noi (-1) -> VAN init duoc (module tu pull-up)",
      err == ERR_OK and addr == 0x29 and "NO_XSHUT" in log,
      "err=%s addr=%s log=%s" % (err, addr, log))

# ---- T3 (HOI QUY): reset mem, XSHUT khong noi, chip con giu 0x2A ----
# Dia chi nam trong RAM chip -> RESET ESP32 khong xoa. Driver phai tu lanh.
c = Chip(xshut_wired=False, addr=0x2A)
bus = Bus([c])
err, addr, log = tof_driver_init(bus, c, -1, 0x2A)
check("T3 reset mem + chip con giu 0x2A -> tim thay, KHONG doi lai dia chi",
      err == ERR_OK and addr == 0x2A and "ALREADY_AT_TARGET" in log,
      "err=%s addr=%s log=%s" % (err, addr, log))


def find_sensor_addr_OLD(bus, wanted_addr):
    """Ban CU chi biet 0x29 -- chung minh test co y nghia."""
    return DEFAULT_ADDR if bus.probe(DEFAULT_ADDR) else None


c_old = Chip(xshut_wired=False, addr=0x2A)
check("T3b ban CU that bai dung o tinh huong nay (test co y nghia)",
      find_sensor_addr_OLD(Bus([c_old]), 0x2A) is None)

# ---- T4: doi dia chi that bai -> o lai, VAN bay duoc (khong con loi cung) ----
c = Chip(xshut_wired=True, addr_write_ok=False)
bus = Bus([c])
err, addr, log = tof_driver_init(bus, c, 0, 0x2A)
check("T4 doi dia chi hong -> fallback ve 0x29, KHONG vut cam bien",
      err == ERR_OK and addr == DEFAULT_ADDR and "ADDR_CHANGE_SOFT_FAIL" in log,
      "err=%s addr=%s log=%s" % (err, addr, log))

# ---- T5: khong co chip nao tren bus ----
c = Chip(xshut_wired=True, powered=False)
bus = Bus([c])
err, addr, log = tof_driver_init(bus, c, 0, 0x29)
check("T5 khong co chip -> NOT_FOUND (loi ro rang, khong mo ho)",
      err == ERR_NOT_FOUND and "NOT_FOUND" in log, "err=%s" % err)

# ---- T6: cach lai SAI (push-pull 3.3V) phai lam chip chet ----
# Khong test code hien tai -- ghi lai VI SAO code phai la open-drain.
c = Chip(xshut_wired=True)
c.xshut_drive_low()
c.xshut_drive_high_3v3()
check("T6 day 3.3V vao XSHUT -> chip khong con tra loi (ngoai spec 2.8V)",
      not Bus([c]).probe(DEFAULT_ADDR))

c = Chip(xshut_wired=True)
c.xshut_drive_low()
c.xshut_release_highz()
check("T6b nha ve high-Z -> chip tra loi binh thuong",
      Bus([c]).probe(DEFAULT_ADDR))


# =============================================================================
# (C) Bat bien: duong DUAL da bi xoa HAN khoi code
# =============================================================================
print("\n== (C) Duong dual-sensor da bi xoa khoi CODE ==")

for ident in ("tof_driver_init_dual", "tof_driver_read_aux", "s_sensors",
              "slot_for_addr", "read_slot", "poll_one"):
    check("code KHONG con '%s'" % ident, ident not in code,
          "con trong code (khong phai comment)")

for ident in ("tof_driver_init_dual", "tof_driver_read_aux"):
    check("header KHONG con khai bao '%s'" % ident, ident not in hdr_code)

# API single-sensor phai co dung 4 tham so (bus, xshut_gpio, addr, scl_hz).
m = re.search(r"esp_err_t\s+tof_driver_init\s*\(([^)]*)\)", hdr_code, re.S)
check("header khai bao tof_driver_init(bus, xshut_gpio, addr, scl_hz)",
      m is not None and m.group(1).count(",") == 3,
      "tim thay: %s" % (m.group(1).replace("\n", " ") if m else "khong co"))

check("chi con MOT bien sensor (s_sensor, khong phai mang)",
      re.search(r"static\s+tof_sensor_state_t\s+s_sensor\s*;", code) is not None)


# =============================================================================
# (D) Tach driver theo DONG CHIP: facade khong duoc biet gi ve thanh ghi cua chip
# =============================================================================
# Muc dich cua viec tach la facade CHI lo XSHUT / do dia chi / watchdog /
# staleness, con moi thanh ghi cua chip nam trong backend. Neu mot hang so cua
# chip lot nguoc vao facade thi viec tach da hong ve BAN CHAT du van build duoc
# -- va lan sau them chip thu ba se lai phai sua facade. Bat o day.
print("\n== (D) Tach backend theo dong chip ==")

DRV_DIR = os.path.join(ROOT, "components", "flight_core", "src", "drivers")
be_hdr  = strip_comments(io.open(os.path.join(DRV_DIR, "tof_backend.h"), encoding="utf-8").read())
l0x     = strip_comments(io.open(os.path.join(DRV_DIR, "vl53l0x_driver.c"), encoding="utf-8").read())
l1x     = strip_comments(io.open(os.path.join(DRV_DIR, "vl53l1x_driver.c"), encoding="utf-8").read())
board_h = io.open(os.path.join(ROOT, "main", "board_config.h"), encoding="utf-8").read()

# D1: facade KHONG duoc chua thanh ghi rieng cua chip nao.
# 0x29 (dia chi I2C) va 0xC0/0x010F (doc ID de nhan dien chip la) la NGOAI LE co
# y: ca hai dong VL53 dung chung 0x29, va facade PHAI doc duoc ID cua ca hai de
# noi duoc "ban cam nham chip nao".
for reg in ("SYSRANGE_START", "SYSTEM_SEQUENCE_CONFIG", "GLOBAL_CONFIG_SPAD",
            "VL53L0X_TUNING", "stop_variable", "L1X_DEFAULT_CONFIG",
            "SD_CONFIG_WOI", "RANGE_CONFIG_VCSEL"):
    check("facade KHONG chua thanh ghi rieng cua chip '%s'" % reg, reg not in code)

# D2: moi backend phai hien thuc DU 5 ham cua giao dien.
for suffix, body, name in (("l0x", l0x, "VL53L0X"), ("l1x", l1x, "VL53L1X")):
    for fn in ("set_addr", "probe", "setup", "poll", "restart"):
        sym = "tof_backend_%s_%s" % (fn, suffix)
        check("backend %s hien thuc %s()" % (name, sym), sym in body)
        check("giao dien khai bao %s()" % sym, sym in be_hdr)

# D3: hai backend KHONG duoc chua thanh ghi cua nhau -- day chinh la loi ma
# viec tach sinh ra de chan (mot VL53L1X cam vao driver L0X van ACK nhung doc
# ra rac, xem identify_foreign_device()).
check("backend L0X khong dung index 16-bit cua L1X", "0x010F" not in l0x)
check("backend L1X khong dung thanh ghi 8-bit cua L0X",
      "VL53L0X_REG_" not in l1x)

# D4: moi backend phai TU GATE theo BOARD_TOF_CHIP. Khong gate thi ca hai cung
# sinh code va se dung ten ham trung nhau o tang static -> hoac loi link, hoac
# (te hon) chip sai duoc init im lang.
check("backend L0X tu gate theo BOARD_TOF_CHIP",
      "BOARD_TOF_CHIP == TOF_CHIP_VL53L0X" in l0x)
check("backend L1X tu gate theo BOARD_TOF_CHIP",
      "BOARD_TOF_CHIP == TOF_CHIP_VL53L1X" in l1x)

# D5: board_config.h phai TU DINH NGHIA ma dong chip. Neu no phai include header
# noi bo cua driver moi doc duoc thi mo ta phan cung da phu thuoc nguoc vao
# driver -- dung chieu phu thuoc.
check("board_config.h tu dinh nghia TOF_CHIP_VL53L0X",
      re.search(r"#define\s+TOF_CHIP_VL53L0X\s+0", board_h) is not None)
check("board_config.h tu dinh nghia TOF_CHIP_VL53L1X",
      re.search(r"#define\s+TOF_CHIP_VL53L1X\s+1", board_h) is not None)

m = re.search(r"#define\s+BOARD_TOF_CHIP\s+(TOF_CHIP_VL53L\dX)", board_h)
check("board_config.h chon mot dong chip hop le", m is not None,
      "khong tim thay #define BOARD_TOF_CHIP")
if m:
    print("  (BOARD_TOF_CHIP = %s)" % m.group(1))

# D6: RANG BUOC THAT khi dung L1X -- nhip mau phai nhanh hon nguong "mau da cu".
# Driver da co _Static_assert cho viec nay, nhung no chi ban khi BUILD dung chip
# do; kiem o day de doi cau hinh xong la biet ngay, khong phai cho build.
if m and m.group(1) == "TOF_CHIP_VL53L1X":
    def cfg(name, default=None):
        mm = re.search(r"#define\s+%s\s+(\d+)" % name, board_h)
        return int(mm.group(1)) if mm else default
    tb   = cfg("BOARD_TOF_L1X_TIMING_BUDGET_MS")
    im   = cfg("BOARD_TOF_L1X_INTER_MEASUREMENT_MS", tb)
    mode = cfg("BOARD_TOF_L1X_DISTANCE_MODE")
    stale_m = re.search(r"#define\s+TOF_STALE_TIMEOUT_MS\s+(\d+)", hdr)
    stale = int(stale_m.group(1)) if stale_m else 200

    # ST ULD: IM phai LON HON TB, khong phai >=. Dat bang nhau lam chip bo mau
    # khong deu -- trieu chung la "ToF luc duoc luc khong", rat de di tim nham
    # o day/nguon.
    check("L1X: inter-measurement > timing budget (ST ULD)",
          im is not None and tb is not None and im > tb,
          "IM=%s TB=%s" % (im, tb))
    check("L1X: timing budget co trong bang cua ST",
          tb in (20, 33, 50, 100, 200, 500) or (tb == 15 and mode == 1),
          "TB=%sms khong co trong bang (15 chi o SHORT/20/33/50/100/200/500)" % tb)
    check("L1X: nhip mau du nhanh so voi TOF_STALE_TIMEOUT_MS",
          im is not None and im * 2 <= stale,
          "IM=%sms x2 > stale=%sms -> moi mau toi khi mau truoc da cu" % (im, stale))
    check("L1X: distance mode la 1 (SHORT) hoac 2 (LONG)", mode in (1, 2),
          "mode=%s" % mode)

    # D7: HAI LOI CU THE da tung co trong ban dau cua backend L1X. Ca hai deu
    # BUILD DUOC va chi bieu hien luc chay, nen phai chan bang test:
    #
    #  (a) Osc calibration nam o 0x00DE, KHONG phai 0x0022. Doc nham -> he so
    #      quy doi sai -> inter-measurement sai (thuong qua NGAN) -> chip bo mau
    #      im lang. Trieu chung giong het loi day.
    #  (b) Ket qua phai doc CA KHOI 17 byte tu 0x0089 trong MOT transaction, roi
    #      lay range o offset [13..14]. Doc status va range thanh HAI lan mo ra
    #      cua so de chip cap nhat giua chung -> ghep status cua mau nay voi
    #      khoang cach cua mau kia.
    # Kiem CHINH XAC: hang so ten OSC_CALIBRATE phai = 0x00DE. Khong grep suong
    # "0x0022 khong xuat hien" -- so do CO trong bang timing budget cua LONG mode
    # ({20, 0x001E, 0x0022}) mot cach hoan toan hop le.
    m_osc = re.search(r"#define\s+L1X_RESULT_OSC_CALIBRATE_VAL\s+(0x[0-9A-Fa-f]+)", l1x)
    check("L1X dinh nghia OSC_CALIBRATE_VAL = 0x00DE",
          m_osc is not None and int(m_osc.group(1), 16) == 0x00DE,
          "doc duoc %s -- 0x0022 la thanh ghi KHAC, doc nham lam IM sai va chip bo mau"
          % (m_osc.group(1) if m_osc else "khong co"))
    check("l1x_set_inter_measurement() doc dung hang so do",
          "L1X_RESULT_OSC_CALIBRATE_VAL" in l1x)
    check("L1X doc ket qua thanh MOT khoi 17 byte (atomic)",
          re.search(r"result\[17\]", l1x) is not None and
          "result[13]" in l1x and "result[14]" in l1x,
          "doc status/range rieng le -> co the ghep nham hai mau khac nhau")

    # D8: poll() TUYET DOI khong duoc chan. No chay trong sensor_hub, tren duong
    # nhip cam bien cua vong bay. Vong cho co chan CHI duoc phep o init.
    m_poll = re.search(r"esp_err_t\s+tof_backend_poll_l1x\s*\([^)]*\)\s*\{(.*?)\n\}",
                       l1x, re.S)
    check("tim thay than ham tof_backend_poll_l1x()", m_poll is not None)
    if m_poll:
        body = m_poll.group(1)
        check("poll() KHONG chan (khong vTaskDelay / khong vong cho)",
              "vTaskDelay" not in body and "wait_ready_init" not in body,
              "poll() chay trong sensor_hub -- chan o day la treo vong bay")

    # D9: cuc tinh ngat phai DOC TU CHIP, khong duoc gia dinh. Gia dinh sai thi
    # vong cho mau khong bao gio thoat, hoac thoat moi vong va doc lai cung mot
    # mau -- ca hai deu khong bao loi.
    check("L1X doc cuc tinh ngat tu GPIO_HV_MUX_CTRL",
          "l1x_read_interrupt_polarity" in l1x and "GPIO_HV_MUX_CTRL" in l1x)

    # D10: I/O rail cua I2C phia sensor phai duoc dat TUONG MINH. Sai rail ->
    # muc logic khong dat nguong -> NACK ngau nhien, giong het loi day.
    check("board_config.h dat tuong minh BOARD_TOF_L1X_IO_2V8",
          re.search(r"#define\s+BOARD_TOF_L1X_IO_2V8\s+[01]", board_h) is not None)
    check("bang cau hinh L1X dung co IO_2V8 cho byte 0x2E",
          "BOARD_TOF_L1X_IO_2V8" in l1x)

    # D11: bang cau hinh ULD phai phu DUNG dai 0x2D..0x87. Thieu/thua mot byte
    # se lam lech TOAN BO cac byte sau no -- driver van build, chip van ACK, va
    # so do ra rac. Driver da co _Static_assert; kiem o day de bat som hon.
    check("L1X co _Static_assert kich thuoc bang cau hinh",
          "L1X_CONFIG_END_REG - L1X_CONFIG_START_REG" in l1x)

    # -------------------------------------------------------------------------
    # RANG BUOC GIUA HAI FILE: board_config.h (dong chip + mode) vs
    # alt_estimator.h (tran bay + gate tilt).
    # -------------------------------------------------------------------------
    # Driver da co _Static_assert cho ca ba, nhung chung chi ban khi BUILD dung
    # chip do. Kiem o day de doi cau hinh xong la biet ngay, khong phai cho
    # build -- va de thay CON SO chu khong chi thay pass/fail.
    alt_h = io.open(os.path.join(ROOT, "components", "flight_core", "include",
                                 "flight_core", "alt_estimator.h"), encoding="utf-8").read()

    def altnum(name):
        mm = re.search(r"#define\s+%s\s+([\d.]+)f" % name, alt_h)
        return float(mm.group(1)) if mm else None

    ceiling  = altnum("ALT_EST_MAX_FLIGHT_Z_M")
    gate_max = altnum("ALT_EST_TOF_MAX_RANGE_M")
    tilt_cos = altnum("ALT_EST_TOF_TILT_MIN_COS")

    # Tam TIN CAY (khong phai tam danh nghia). SHORT on dinh ke ca duoi anh sang
    # nen manh; LONG chi dat ~3.6m tren be mat phan xa tot trong anh sang yeu.
    trusted = 1.30 if mode == 1 else 3.60
    mode_name = "SHORT" if mode == 1 else "LONG"

    check("tran bay nam trong tam tin cay cua distance mode",
          ceiling is not None and ceiling <= trusted,
          "tran %.2fm > %.2fm cua mode %s -> mat mau ToF giua chung chuyen bay"
          % (ceiling or -1, trusted, mode_name))

    # O dung tran ma nghieng toi ALT_EST_TOF_TILT_MIN_COS, range THO ma cam bien
    # do duoc la tran/cos. Gate hep hon so do se cat mau dung tai diem lam viec
    # cao nhat, dung luc dang nghieng.
    if ceiling and gate_max and tilt_cos:
        need = ceiling / tilt_cos
        check("gate estimator phu duoc tran bay SAU khi bu tilt",
              gate_max >= need,
              "can >= %.2fm (tran %.2f / cos %.2f), dang co %.2fm"
              % (need, ceiling, tilt_cos, gate_max))

    # Gate estimator khong duoc vuot chan cung cua driver: vung giua hai nguong
    # la vung estimator CHAP NHAN nhung driver da loai -> mot dai do cao khong
    # bao gio co correction ma khong ai giai thich duoc.
    hard_cap = 4.0 if mode in (1, 2) else 4.0
    check("gate estimator khong vuot chan cung cua driver L1X (4000mm)",
          gate_max is not None and gate_max <= hard_cap,
          "gate %.2fm > %.2fm" % (gate_max or -1, hard_cap))

    print("  (mode=%s TB=%sms IM=%sms -> ~%dHz)" % (mode_name, tb, im, 1000 // im if im else 0))
    print("  (tran bay=%.2fm, tam tin cay=%.2fm, gate=%.2fm, tilt_cos=%.2f)"
          % (ceiling or -1, trusted, gate_max or -1, tilt_cos or -1))


print("")
if fails:
    print("KET QUA: %d FAIL -- %s" % (len(fails), ", ".join(fails)))
    sys.exit(1)
print("KET QUA: ALL PASS")
