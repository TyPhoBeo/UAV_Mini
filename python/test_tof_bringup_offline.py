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


def define_int(name):
    m = re.search(r"^#define\s+" + name + r"\s+(\d+)", src, re.M)
    return int(m.group(1)) if m else None


boot_ms = define_int("VL53L0X_BOOT_DELAY_MS")
hold_ms = define_int("VL53L0X_XSHUT_HOLD_MS")
probe_tries = define_int("VL53L0X_PROBE_ATTEMPTS")

# vTaskDelay(pdMS_TO_TICKS(n)) chi bao dam (n-1) tick TRON VEN: tick dau tien bi
# cat cut theo pha. Voi tick 1000Hz -> thoi gian THAT toi thieu = (n-1) ms.
T_BOOT_MAX_MS = 1.2
check("VL53L0X_BOOT_DELAY_MS ton tai", boot_ms is not None)
check("boot delay du cho t_boot=1.2ms ke ca khi tick bi cat cut",
      boot_ms is not None and (boot_ms - 1) >= T_BOOT_MAX_MS,
      "boot_ms=%s -> toi thieu that %.1fms" % (boot_ms, (boot_ms - 1) if boot_ms else -1))
check("gia tri cu (2ms) DA bi bat la vi pham", (2 - 1) < T_BOOT_MAX_MS)
check("VL53L0X_XSHUT_HOLD_MS ton tai va >= boot delay",
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
      "VL53L0X_DEFAULT_I2C_ADDR" in code and "wanted_addr" in code)

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


print("")
if fails:
    print("KET QUA: %d FAIL -- %s" % (len(fails), ", ".join(fails)))
    sys.exit(1)
print("KET QUA: ALL PASS")
