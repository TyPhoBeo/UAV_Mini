"""Bug THAT da gap tren phan cung: bam ARM -> tu DISARM ngay sau do.

Nguyen nhan KHONG phai mat ket noi. CMD_ARM goi baro_driver_calibrate_ground()
NGAY TRONG stabilize_task, va ham do chan ~970ms (BARO_GROUND_SETTLE_DELAY_MS
300ms + BARO_GROUND_REF_SAMPLES 32 x BARO_GROUND_REF_DELAY_MS 20ms). Suot khoang
do KHONG lenh nao duoc rut khoi queue va commander_evaluate() khong chay, nen
last_heartbeat_us gia di dung 970ms du tram mat dat van gui deu.

Log that (rut gon):
    ARM=0 ... HBAGE=155   FAULT=0
    ARM=1 ... HBAGE=978   FAULT=0   LMX=970536  DLM=1   <- vong bay dung 970ms
    ARM=0 ... HBAGE=1026  FAULT=1              <- vuot 1000ms -> soft fault

Dau hieu phan biet voi mat link THAT: CMDAGE va HBAGE nhay CUNG mot luong (ca
hai dong ho cung bi dong bang) VA loop_max nhay len ~1s. Mat link that thi
loop_max van binh thuong.

TRANSCRIPTION cua buoc 4 trong stabilize_task (flight_core.c) +
commander_credit_stall() (commander.h).
"""
import io
import re
import sys
import pathlib

REPO = pathlib.Path(__file__).resolve().parents[1]
FAILS = []


def check(name, cond, extra=""):
    print(("  OK   " if cond else "  FAIL ") + name + (f"   {extra}" if extra else ""))
    if not cond:
        FAILS.append(name)


def dnum(path, name):
    src = io.open(REPO / path, encoding="utf-8").read()
    m = re.search(r"^#define\s+%s\s+\(?([-+0-9.eE]+)f?\)?" % name, src, re.M)
    assert m, f"khong doc duoc {name} trong {path}"
    return float(m.group(1))


# ---- Hang so THAT, doc thang tu source ----
HB_TIMEOUT_MS = dnum("components/flight_core/include/flight_core/tuning.h",
                     "COMMANDER_DEFAULT_HEARTBEAT_MS")
SETTLE_MS = dnum("components/flight_core/src/drivers/baro_driver.c",
                 "BARO_GROUND_SETTLE_DELAY_MS")
REF_N = dnum("components/flight_core/src/drivers/baro_driver.c",
             "BARO_GROUND_REF_SAMPLES")
REF_DELAY_MS = dnum("components/flight_core/src/drivers/baro_driver.c",
                    "BARO_GROUND_REF_DELAY_MS")
CTRL_HZ = dnum("components/flight_core/src/flight_core.c", "CONTROL_TASK_HZ")

CALIB_BLOCK_MS = SETTLE_MS + REF_N * REF_DELAY_MS
DEADLINE_US = 1_000_000 / CTRL_HZ

print("Hang so doc tu source:")
print(f"  heartbeat_timeout   = {HB_TIMEOUT_MS:.0f} ms")
print(f"  calib baro chan     = {CALIB_BLOCK_MS:.0f} ms "
      f"({SETTLE_MS:.0f} + {REF_N:.0f}x{REF_DELAY_MS:.0f})")
print(f"  1 chu ky dieu khien = {DEADLINE_US:.0f} us")

check("calib baro chan LAU HON heartbeat_timeout (day la goc cua bug)",
      CALIB_BLOCK_MS > HB_TIMEOUT_MS * 0.9,
      f"{CALIB_BLOCK_MS:.0f}ms vs {HB_TIMEOUT_MS:.0f}ms")


class Commander:
    """commander_state_t + commander_credit_stall() (commander.h)."""

    def __init__(self, now_us):
        self.last_heartbeat_us = now_us

    def credit_stall(self, stalled_us):
        if stalled_us <= 0:
            return
        self.last_heartbeat_us += stalled_us

    def hb_age_ms(self, now_us):
        return (now_us - self.last_heartbeat_us) / 1000.0

    def soft_fault(self, now_us):
        return self.hb_age_ms(now_us) > HB_TIMEOUT_MS


def run_arm(credit_enabled, hb_age_at_arm_ms=155.0, block_ms=CALIB_BLOCK_MS):
    """Mo phong buoc 4 cua stabilize_task khi xu ly CMD_ARM."""
    now = 10_000_000
    c = Commander(now - int(hb_age_at_arm_ms * 1000))

    drain_start = now
    # apply_command(CMD_ARM) -> calibrate_ground() chan block_ms
    now += int(block_ms * 1000)
    drain_us = now - drain_start

    if credit_enabled and drain_us > DEADLINE_US:
        c.credit_stall(drain_us)
        if c.last_heartbeat_us > now:      # khong day moc vao TUONG LAI
            c.last_heartbeat_us = now
    return c, now


print("\nTEST 1: TRUOC khi sua -- ARM xong la tu disarm")
c, now = run_arm(credit_enabled=False)
check("khong bu -> HBAGE vuot han", c.hb_age_ms(now) > HB_TIMEOUT_MS,
      f"HBAGE={c.hb_age_ms(now):.0f}ms > {HB_TIMEOUT_MS:.0f}ms")
check("khong bu -> SOFT FAULT (chinh la bug)", c.soft_fault(now) is True)

print("\nTEST 2: SAU khi sua -- ARM giu duoc")
c, now = run_arm(credit_enabled=True)
check("co bu -> HBAGE ve dung tuoi THAT truoc luc chan",
      abs(c.hb_age_ms(now) - 155.0) < 1.0, f"HBAGE={c.hb_age_ms(now):.0f}ms")
check("co bu -> KHONG soft fault", c.soft_fault(now) is False)

print("\nTEST 3 (AN TOAN): mat link THAT van phai trip, khong bi che giau")
# Tram mat dat chet TRUOC khi bam ARM: heartbeat da cu 1200ms roi.
c, now = run_arm(credit_enabled=True, hb_age_at_arm_ms=1200.0)
check("heartbeat da cu san -> VAN soft fault du co bu", c.soft_fault(now) is True,
      f"HBAGE={c.hb_age_ms(now):.0f}ms")

print("\nTEST 4 (AN TOAN): bu KHONG day moc heartbeat vao tuong lai")
# Lenh vua xu ly chinh la CMD_HEARTBEAT -> last_heartbeat_us = now truoc khi chan.
now0 = 10_000_000
c = Commander(now0)
now1 = now0 + int(CALIB_BLOCK_MS * 1000)
c.credit_stall(now1 - now0)
if c.last_heartbeat_us > now1:
    c.last_heartbeat_us = now1
check("moc heartbeat KHONG vuot qua hien tai", c.last_heartbeat_us <= now1,
      f"last={c.last_heartbeat_us} now={now1}")
check("tuoi heartbeat khong am", c.hb_age_ms(now1) >= 0.0,
      f"HBAGE={c.hb_age_ms(now1):.0f}ms")

print("\nTEST 5: xu ly lenh NGAN thi KHONG bu (khong che giau gi)")
c, now = run_arm(credit_enabled=True, block_ms=1.0)   # 1ms < 1 chu ky 4ms
check("drain ngan -> khong goi credit_stall", abs(c.hb_age_ms(now) - 156.0) < 1.5,
      f"HBAGE={c.hb_age_ms(now):.0f}ms (155 + 1ms troi qua)")

# ---- Kiem TRUC TIEP tren source C: co that su goi credit khong ----
print("\nTEST 6: source C phai co duong bu that")
fc = io.open(REPO / "components/flight_core/src/flight_core.c", encoding="utf-8").read()
cmd_h = io.open(REPO / "components/flight_core/include/flight_core/commander.h",
                encoding="utf-8").read()
check("commander.h dinh nghia commander_credit_stall()",
      "commander_credit_stall" in cmd_h)
check("flight_core.c GOI commander_credit_stall sau khi rut queue",
      "commander_credit_stall(&s_cmd_state" in fc)
check("co clamp chong day moc vao tuong lai",
      "s_cmd_state.last_heartbeat_us > after_us" in fc)
check("co reset s_last_tick_us de khong dem deadline-miss oan",
      re.search(r"s_last_tick_us = after_us", fc) is not None)

print()
if FAILS:
    print(f"FAIL: {len(FAILS)} test -> {FAILS}")
    sys.exit(1)
print("PASS: tat ca TEST 1..6 (bu thoi gian firmware tu chiem cua watchdog)")
