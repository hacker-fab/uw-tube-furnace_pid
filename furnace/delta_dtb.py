"""Modbus-ASCII driver for a Delta DTB temperature controller.

Only the registers we actually need for simple PID setpoint control are
exposed. The controller must have "Communication Write Enable" turned on
(hold SET, cycle to ``CoSH`` and enable it) before writes will take effect.

Modbus ASCII frame:
    request :  ':' + slave(2) + func(2) + addr(4) + data(4) + LRC(2) + CRLF
    response:  echo of the request for writes (06), or data frame for reads (03)
"""

import time

import serial

from .config import Config


class DeltaDTB:
    # --- Frame layout ---
    _DATA_START = 7          # index of the first register's hex in a read response
    _HEX_PER_REG = 4         # each 16-bit register is 4 hex chars

    # --- Register map ---
    PV = 0x1000             # process value (current temp), unit 0.1 °C
    SV = 0x1001             # set value (target temp), unit 0.1 °C
    LIMIT_HIGH = 0x1002
    LIMIT_LOW = 0x1003
    SENSOR_TYPE = 0x1004
    CTRL_METHOD = 0x1005   # 0=PID, 1=ON/OFF, 2=manual, 3=PID program
    HEAT_COOL_SEL = 0x1006  # 0=heating
    CYCLE_TIME = 0x1007    # relay duty-cycle period (s)
    PID_P = 0x1009         # proportional band, unit 0.1
    PID_I = 0x100A         # integral time (s)
    PID_D = 0x100B         # derivative time (s)
    HYS1 = 0x1010
    OUT1_VOL = 0x1012      # output power %, unit 0.1
    UNLOCK = 0x102C        # setting lock status
    AUTO_TUNE = 0x103B     # 0=off, 1=running
    RUN_STOP = 0x103C      # 0=stop, 1=run, 2=end, 3=hold

    # Hardware PV error codes (returned in place of a temperature).
    _PV_ERRORS = {
        0x8002: "initializing",
        0x8003: "sensor disconnected",
        0x8004: "sensor input error",
        0x8006: "ADC input error",
        0x8007: "memory error",
    }

    def __init__(self, config: Config):
        self.cfg = config
        self.slave_id = config.slave_id
        self.ser = serial.Serial(
            config.port, config.baud,
            bytesize=7, parity="E", stopbits=1, timeout=1,
        )

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    # ------------------------------------------------------------------ #
    # Low-level frame helpers
    # ------------------------------------------------------------------ #
    @staticmethod
    def _lrc(payload: str) -> str:
        byte_sum = sum(int(payload[i:i + 2], 16) for i in range(0, len(payload), 2))
        return f"{(0x100 - (byte_sum & 0xFF)) & 0xFF:02X}"

    def _frame(self, payload: str) -> bytes:
        return f":{payload}{self._lrc(payload)}\r\n".encode("ascii")

    def _write_register(self, address: int, value: int) -> bool:
        payload = f"{self.slave_id:02X}06{address:04X}{int(value) & 0xFFFF:04X}"
        cmd = self._frame(payload)
        try:
            self.ser.write(cmd)
            resp = self.ser.read(100)
        except serial.SerialException as e:
            print(f"[serial] write error at {hex(address)}: {e}")
            return False
        if not resp:
            return False
        if resp.strip() == cmd.strip():       # controller echoes a successful write
            return True
        if resp[3:5] == b"86":                # 0x06 | 0x80 exception
            return False
        return False

    def safe_write(self, address: int, value: int, retries: int = 3) -> bool:
        """Write a register, retrying a few times on failure."""
        for _ in range(retries):
            if self._write_register(address, value):
                return True
            time.sleep(0.1)
        print(f"[serial] failed to write {hex(address)}={value} after {retries} tries")
        return False

    def _read_registers(self, address: int, count: int = 1):
        """Read ``count`` registers; return a list of ints, or None on failure."""
        payload = f"{self.slave_id:02X}03{address:04X}{count:04X}"
        cmd = self._frame(payload)
        try:
            self.ser.write(cmd)
            time.sleep(0.05)
            resp = self.ser.read(100)
        except serial.SerialException as e:
            print(f"[serial] read error at {hex(address)}: {e}")
            return None
        if not resp or not resp.startswith(b":") or not resp.endswith(b"\r\n"):
            return None
        if resp[3:5] == b"83":                # 0x03 | 0x80 exception
            return None
        try:
            values = []
            for i in range(count):
                start = self._DATA_START + i * self._HEX_PER_REG
                values.append(int(resp[start:start + self._HEX_PER_REG], 16))
            return values
        except (ValueError, IndexError):
            return None

    # ------------------------------------------------------------------ #
    # Reads
    # ------------------------------------------------------------------ #
    def get_pv(self):
        """Current temperature in °C, or None if the read failed / sensor errored."""
        regs = self._read_registers(self.PV, 1)
        if not regs:
            return None
        raw = regs[0]
        if raw in self._PV_ERRORS:
            print(f"[furnace] PV error: {self._PV_ERRORS[raw]} ({raw:#06x})")
            return None
        return raw / 10.0

    def get_sv(self):
        regs = self._read_registers(self.SV, 1)
        return regs[0] / 10.0 if regs else None

    def get_power_percent(self) -> float:
        regs = self._read_registers(self.OUT1_VOL, 1)
        return regs[0] / 10.0 if regs else 0.0

    def get_pid_values(self):
        """Return (P, I, D). P is 0.1-unit scaled; I and D are in seconds."""
        regs = self._read_registers(self.PID_P, 3)
        if not regs:
            return (0.0, 0, 0)
        return (regs[0] / 10.0, regs[1], regs[2])

    def is_auto_tuning(self) -> bool:
        regs = self._read_registers(self.AUTO_TUNE, 1)
        if regs is None:
            return getattr(self, "_last_at", False)
        self._last_at = regs[0] == 1
        return self._last_at

    # ------------------------------------------------------------------ #
    # Writes / commands
    # ------------------------------------------------------------------ #
    def set_setpoint(self, temp: float) -> bool:
        return self.safe_write(self.SV, int(round(temp * 10)))

    def set_control_method(self, method: int) -> bool:
        return self.safe_write(self.CTRL_METHOD, method)

    def set_manual_power(self, percent: float) -> bool:
        return self.safe_write(self.OUT1_VOL, int(round(percent * 10)))

    def set_pid(self, p: float, i: float, d: float):
        self.safe_write(self.PID_P, int(round(p * 10)))
        self.safe_write(self.PID_I, int(round(i)))
        self.safe_write(self.PID_D, int(round(d)))

    def set_hysteresis(self, hys: float) -> bool:
        return self.safe_write(self.HYS1, int(round(hys * 10)))

    def start_auto_tune(self) -> bool:
        return self.safe_write(self.AUTO_TUNE, 1)

    def stop_auto_tune(self) -> bool:
        return self.safe_write(self.AUTO_TUNE, 0)

    def run(self) -> bool:
        return self.safe_write(self.RUN_STOP, 1)

    def stop(self) -> bool:
        return self.safe_write(self.RUN_STOP, 0)

    def emergency_stop(self):
        """Hard-cut the heater: manual mode at 0% power."""
        self.set_control_method(2)
        self.set_manual_power(0)

    def initialize(self):
        """Unlock, set sensor/limits, and enter plain PID setpoint mode at 0 °C."""
        print("[furnace] initializing hardware...")
        self.safe_write(self.UNLOCK, 0)
        self.emergency_stop()                       # start from a known-safe state
        self.safe_write(self.SENSOR_TYPE, self.cfg.sensor_type)
        self.safe_write(self.SV, 0)
        self.safe_write(self.LIMIT_HIGH, int(self.cfg.temp_limit_high * 10))
        self.safe_write(self.LIMIT_LOW, int(self.cfg.temp_limit_low * 10))
        self.safe_write(self.HEAT_COOL_SEL, 0)      # heating only
        self.safe_write(self.CYCLE_TIME, self.cfg.relay_period_s)
        self.set_control_method(0)                  # PID setpoint mode
        print("[furnace] ready.")
