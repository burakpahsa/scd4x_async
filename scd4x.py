# SPDX-FileCopyrightText: 2017 Scott Shawcroft, written for Adafruit Industries
# SPDX-FileCopyrightText: Copyright (c) 2021 ladyada for Adafruit Industries
# SPDX-FileCopyrightText: Copyright (c) 2022-2024 peter-l5
# SPDX-License-Identifier: MIT
"""
`micropython_scd4x_async`
================================================================================

Async-capable MicroPython driver for Sensirion SCD4X CO2 sensor

Derived from:
- Adafruit CircuitPython SCD4X driver
- peter-l5 MicroPython SCD4X port

Adds uasyncio-based methods for non-blocking single-shot and periodic reads,
plus bus locking for both sync and async use.
"""

import time
import struct
from machine import I2C
from micropython import const
import uasyncio as asyncio
import _thread

# I2C address
SCD4X_DEFAULT_ADDR = const(0x62)

# Command constants
_SCD4X_REINIT                                = const(0x3646)
_SCD4X_FACTORYRESET                          = const(0x3632)
_SCD4X_FORCEDRECAL                           = const(0x362F)
_SCD4X_SELFTEST                              = const(0x3639)
_SCD4X_DATAREADY                             = const(0xE4B8)
_SCD4X_STOPPERIODICMEASUREMENT               = const(0x3F86)
_SCD4X_STARTPERIODICMEASUREMENT              = const(0x21B1)
_SCD4X_STARTLOWPOWERPERIODICMEASUREMENT      = const(0x21AC)
_SCD4X_READMEASUREMENT                       = const(0xEC05)
_SCD4X_SERIALNUMBER                          = const(0x3682)
_SCD4X_GETTEMPOFFSET                         = const(0x2318)
_SCD4X_SETTEMPOFFSET                         = const(0x241D)
_SCD4X_GETALTITUDE                           = const(0x2322)
_SCD4X_SETALTITUDE                           = const(0x2427)
_SCD4X_SETPRESSURE                           = const(0xE000)
_SCD4X_PERSISTSETTINGS                       = const(0x3615)
_SCD4X_GETASCE                               = const(0x2313)
_SCD4X_SETASCE                               = const(0x2416)
_SCD4X_MEASURESINGLESHOT                     = const(0x219D)
_SCD4X_MEASURESINGLESHOTRHTONLY              = const(0x2196)

class SCD4X:
    """
    MicroPython helper class for SCD4X, with both sync and async methods.
    """
    # Locks for bus arbitration
    _i2c_lock_thread = _thread.allocate_lock()
    _i2c_lock_async  = asyncio.Lock()

    def __init__(self, i2c_bus: I2C, address: int = SCD4X_DEFAULT_ADDR) -> None:
        self.address = address
        self.i2c = i2c_bus
        self._buffer = bytearray(18)
        self._cmd    = bytearray(2)
        self._crcbuf = bytearray(2)
        self._temperature = None
        self._relative_humidity = None
        self._co2 = None
        # Ensure sensor is idle
        self.stop_periodic_measurement()

    # --- Sync bus operations (with thread lock) ---
    def _send_command(self, cmd: int, cmd_delay: float = 0) -> None:
        self._cmd[0] = (cmd >> 8) & 0xFF
        self._cmd[1] = cmd & 0xFF
        with SCD4X._i2c_lock_thread:
            self.i2c.writeto(self.address, self._cmd)
        time.sleep(cmd_delay)

    def _read_reply(self, buff: bytearray, num: int) -> None:
        with SCD4X._i2c_lock_thread:
            self.i2c.readfrom_into(self.address, buff)
        self._check_buffer_crc(buff[:num])

    # --- Async bus operations ---
    async def _send_command_async(self, cmd: int, delay: float = 0) -> None:
        self._cmd[0] = (cmd >> 8) & 0xFF
        self._cmd[1] = cmd & 0xFF
        async with SCD4X._i2c_lock_async:
            self.i2c.writeto(self.address, self._cmd)
        await asyncio.sleep(delay)

    async def _read_reply_async(self, buff: bytearray, num: int) -> None:
        async with SCD4X._i2c_lock_async:
            self.i2c.readfrom_into(self.address, buff)
        self._check_buffer_crc(buff[:num])

    # --- CRC8 check ---
    @staticmethod
    def _crc8(buffer: bytearray) -> int:
        crc = 0xFF
        for b in buffer:
            crc ^= b
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0x31
                else:
                    crc <<= 1
        return crc & 0xFF

    def _check_buffer_crc(self, buf: bytearray) -> bool:
        for i in range(0, len(buf), 3):
            self._crcbuf[0] = buf[i]
            self._crcbuf[1] = buf[i+1]
            if self._crc8(self._crcbuf) != buf[i+2]:
                raise RuntimeError("CRC check failed")
        return True

    # --- Properties & sync methods ---
    @property
    def CO2(self) -> int:
        if self.data_ready:
            self._read_data()
        return self._co2

    @property
    def temperature(self) -> float:
        if self.data_ready:
            self._read_data()
        return self._temperature

    @property
    def relative_humidity(self) -> float:
        if self.data_ready:
            self._read_data()
        return self._relative_humidity

    @property
    def data_ready(self) -> bool:
        self._send_command(_SCD4X_DATAREADY, cmd_delay=0.001)
        self._read_reply(self._buffer, 3)
        # msb bits non-zero indicates ready
        return not ((self._buffer[0] & 0x07 == 0) and (self._buffer[1] == 0))

    def _read_data(self) -> None:
        self._send_command(_SCD4X_READMEASUREMENT, cmd_delay=0.001)
        self._read_reply(self._buffer, 9)
        # CO2
        self._co2 = (self._buffer[0] << 8) | self._buffer[1]
        # Temperature (per datasheet)
        raw_t = (self._buffer[3] << 8) | self._buffer[4]
        self._temperature = -45 + 175 * (raw_t / 65535)
        # Humidity
        raw_h = (self._buffer[6] << 8) | self._buffer[7]
        self._relative_humidity = 100 * (raw_h / 65535)

    # --- Configuration methods ---
    def reinit(self) -> None:
        self.stop_periodic_measurement()
        self._send_command(_SCD4X_REINIT, cmd_delay=0.02)

    def factory_reset(self) -> None:
        self.stop_periodic_measurement()
        self._send_command(_SCD4X_FACTORYRESET, cmd_delay=1.2)

    def force_calibration(self, target_co2: int) -> None:
        self.stop_periodic_measurement()
        self._set_command_value(_SCD4X_FORCEDRECAL, target_co2)
        time.sleep(0.5)
        self._read_reply(self._buffer, 3)
        corr = struct.unpack_from(">h", self._buffer[0:2])[0]
        if corr == 0xFFFF:
            raise RuntimeError("Forced recalibration failed; ensure sensor active >3 min.")

    @property
    def self_calibration_enabled(self) -> bool:
        self._send_command(_SCD4X_GETASCE, cmd_delay=0.001)
        self._read_reply(self._buffer, 3)
        return self._buffer[1] == 1

    @self_calibration_enabled.setter
    def self_calibration_enabled(self, enable: bool) -> None:
        self._set_command_value(_SCD4X_SETASCE, enable)

    def self_test(self) -> None:
        self.stop_periodic_measurement()
        self._send_command(_SCD4X_SELFTEST, cmd_delay=10)
        self._read_reply(self._buffer, 3)
        if any(b != 0 for b in self._buffer[:2]):
            raise RuntimeError("Self test failed")

    def stop_periodic_measurement(self) -> None:
        self._send_command(_SCD4X_STOPPERIODICMEASUREMENT, cmd_delay=0.5)

    def start_periodic_measurement(self) -> None:
        self._send_command(_SCD4X_STARTPERIODICMEASUREMENT)

    def start_low_power_periodic(self) -> None:
        self._send_command(_SCD4X_STARTLOWPOWERPERIODICMEASUREMENT)

    def persist_settings(self) -> None:
        self._send_command(_SCD4X_PERSISTSETTINGS, cmd_delay=0.8)

    def set_ambient_pressure(self, pressure_hpa: int) -> None:
        if not (0 <= pressure_hpa <= 65535):
            raise AttributeError("Pressure must be 0–65535 hPa")
        self._set_command_value(_SCD4X_SETPRESSURE, pressure_hpa)

    @property
    def altitude(self) -> int:
        self._send_command(_SCD4X_GETALTITUDE, cmd_delay=0.001)
        self._read_reply(self._buffer, 3)
        return (self._buffer[0] << 8) | self._buffer[1]

    @altitude.setter
    def altitude(self, meters: int) -> None:
        if not (0 <= meters <= 65535):
            raise AttributeError("Altitude must be 0–65535 m")
        self._set_command_value(_SCD4X_SETALTITUDE, meters)

    @property
    def temperature_offset(self) -> float:
        self._send_command(_SCD4X_GETTEMPOFFSET, cmd_delay=0.001)
        self._read_reply(self._buffer, 3)
        raw = (self._buffer[0] << 8) | self._buffer[1]
        return raw * 175.0 / 65535

    @temperature_offset.setter
    def temperature_offset(self, offset_c: float) -> None:
        if offset_c > 374:
            raise AttributeError("Offset <=374°C")
        val = int(offset_c * 65535 / 175)
        self._set_command_value(_SCD4X_SETTEMPOFFSET, val)

    def _set_command_value(self, cmd: int, value, cmd_delay: float = 0) -> None:
        self._cmd[0] = (cmd >> 8) & 0xFF
        self._cmd[1] = cmd & 0xFF
        self._crcbuf[0] = self._cmd[0]
        self._crcbuf[1] = (value >> 8) & 0xFF
        self._buffer[0] = self._cmd[0]
        self._buffer[1] = self._cmd[1]
        self._buffer[2] = (value >> 8) & 0xFF
        self._buffer[3] = value & 0xFF
        self._buffer[4] = self._crc8(self._crcbuf)
        with SCD4X._i2c_lock_thread:
            self.i2c.writeto(self.address, self._buffer[:5])
        time.sleep(cmd_delay)

    @property
    def serial_number(self) -> str:
        self._send_command(_SCD4X_SERIALNUMBER, cmd_delay=0.001)
        self._read_reply(self._buffer, 9)
        # convert tuple to hex string
        sn = tuple(self._buffer[i] for i in (0,1,3,4,6,7))
        return "".join(f"{b:02X}" for b in sn)

    # --- Single-shot sync methods ---
    def measure_single_shot(self) -> None:
        """Trigger one-shot CO2+T+RH (SCD41 only)."""
        self._send_command(_SCD4X_MEASURESINGLESHOT, cmd_delay=5)

    def measure_single_shot_rht_only(self) -> None:
        """Trigger one-shot T+RH (SCD41 only)."""
        self._send_command(_SCD4X_MEASURESINGLESHOTRHTONLY, cmd_delay=0.05)

    # --- Async-friendly methods ---
    async def data_ready_async(self, poll_interval_ms: int = 100) -> bool:
        """Poll asynchronously until new data is ready."""
        while True:
            if self.data_ready:
                return True
            await asyncio.sleep_ms(poll_interval_ms)

    async def read_single_shot_async(self) -> tuple:
        """Trigger one-shot and return (co2_ppm, temp_c, rh_pct)."""
        await self._send_command_async(_SCD4X_MEASURESINGLESHOT, delay=5)
        await self.data_ready_async()
        return (self.CO2, self.temperature, self.relative_humidity)

    async def start_periodic_async(self, interval_s: int = 5, callback=None) -> None:
        """Start periodic mode and dispatch samples to callback."""
        self.start_periodic_measurement()
        while True:
            await asyncio.sleep(interval_s)
            if self.data_ready:
                sample = (self.CO2, self.temperature, self.relative_humidity)
                if callback:
                    callback(*sample)
