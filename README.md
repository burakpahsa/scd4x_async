# scd4x_async

Async-capable MicroPython driver for Sensirion SCD4X CO2 sensor

Updated MicroPython SCD4X driver with:

- Async bus operations with uasyncio.Lock() for non‑blocking I²C access
- Thread‑safe sync methods via a _thread lock
- Async data‑ready polling (data_ready_async)
- Async single‑shot read (read_single_shot_async) returning (CO2, °C, %RH)
- Async periodic mode (start_periodic_async) dispatching samples to a callback
- Normalized conversion formulas matching the datasheet (using 65535 denominator)
- Hex-string serial helper for convenience
- Retained all existing sync features (factory reset, ASC, altitude, calibration, etc.)


Feel free to integrate this async driver into your uasyncio-based projects


Derived from:
- Adafruit CircuitPython SCD4X driver
- peter-l5 MicroPython SCD4X port