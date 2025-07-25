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


### MicroPython SCD4X Async Driver Documentation

This document provides installation instructions, usage examples, and a reference for the `SCD4X` class with both synchronous and asynchronous APIs.

---

## 1. Installation

1. Copy `micropython_scd4x_async.py` into your project directory.
2. Ensure your board has MicroPython with `uasyncio` enabled.
3. Import in your script:

   ```python
   from micropython_scd4x_async import SCD4X
   ```

---

## 2. Initialization

```python
from machine import I2C, Pin
import uasyncio as asyncio

# Create I2C bus at 100kHz (example pins)
i2c = I2C(0, sda=Pin(2), scl=Pin(3), freq=100000)

# Instantiate sensor (default address: 0x62)
sensor = SCD4X(i2c)
```

---

## 3. Synchronous API

### 3.1 Single‑Shot Measurements

```python
# CO2 + Temperature + Humidity (SCD41 only)
sensor.measure_single_shot()
# Wait until ready
while not sensor.data_ready:
    time.sleep(0.1)
co2 = sensor.CO2
temp = sensor.temperature
rh  = sensor.relative_humidity

# T+RH only
sensor.measure_single_shot_rht_only()
while not sensor.data_ready:
    time.sleep(0.05)
temp = sensor.temperature
rh   = sensor.relative_humidity
```

### 3.2 Periodic Mode

```python
# Start periodic measurement (~5s interval)
sensor.start_periodic_measurement()

# Later in loop:
if sensor.data_ready:
    print(sensor.CO2, sensor.temperature, sensor.relative_humidity)
```

### 3.3 Configuration

* `sensor.reinit()` – reload settings
* `sensor.factory_reset()` – reset EEPROM
* `sensor.force_calibration(ppm: int)` – set new baseline
* `sensor.self_calibration_enabled = True|False`
* `sensor.set_ambient_pressure(hpa: int)`
* `sensor.altitude = meters`
* `sensor.temperature_offset = degrees_c`
* `sensor.persist_settings()`
* `sensor.serial_number` → hex string

---

## 4. Asynchronous API

All async methods require `uasyncio`.

### 4.1 Async Single‑Shot

```python
async def one_shot():
    co2, temp, rh = await sensor.read_single_shot_async()
    print(co2, temp, rh)

asyncio.run(one_shot())
```

### 4.2 Async Data‑Ready Polling

```python
# Manually await readiness
await sensor.data_ready_async(poll_interval_ms=200)
print(sensor.CO2)
```

### 4.3 Async Periodic

```python
async def log_readings(co2, t, rh):
    print(f"CO2={co2}ppm  T={t:.1f}°C  RH={rh:.1f}%")

# Start background task (runs forever)
asyncio.create_task(sensor.start_periodic_async(interval_s=5, callback=log_readings))

# Run other async tasks...
asyncio.run(main())
```

---

## 5. API Reference

| Method / Property                        | Type      | Description                                                               |
| ---------------------------------------- | --------- | ------------------------------------------------------------------------- |
| `SCD4X(i2c, address=0x62)`               | class     | Constructor; takes a `machine.I2C` instance                               |
| `CO2`                                    | property  | Latest CO₂ reading (ppm)                                                  |
| `temperature`                            | property  | Latest temperature (°C)                                                   |
| `relative_humidity`                      | property  | Latest RH (%)                                                             |
| `data_ready`                             | property  | `True` if a new sample is available                                       |
| `measure_single_shot()`                  | method    | Sync one‑shot CO₂+T+RH (SCD41 only)                                       |
| `measure_single_shot_rht_only()`         | method    | Sync one‑shot T+RH only (SCD41 only)                                      |
| `start_periodic_measurement()`           | method    | Enable periodic mode (\~5s sample interval)                               |
| `start_low_power_periodic()`             | method    | Enable low‑power periodic (\~30s interval)                                |
| `stop_periodic_measurement()`            | method    | Stop any measurement mode                                                 |
| `read_single_shot_async()`               | coroutine | Async one‑shot, returns `(co2, temp, rh)`                                 |
| `data_ready_async(poll_interval_ms=100)` | coroutine | Await data ready without blocking                                         |
| `start_periodic_async(interval_s, cb)`   | coroutine | Async periodic loop invoking `cb(co2,temp,rh)` every `interval_s` seconds |
| `reinit()`                               | method    | Reload user settings from EEPROM                                          |
| `factory_reset()`                        | method    | Erase EEPROM settings                                                     |
| `force_calibration(ppm)`                 | method    | Force recalibration to `ppm`                                              |
| `self_calibration_enabled`               | property  | Get/set ASC enable flag                                                   |
| `persist_settings()`                     | method    | Write current offsets/ASC to EEPROM                                       |
| `set_ambient_pressure(hpa)`              | method    | Adjust CO₂ calc for local pressure                                        |
| `altitude`                               | property  | Get/set altitude (m)                                                      |
| `temperature_offset`                     | property  | Get/set temperature offset (°C)                                           |
| `serial_number`                          | property  | Get sensor serial as hex string                                           |

---

*For more details on conversion formulas and timing, consult the Sensirion SCD4X datasheet.*
