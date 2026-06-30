# Proximity Unlock — BLE Scan-Based Smart Lock

![esp32c6](https://img.shields.io/badge/target-ESP32--C6-blue)

A BLE peripheral that unlocks a servo motor when a **bonded phone approaches the door**, using **BLE scanning only** — no repeated connections after initial pairing.

## How It Works

### Pairing

1. Press the button on **GPIO4** to enter pairing mode (60s timeout).
2. Open Bluetooth settings on your phone and connect to `"Proximity Unlock"`.
3. Confirm the pairing code on both sides.
4. After pairing, the device **disconnects** and enters **scan mode**.

> No connection is ever established after pairing. The system listens only.

### Unlock Decision

```
BLE Scan (continuous, no duplicate filter)
  │
  ├─ Device is bonded?
  │    ├─ No → ignore
  │    └─ Yes → check scan cache
  │              ├─ In cache (seen <30s ago) → indoor, ignore
  │              └─ Not in cache → check RSSI
  │                                    ├─ RSSI > -50 dBm → UNLOCK!
  │                                    └─ RSSI ≤ -50 dBm → wait (don't cache)
```

- **Observation window**: 10s after entering scan mode. Any bonded devices seen during this window are silently added to the cache (they're already indoors).
- **Indoor timeout (TTL)**: 30s. If a device is not seen in BLE scans for 30s, it's no longer considered "indoors" and can trigger a new unlock on return.
- **RSSI threshold**: -50 dBm. The phone must be very close to the door to unlock.

### Locking

After unlocking, the servo stays open for **5 seconds**, then automatically rotates back to the lock position.

### Hardware

| Component | Pin |
|-----------|-----|
| Servo (signal) | Configurable via `menuconfig` (default depends on config) |
| Button (pairing) | GPIO4 (active low, internal pull-up) |

### Servo Motion

| Action | Rotation |
|--------|----------|
| Lock (rest) | 180° |
| Unlock | 180° → 30° |
| Duration | 800ms |

### Multi-User Support

Multiple phones can be paired. The first bonded phone to approach triggers the unlock. Once unlocked, the 5s hold timer runs — other phones arriving during this window won't re-trigger.

## Building & Flashing

```bash
# Set environment (adjust paths for your setup)
set IDF_PATH=E:\esp\v6.0\esp-idf
set IDF_TOOLS_PATH=C:\Users\<user>\.espressif
set PATH=%IDF_TOOLS_PATH%\python_env\idf6.0_py3.11_env\Scripts;%PATH%

cd e:\ESP-Project\esp_hid_device
idf.py build
idf.py -p COM<x> flash
```

Monitor output:
```bash
idf.py -p COM<x> monitor
```

## Configuration

Run `idf.py menuconfig` → **Proximity Unlock Configuration**:
- **Servo GPIO pin** — change which GPIO the servo signal wire is connected to

## Project Structure

```
main/
├── CMakeLists.txt              — Build config
├── Kconfig.projbuild           — Menuconfig options
├── esp_hid_device_main.c       — Core logic: scan, cache, unlock
├── esp_hid_gap.c/.h            — BLE GAP layer (scan, advertising)
├── servo_control.c/.h          — SG90 servo PWM driver
```

## Key Parameters (edit in `main.c`)

| Macro | Default | Description |
|-------|---------|-------------|
| `RSSI_THRESHOLD_NEAR` | -50 | RSSI threshold to trigger unlock (dBm) |
| `UNLOCK_HOLD_TIME_US` | 5000000 | Hold unlocked duration (5s) |
| `SCAN_CACHE_TTL_US` | 30000000 | Indoor device TTL (30s) |
| `SCAN_GRACE_PERIOD_US` | 10000000 | Observation window after scan start (10s) |
| `SERVO_ANGLE_UNLOCK` | 30 | Servo position when unlocked (degrees) |
| `SERVO_ANGLE_LOCK` | 180 | Servo position when locked (degrees) |
| `SERVO_MOVE_TIME_MS` | 800 | Time for servo rotation (ms) |
