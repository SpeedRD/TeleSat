# TelemetrySat — FreeRTOS CubeSat OBC Simulator

A FreeRTOS-based CubeSat On-Board Computer (OBC) simulator running on an ESP32, implementing a CCSDS-inspired telemetry/telecommand (TM/TC) protocol stack over USB serial with a Raspberry Pi 3B acting as ground station. The system reads live sensor data from a DHT11 (temperature/humidity) and MPU6050 IMU, builds structured 32-byte binary telemetry packets with CRC16-CCITT integrity checking, manages a four-mode satellite state machine, and runs a software watchdog that monitors task liveness and downlinks fault flags in every packet. A Flask + SocketIO web dashboard on the Raspberry Pi provides real-time telemetry display and TC uplink — the full ground-to-space link in miniature.

---

## System Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                        ESP32 OBC                                │
│                                                                 │
│  DHT11 ──── taskSensorRead ──┐                                  │
│  MPU6050 ──────────────────► ├─► g_sensorData (mutex) ──►       │
│  DS1307 RTC ──────────────── ┘   taskPacketBuilder              │
│                                       │ 32-byte TM frame        │
│                                       ▼                         │
│                                  xTxQueue (depth 4)             │
│                                       │                         │
│                                  taskUARTTx                     │
│                                       │ Serial.write() binary   │
└───────────────────────────────────────┼─────────────────────────┘
                                        │
                          USB Serial — 115200 baud
                          TM ↓  32 bytes  (little-endian)
                          TC ↑   6 bytes  (big-endian CRC)
                                        │
┌───────────────────────────────────────┼─────────────────────────┐
│                   Raspberry Pi 3B Ground Station                │
│                                       │                         │
│                              serial_reader.py                   │
│                         (sync on 0x90 0xEB, CRC validate)       │
│                                       │ dict via callback        │
│                                  app.py                         │
│                         (Flask + SocketIO, eventlet)            │
│                                       │ WebSocket               │
│                              Browser Dashboard                  │
│                         (live TM display, TC buttons)           │
│                                       │                         │
│                  TC uplink: button → socketio.emit('command')   │
│                    → tc_builder.py → serial.write() → ESP32     │
└─────────────────────────────────────────────────────────────────┘
```

---

## Hardware

| Component | Role | Interface | Notes |
|-----------|------|-----------|-------|
| ESP32-WROOM-32 (NodeMCU-32S) | OBC — Xtensa LX6 dual-core, 240 MHz, 520 KB RAM | — | FreeRTOS built into Arduino core; no external library needed |
| DHT11 | Temperature + humidity sensor | GPIO4 (single-wire) | 2 s minimum sample interval; sets `FAULT_DHT_ERR` on NaN read |
| MPU6050 GY-521 | 6-axis IMU (accelerometer + gyroscope) | I²C, addr `0x69` | AD0 pulled HIGH to 3V3 to avoid address clash with DS1307 (`0x68`) |
| DS1307 RTC | Real-time Unix timestamp for TM packets | I²C shared bus, addr `0x68` | **Requires 5 V on VCC** (VIN pin, not 3V3). Needs 1000 ms stabilisation after `Wire.begin()` or reads return garbage |
| Green LED | NOMINAL mode indicator | GPIO15 (active HIGH) | Driven exclusively by `taskModeManager` |
| Yellow LED | SAFE mode indicator | GPIO16 (active HIGH) | Green + Yellow both on = DIAGNOSTIC |
| Red LED | FAULT / watchdog indicator | GPIO17 (active HIGH) | Reserved for Week 6+ fault display |
| Raspberry Pi 3B | Ground station | USB serial `/dev/ttyUSB0` or `/dev/ttyUSB1` | Runs Flask + SocketIO dashboard; browser access on port 5000 |

I²C bus: SCL = GPIO22, SDA = GPIO21. Both devices share the same bus with hardware pull-ups.

---

## Firmware Architecture (ESP32)

### FreeRTOS Task Design

| Task | Priority | Stack (words) | Period | Responsibility |
|------|----------|---------------|--------|----------------|
| `taskSensorRead` | 3 (high) | 4096 | 2 s | Reads DHT11 + MPU6050, writes to `g_sensorData`, updates watchdog heartbeat, sets/clears `FAULT_DHT_ERR` |
| `taskPacketBuilder` | 2 | 4096 | 1 s | Snapshots `g_sensorData` under mutex, calls `buildTMPacket()`, pushes to `xTxQueue` |
| `taskUARTTx` | 2 | 4096 | event-driven | Blocks on `xTxQueue`; calls `Serial.write()` with raw 32-byte packet on each dequeue |
| `taskCommandRx` | 3 (high) | 4096 | 10 ms poll | Byte-level TC parser with sync-word state machine; validates CRC16; dispatches mode changes and commands |
| `taskModeManager` | 2 | 4096 | event-driven | Owns LED GPIO pins; blocks on `xModeQueue`; updates `g_currentMode` and `g_sensorData.mode` under mutex |
| `taskWatchdog` | 1 (low) | 4096 | 5 s | Checks `g_heartbeats` tick counts; sets `FAULT_WDG_HIT` and forces SAFE mode on 10 s timeout |
| `taskI2CScanner` | 2 | 4096 | once | Scans I²C bus at boot, prints found addresses, self-terminates via `vTaskDelete(NULL)` |

All tasks are pinned to core 1; core 0 is left free for the ESP32 Wi-Fi/BT stack (unused here but good practice). Stack sizes are set to 4096 words throughout — 2048 caused confirmed stack canary crashes in the DHT library due to its internal use of `Serial.printf` with floating-point formatting.

Shared sensor state is protected by a single `SemaphoreHandle_t xSensorMutex`. The TX path uses a `QueueHandle_t xTxQueue` (depth 4, item size 32 bytes) to decouple `taskPacketBuilder` from `taskUARTTx`, preventing any Serial stall from blocking sensor acquisition. No dynamic memory allocation (`malloc`/`new`) occurs after boot.

### Satellite Mode State Machine

```
         ┌─────────┐
  boot   │         │ self-test complete
 ───────►│  BOOT   ├──────────────────────────┐
         │  (0)    │                          │
         └─────────┘                          ▼
                                         ┌─────────┐
         ┌── CMD_MODE_SAFE ◄─────────────┤  SAFE   │◄──── CMD_MODE_SAFE
         │                               │  (1)    │      watchdog fault
         │   CMD_MODE_NOM ──────────────►└─────────┘
         │                                    ▲
         │   ┌─────────┐                      │ CMD_MODE_SAFE
         └──►│ NOMINAL │◄──── CMD_MODE_NOM    │ watchdog fault
             │  (2)    │                      │
             └────┬────┘                      │
                  │ CMD_MODE_DIAG             │
                  ▼                           │
             ┌──────────────┐                 │
             │ DIAGNOSTIC   ├─────────────────┘
             │   (3)        │
             └──────────────┘
```

| Mode | LED | Behaviour |
|------|-----|-----------|
| BOOT | — | Hardware init, I²C scan, sensor warm-up; transitions to SAFE automatically |
| SAFE | Yellow | Reduced-rate operations, minimal TX; entry point after faults |
| NOMINAL | Green | 1 s sensor read + full TM stream |
| DIAGNOSTIC | Green + Yellow | 200 ms sensor read, verbose output |

This pattern maps directly to the operational modes defined in ECSS-E-ST-70-41C (On/Off, Safe, Normal) — a deliberate choice to keep the architecture recognisable to anyone familiar with real spacecraft software. Ground-commanded mode transitions arrive as TC packets validated before any state change.

### CCSDS-Inspired Telemetry Packet (32 bytes)

All multi-byte fields are little-endian (ESP32 native). Floats are encoded as `int16_t × 100` to avoid IEEE 754 overhead and ensure exact CRC reproducibility across platforms.

| Offset | Size | Field | Encoding | Description |
|--------|------|-------|----------|-------------|
| 0–1 | 2 B | `sync_word` | `0xEB90` | Frame synchronisation marker; ground station scans for `[0x90, 0xEB]` |
| 2–3 | 2 B | `apid` | `0x0001` | Application Process ID — housekeeping TM |
| 4–5 | 2 B | `seq_count` | uint16, wraps at 65535 | Sequence counter; gap detection reveals dropped packets |
| 6–7 | 2 B | `data_len` | `24` | Payload byte count (header excluded) |
| 8–11 | 4 B | `timestamp` | uint32 Unix seconds | DS1307 real-time clock; absolute mission elapsed time |
| 12 | 1 B | `sat_mode` | uint8 (0–3) | Current satellite mode enum value |
| 13–14 | 2 B | `temperature` | int16 = °C × 100 | DHT11 reading; 2501 = 25.01 °C |
| 15–16 | 2 B | `humidity` | int16 = % × 100 | DHT11 relative humidity |
| 17–18 | 2 B | `accel_x` | int16 = m/s² × 100 | MPU6050 X-axis acceleration |
| 19–20 | 2 B | `accel_y` | int16 = m/s² × 100 | MPU6050 Y-axis acceleration |
| 21–22 | 2 B | `accel_z` | int16 = m/s² × 100 | MPU6050 Z-axis acceleration |
| 23–24 | 2 B | `gyro_x` | int16 = rad/s × 100 | MPU6050 X-axis angular rate |
| 25–26 | 2 B | `gyro_y` | int16 = rad/s × 100 | MPU6050 Y-axis angular rate |
| 27–28 | 2 B | `gyro_z` | int16 = rad/s × 100 | MPU6050 Z-axis angular rate |
| 29 | 1 B | `fault_flags` | bitmask | `0x01`=DHT_ERR, `0x02`=IMU_ERR, `0x04`=WDG_HIT |
| 30–31 | 2 B | `crc16` | CRC16-CCITT over bytes 0–29 | Integrity check; packet discarded on mismatch |

The sync word enables robust framing on a noisy serial link — the ground station scans the byte stream for `[0x90, 0xEB]` and attempts packet assembly from that offset, discarding misaligned fragments. Sequence counting allows the dashboard to detect missing packets without explicit acknowledgement. This is the same framing philosophy used in the CCSDS Space Packet Protocol.

### Telecommand Packet (6 bytes)

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0–1 | 2 B | `sync_word` | `0xAB12` — distinct from TM sync to prevent mis-classification |
| 2 | 1 B | `cmd_id` | `0x01`=PING, `0x02`=MODE_SAFE, `0x03`=MODE_NOM, `0x04`=MODE_DIAG, `0x05`=RESET, `0x06`=CLEAR_FAULT |
| 3 | 1 B | `cmd_arg` | Command argument (reserved, `0x00` for current commands) |
| 4–5 | 2 B | `crc16` | CRC16-CCITT over bytes 0–3; big-endian (MSB first) |

The CRC field is big-endian here to match the Python `tc_builder.py` output (`>>8 & 0xFF, then & 0xFF`). Commands are only dispatched after CRC validation passes; invalid packets produce a `[CMD] Bad CRC — discarding` log and are silently dropped.

### CRC16-CCITT

Polynomial `0x1021`, initial value `0xFFFF`, no input/output reflection. The same implementation runs on both the ESP32 (C++, `ccsds.cpp`) and the ground station (Python, `crc16.py`), ensuring bit-exact agreement.

CRC16-CCITT is the standard used in space communications (CCSDS, HDLC, X.25). It was chosen over simpler checksums because it detects all single-bit errors, all double-bit errors, and all burst errors up to 16 bits in length — properties that matter when transmitting over a USB-UART bridge that can occasionally corrupt bytes under load.

### Watchdog & Fault Detection

Three tasks (`taskSensorRead`, `taskMPU6050Test`, `taskCommandRx`) write their `xTaskGetTickCount()` timestamp into a shared `volatile WatchdogHeartbeats` struct after each work cycle. `taskWatchdog` runs at priority 1 (lowest) and wakes every 5 s (`WDG_CHECK_MS`). It compares each stored tick against the current tick; if any heartbeat is stale by more than 10 s (`WDG_TIMEOUT_MS`), it sets `FAULT_WDG_HIT` in `g_sensorData.fault_flags` and queues a forced transition to SAFE mode.

Fault flags are a 3-bit bitmask downlinked at byte 29 of every TM packet:

```
bit 0  FAULT_DHT_ERR  — DHT11 returned NaN (wiring fault or sampling error)
bit 1  FAULT_IMU_ERR  — reserved for MPU6050 init failure detection
bit 2  FAULT_WDG_HIT  — at least one task missed its heartbeat window
```

The `CMD_CLEAR_FAULT` (0x06) telecommand zeroes the full `fault_flags` byte. The watchdog's initial delay equals `WDG_TIMEOUT_MS` (10 s) to allow all tasks to complete their first cycle before liveness monitoring begins — avoiding spurious faults at boot.

---

## Ground Station (Raspberry Pi 3B)

The ground station is a three-layer Python stack:

**Layer 1 — `serial_reader.py`**: A daemon thread continuously reads from the serial port. It maintains a `bytearray` ring buffer and scans for the TM sync pattern `[0x90, 0xEB]` (the little-endian representation of `0xEB90`). When 32 contiguous bytes are found starting at the sync offset, it unpacks the struct with `struct.unpack('<HHHHIBhhhhhhhhBH', raw)`, verifies `crc16 == compute_crc16(raw[:30])`, and calls the registered callback with a parsed dict. Invalid packets are silently discarded.

**Layer 2 — `app.py`**: Flask + Flask-SocketIO (eventlet async mode). The `on_packet` callback calls `socketio.emit('telemetry', pkt)`, broadcasting to all connected browser clients over WebSocket. Incoming `'command'` SocketIO events call `tc_builder.build_tc_packet(cmd_name)`, which constructs and CRC-signs the 6-byte TC frame and writes it directly to the serial port.

**Layer 3 — `templates/index.html`**: A self-contained single-file browser dashboard. SocketIO client receives `'telemetry'` events and updates the DOM in real time — no polling, no page refresh.

TC uplink path in full:
```
Browser button click
  → socketio.emit('command', {cmd: 'MODE_NOM'})
  → app.py on_command()
  → tc_builder.build_tc_packet('MODE_NOM')   # [0xAB, 0x12, 0x03, 0x00, CRC_H, CRC_L]
  → serial.write(6 bytes)
  → ESP32 taskCommandRx (CRC validate → xQueueSend to xModeQueue)
  → taskModeManager (LED update + g_sensorData.mode = MODE_NOMINAL)
  → next TM packet carries sat_mode = 2
```

---

## Dashboard

The browser dashboard (`ground_station/templates/index.html`) is a fully self-contained HTML file with inline CSS and JavaScript — no build step, no npm, deployable by copying a single file. It connects to the Flask-SocketIO server and updates in real time over WebSocket.

Features: live mode indicator with colour-coded LED (green=NOMINAL, amber=SAFE, teal=DIAGNOSTIC, grey=BOOT), packet counter and sequence number, temperature/humidity with large numeric display, 6-axis IMU readout with per-field trend arrows (↑/↓/→ based on delta from previous packet), fault flag indicators (DHT/IMU/WDG) with CLR/ERR badge and blink animation on fault, watchdog health status, scrolling TM packet stream log with per-field colour coding and hex dump, TC command buttons for all six commands, and a CRC display in the footer. The canvas background renders a static Earth-from-orbit scene (pure black space, partial Earth arc in lower-left, atmospheric limb gradient).

---

## Project Structure

```
TelemetrySat/
├── telesat_obc/                  # Arduino sketch (ESP32 firmware)
│   ├── telesat_obc.ino           # Main sketch — globals, all FreeRTOS tasks, setup()
│   ├── config.h                  # Pin assignments, baud rate, timing constants, command IDs
│   ├── ccsds.h                   # TM_Packet + TC_Packet structs, SensorData, fault defines
│   └── ccsds.cpp                 # computeCRC16(), buildTMPacket(), sendPacketBinary(), printPacketHex()
├── ground_station/               # Raspberry Pi ground station
│   ├── app.py                    # Flask + SocketIO server, TC dispatch
│   ├── serial_reader.py          # Daemon thread: byte sync, struct unpack, CRC validation
│   ├── tc_builder.py             # Builds and CRC-signs 6-byte TC packets
│   ├── crc16.py                  # CRC16-CCITT (matches firmware implementation exactly)
│   ├── requirements.txt          # flask, flask-socketio, pyserial, eventlet
│   └── templates/
│       └── index.html            # Self-contained live dashboard (HTML + CSS + JS)
└── README.md
```

---

## Build & Deploy

### Firmware (Arduino IDE)

**Required libraries** (install via Arduino Library Manager):
- `DHT sensor library` by Adafruit (tested with v1.4.x)
- `Adafruit Unified Sensor` (dependency of DHT library)
- `Adafruit MPU6050` (tested with v2.2.x)
- `RTClib` by Adafruit (tested with v2.1.x)

FreeRTOS is provided by the ESP32 Arduino core — do not install a separate FreeRTOS library.

**Board settings** (Tools menu):
- Board: `ESP32 Dev Module`
- CPU Frequency: `240MHz`
- Flash Mode: `QIO`
- Upload Speed: `921600`
- Port: whichever `/dev/cu.usbserial-*` or `COMx` appears on plug-in

**Upload**: Sketch → Upload. The ESP32 will auto-reset into the bootloader on most boards. After upload, open Serial Monitor at 115200 baud to observe `[BOOT]`, `[SENSOR]`, `[IMU]`, `[MODE]`, and `[WDG]` diagnostic output. Note: `[UART]` output was removed from `taskUARTTx` — raw binary packets are sent silently.

### Ground Station

```bash
cd ground_station
pip3 install -r requirements.txt
python3 app.py
# Open http://<raspberry-pi-ip>:5000 in any browser
```

Override the serial port if the ESP32 appears on a different device node:

```bash
SERIAL_PORT=/dev/ttyACM0 python3 app.py
```

Check available ports with `ls /dev/tty*` after plugging in the USB cable. The serial reader reconnects automatically if the port is temporarily unavailable.

---

## Key Engineering Decisions

**1. FreeRTOS over a superloop**

A traditional Arduino `loop()` superloop would require manual time-slicing with `millis()` comparisons and careful ordering of tasks to avoid one slow operation blocking others. FreeRTOS gives genuine preemptive scheduling: `taskSensorRead` at priority 3 will preempt lower-priority tasks the moment it unblocks, regardless of what they are doing. More importantly, if any task stalls (hung on a sensor read, for example), the watchdog task — which runs independently — will still fire and flag the fault. This isolation between tasks is the property that makes FreeRTOS appropriate for safety-critical systems.

**2. CCSDS-inspired packet structure**

Building a proprietary binary format would have been simpler, but CCSDS provides a tested template for the exact problems this project faces: sync word framing for stream recovery after byte loss, APID routing for future multi-application expansion, and sequence counting for gap detection without acknowledgement overhead. Using a structure recognisable to aerospace engineers also makes the project legible to an academic reviewer familiar with ESA or NASA mission software.

**3. Binary protocol over ASCII serial**

Earlier prototypes used `Serial.printf` for all output. This created a fundamental problem: the ground station had to parse human-readable strings, which are fragile (format changes break the parser), bandwidth-inefficient (a float like `25.34` costs 5 bytes vs 2 bytes as `int16_t`), and impossible to CRC-protect as a whole. Switching to binary with a fixed-length framed packet means the receiver always knows exactly how many bytes to expect, can CRC-validate the entire payload in one call, and is immune to text formatting changes in the firmware.

**4. Decoupled TX queue**

`taskPacketBuilder` and `taskUARTTx` could have been merged into a single task that builds and immediately writes each packet. The problem: `Serial.write()` blocks until the UART buffer drains. At 115200 baud, transmitting 32 bytes takes approximately 2.8 ms — long enough to delay the next sensor read if both tasks are fused. The `xTxQueue` with depth 4 decouples them: the builder deposits packets at its own rate and immediately returns to wait for the next cycle, while the UART task drains the queue at whatever rate the serial link permits. This is the producer/consumer pattern from operating systems theory applied directly.

---

## What I Learned / Context

This project was built as a portfolio piece, specifically to demonstrate embedded systems and spacecraft software engineering competence. The goal was to go beyond blinking LEDs and actually implement the kind of system architecture — tasking, IPC, protocol framing, fault detection — that appears in real OBC software like NASA cFS or ESA's OBSW reference implementations.

The project started on a NodeMCU V3 (ESP8266). That choice lasted about one afternoon: the ESP8266 Arduino core uses the Non-OS SDK and does not expose FreeRTOS headers to sketch code. Migrating to ESP32 resolved this, but introduced its own surprises — stack canary crashes traced back to `STACK_SIZE = 2048` (the DHT library's `Serial.printf` with float formatting exhausts this), and the DS1307 RTC silently reading garbage until I realised it requires 5 V on VCC, not 3.3 V, and a 1000 ms stabilisation delay after `Wire.begin()`. The MPU6050 and DS1307 share the I²C bus at addresses `0x69` and `0x68` respectively — the AD0 pin on the MPU6050 must be pulled HIGH to avoid a collision. These are the kinds of details that only appear in datasheets, not tutorials.

The most instructive part of the ground station work was the byte-sync parser in `serial_reader.py`. Receiving a structured binary protocol over a live serial stream means you cannot assume the first byte you read is the start of a packet — you need to scan for the sync word, handle partial packets at buffer boundaries, and discard fragments with bad CRCs without blocking the receiver. Getting this right required working through the edge cases deliberately, rather than assuming the stream would be well-behaved.

---

## References

- [CCSDS Space Packet Protocol — CCSDS 133.0-B-2](https://public.ccsds.org/Pubs/133x0b2e1.pdf)
- [ECSS-E-ST-70-41C — Telemetry and Telecommand Packet Utilization](https://ecss.nl/standard/ecss-e-st-70-41c-space-engineering-telemetry-and-telecommand-packet-utilization-15-april-2016/)
- [ESP32 Technical Reference Manual — Espressif Systems](https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf)
- [FreeRTOS Kernel Developer Docs](https://www.freertos.org/Documentation/RTOS_book.html)
- [NASA Core Flight System (cFS)](https://cfs.gsfc.nasa.gov/) — reference architecture for real OBC software; the task/message-bus pattern here is a simplified analogue

---

## License

MIT — see LICENSE file.
