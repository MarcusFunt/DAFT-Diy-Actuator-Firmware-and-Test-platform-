# DAFT ESP32-C6 TMC2209 Stepper Controller

This repo contains a small test platform for controlling one NEMA17 stepper motor through an ESP32-C6 and a TMC2209 driver.

- `arduino/ESP32C6_TMC2209_Controller/ESP32C6_TMC2209_Controller.ino`
- `python/stepper_control_ui.py`
- `python/stepper_control_dpg.py`
- `python/daft_protocol.py`

## Arduino IDE Setup

Install these Arduino libraries:

- `FastAccelStepper`
- `TMCStepper`

Select your ESP32-C6 board, open the `.ino` file, and upload it.

Enable `USB CDC On Boot` for the selected ESP32-C6 board. Without this option, uploads over COM5 can still work, but the running sketch's `Serial` object will not use the native USB CDC port.

With Arduino CLI, the generic ESP32-C6 build/upload target is:

```powershell
arduino-cli compile --fqbn esp32:esp32:esp32c6:CDCOnBoot=cdc arduino\ESP32C6_TMC2209_Controller
arduino-cli upload -p COM5 --fqbn esp32:esp32:esp32c6:CDCOnBoot=cdc arduino\ESP32C6_TMC2209_Controller
```

The sketch uses these pins:

```cpp
#define PIN_STEP            20
#define PIN_DIR             19
#define PIN_ENABLE          18
#define PIN_TMC_UART_RX     17
#define PIN_TMC_UART_TX     16
```

`PIN_ENABLE` is active low. The TMC2209 UART wiring is:

```text
ESP32-C6 GPIO16 TX -> 1 kOhm -> TMC2209 PDN_UART
ESP32-C6 GPIO17 RX -----------> TMC2209 PDN_UART
GND --------------------------> TMC2209 GND
```

Set the motor current conservatively first. The default is `800 mA RMS`.

## Python UIs

Create a virtual environment if you want one, then install the Python dependencies:

```powershell
pip install -r python/requirements.txt
```

Run the newer DearPyGUI console:

```powershell
python python/stepper_control_dpg.py
```

The original Tkinter UI is still available:

```powershell
python python/stepper_control_ui.py
```

Both UIs talk to the ESP32 over DAFT Motion Binary v1. The old newline text command protocol has been removed from the GUI path.

Before sending motion commands, enable the driver with the UI checkbox. Motion commands are rejected while the driver is disabled.

Positions, speeds, and accelerations are integer step-pulse units. If you set `16` microsteps, one 200-step/rev NEMA17 revolution is `3200` position units.

## DAFT Motion Binary v1

The production command channel is a compact binary protocol:

| Layer | Choice |
| --- | --- |
| Physical transport | USB CDC serial |
| Framing | COBS, terminated by `0x00` |
| Raw packet size | 32 bytes before COBS |
| Endianness | Little-endian |
| Numbers | Integers only |
| Integrity | CRC-16/CCITT-FALSE over bytes `0..29` |
| Timing ownership | ESP32 owns step timing |

Raw packet layout before COBS:

| Offset | Field | Type |
| --- | --- | --- |
| 0 | version | `u8`, currently `1` |
| 1 | msg_id | `u8` |
| 2 | flags | `u8` |
| 3 | payload_len | `u8`, max `23` |
| 4-5 | seq | `u16` |
| 6-28 | payload | 23 bytes |
| 29 | reserved | `u8`, zero |
| 30-31 | crc16 | `u16` |

Host-to-ESP32 command IDs:

| ID | Name |
| --- | --- |
| `0x01` | `PING` |
| `0x02` | `GET_STATUS` |
| `0x03` | `SET_TELEM_RATE` |
| `0x10` | `ENABLE` |
| `0x11` | `DISABLE` |
| `0x12` | `ESTOP` |
| `0x13` | `STOP_RAMP` |
| `0x14` | `ZERO_POSITION` |
| `0x20` | `MOVE_ABS` |
| `0x21` | `MOVE_REL` |
| `0x22` | `RUN_VEL` |
| `0x23` | `SET_SOFT_LIMITS` |
| `0x30` | `DRIVER_CONFIG` |
| `0x40` | `CLEAR_FAULTS` |

ESP32-to-host packet IDs:

| ID | Name |
| --- | --- |
| `0x80` | `ACK` |
| `0x81` | `NACK` |
| `0x82` | `STATUS` |
| `0x83` | `EVENT` |

The ESP32 replies to valid command packets with `ACK(seq)` or `NACK(seq, reason)`. Corrupt COBS/CRC frames are dropped without reply because the sequence number cannot be trusted.

Default telemetry is off after boot. The GUI enables sparse `STATUS` telemetry while connected, and the firmware drops non-critical telemetry if the USB transmit path is busy. Text logging is intentionally disabled on the command port.
