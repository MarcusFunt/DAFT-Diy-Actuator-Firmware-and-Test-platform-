# DAFT V1 Actuator Firmware

DAFT is reusable actuator firmware for one locally planned TMC2209 stepper axis on ESP32-C6. Hosts send mode, target, configuration, and commissioning commands over DAFT Motion Binary v2; the actuator owns pulse timing, driver state, safety limits, persistence, telemetry, and faults.

Out of V1: multi-axis, encoders, homing, CAN/CAN-FD, STM32 HAL backend, ROS2, trajectory queues, closed-loop control, and firmware updates.

## Layout

```text
firmware/
  include/daft/              Platform-neutral C++ API and protocol types
  src/                       Protocol codec, config validation, state machine, config store
  platforms/esp32_arduino/   PlatformIO ESP32-C6 backend
  tests/                     Host-side C++ tests
  platformio.ini             PlatformIO firmware build
  CMakeLists.txt             Host test build

protocol/
  golden_frames.txt          Shared binary protocol fixtures for C++ and Python tests

python/
  daft_protocol/             Binary v2 codec, serial transport, client, config metadata
  apps/cli.py                CLI smoke, commissioning, bench, and soak tool
  apps/dpg_commissioning_gui.py
  tests/                     Python protocol tests
```

## Firmware Build

Install PlatformIO, then build the ESP32-C6 firmware:

```powershell
pio run -d firmware -e esp32c6
```

Upload:

```powershell
pio run -d firmware -e esp32c6 -t upload --upload-port COM5
```

The ESP32-C6 environment uses the Arduino framework through PlatformIO because the hardware backend depends on FastAccelStepper and TMCStepper. The project is no longer an Arduino IDE sketch. This machine's stock PlatformIO `espressif32@6.10.0` package does not support Arduino on ESP32-C6, so `firmware/platformio.ini` pins the environment to `pioarduino/platform-espressif32`.

The firmware identity response includes firmware version, protocol version, config version, config generation, git SHA, UTC build date, and dirty flag. Bench logs capture that identity so hardware reports prove exactly which binary was tested.

Current pins are defined in `firmware/include/daft/config.hpp`:

```cpp
step=20, dir=19, enable=18, tmc_uart_rx=17, tmc_uart_tx=16
```

`enable` is active low. TMC2209 UART wiring:

```text
ESP32-C6 GPIO16 TX -> 1 kOhm -> TMC2209 PDN_UART
ESP32-C6 GPIO17 RX -----------> TMC2209 PDN_UART
GND --------------------------> TMC2209 GND
```

## Tests And CI

CI runs Python tests, CMake/CTest, and the PlatformIO firmware build from `.github/workflows/ci.yml`.

Run the same checks locally:

```powershell
python -m pip install -e ".[test]"
python -m pytest -q python\tests

cmake -G "MinGW Makefiles" -S firmware -B firmware\build-mingw
cmake --build firmware\build-mingw
ctest --test-dir firmware\build-mingw --output-on-failure

pio run -d firmware -e esp32c6
```

The C++ and Python suites both check `protocol/golden_frames.txt` so DAFT Motion Binary v2 wire format drift is caught before hardware testing. The two-slot persistence algorithm is also tested off-device for invalid markers, corrupted payloads, partial writes, old generations, and generation wrap.

## Python Tools

Install the CLI:

```powershell
python -m pip install -e ".[test]"
```

Run common commands without setting `PYTHONPATH`:

```powershell
daft scan
daft ping COM5
daft identity COM5
daft status COM5
daft driver-status COM5
daft mode COM5 position
daft move-rel COM5 --steps 1000 --speed 2000 --accel 5000
daft stop COM5
daft config fields
daft config get COM5 --field all
daft config stage COM5 run_current_ma 700
daft config apply COM5
daft config save COM5
```

Run the DearPyGUI commissioning app:

```powershell
python python\apps\dpg_commissioning_gui.py
```

## Hardware Validation

Short baseline bench:

```powershell
daft bench COM5 --cycles 2 --steps 1000 --speed 1000 --accel 2000 --telemetry-interval-ms 100 --velocity-run-s 0.8 --host-timeout-ms 250
```

Default artifacts:

```text
hardware_logs/bench_YYYYMMDD_HHMMSS.jsonl
hardware_logs/bench_YYYYMMDD_HHMMSS.report.json
```

The JSONL file remains the detailed event stream. The report JSON summarizes pass/fail, identity, movements completed, observed fault bitmask, max counter deltas, UART status, final state, config restoration, duration, telemetry packets, and EVENT packet counts.

Expected final state for a passing bench:

```text
report.ok = true
final_status.state = DISABLED
final_status.faults = 0
final_status.config_staged = false
uart_status.final.uart_ok = true
config_restored = true
```

Recovery if a bench fails or is interrupted:

```powershell
daft ramp-stop COM5 --accel 2000   # or power remove if motion is unsafe
daft clear-faults COM5
daft mode COM5 disabled
daft config get COM5 --field all
daft config stage COM5 host_timeout_ms 1000
daft config stage COM5 direction_inverted 0   # restore the value from the last report if different
daft config apply COM5
daft config save COM5
```

If serial no longer responds, power-cycle the ESP32-C6 and TMC2209, rerun `daft identity COM5`, then restore config from the latest report before continuing.

Longer soak starting point:

```powershell
daft bench COM5 --duration-min 30 --steps 1000 --speeds 500,1000,1500 --accels 1000,2000,4000 --telemetry-intervals-ms 50,100,250 --exercise-direction-inversion --config-save-reboot-cycles 3 --host-timeout-ms 250
```

Move to overnight only after the 30-60 minute run produces `report.ok = true`, clean final state, no UART failures, and config restoration. Do not raise claimed max velocity/current without attaching the passing report that proves it.

## DAFT Motion Binary v2

Raw packet before COBS:

| Field | Type |
| --- | --- |
| version | `u8`, currently `2` |
| msg_id | `u8` |
| flags | `u8` |
| seq | `u16` little-endian |
| payload_len | `u16` little-endian |
| payload | `0..128` bytes |
| crc16 | CRC-16/CCITT-FALSE over header + payload |

The raw packet is COBS encoded and terminated by `0x00`.

Core messages include `PING/PONG`, `GET_IDENTITY`, `GET_CAPABILITIES`, `GET_STATUS`, `GET_FAULTS`, `GET_COUNTERS`, `HEARTBEAT`, `SET_CONTROL_MODE`, `ESTOP`, `RAMP_STOP`, `CLEAR_FAULTS`, config query/stage/apply/save/reset, `GET_DRIVER_STATUS`, `MOVE_ABS`, `MOVE_REL`, `RUN_VEL`, `SET_TELEM_RATE`, and unsolicited `EVENT` packets.

Actuator states are explicit: `BOOTING`, `DISABLED`, `IDLE`, `MOVING_POSITION`, `RUNNING_VELOCITY`, `STOPPING`, `FAULTED`, `CONFIG_STAGED`, and `COMMISSIONING`.

Configuration changes are staged first, then applied, then optionally saved. ESP32 persistence uses two NVS slots with config version, payload length, CRC, generation counter, and a valid marker written last. `enable_timeout_ms`, position/home offsets, telemetry `enabled_fields`, and soft-limit origin are reserved in V1; they are persisted placeholders, not supported runtime semantics.

EVENT packets are emitted for boot reason, state transitions, fault set/clear, config apply/save, driver UART failures, and host timeout.

## V1 Acceptance Criteria

- Supported hardware: one ESP32-C6 board controlling one TMC2209 stepper driver over STEP/DIR plus UART.
- Required commands: identity, capabilities, status, faults, counters, heartbeat, mode, ramp stop, emergency stop, clear faults, move absolute/relative, run velocity, telemetry rate, config query/stage/apply/save/reset, reboot, and driver status.
- Safety behavior: invalid frames/commands are rejected, faults are explicit, velocity mode trips host timeout, controlled stop and emergency stop both leave inspectable state, and soft limits fault before issuing out-of-range motion.
- Persistence behavior: config changes are staged, validated, applied, saved only on command, restored after reboot, and recover from one corrupt/partial slot.
- Test evidence: Python tests, CMake/CTest, PlatformIO build, protocol golden tests, persistence simulator tests, and a passing hardware bench report.
- Max tested velocity/current: claim only the maximum values recorded in the latest passing `.report.json`; the baseline bench uses 1000 steps/s and the default 800 mA run current.
- Known non-goals: multi-axis, homing, encoders, fieldbus, closed-loop control, queued trajectories, and firmware update transport.
