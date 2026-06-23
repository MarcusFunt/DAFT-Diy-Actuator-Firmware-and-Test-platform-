# DAFT V1 Actuator Firmware

DAFT is now structured as reusable actuator firmware instead of an Arduino IDE sketch. V1 targets one locally planned TMC2209 stepper axis on ESP32-C6. Hosts send mode, target, configuration, and commissioning commands over DAFT Motion Binary v2; the actuator owns pulse timing, driver state, safety limits, persistence, telemetry, and faults.

Out of V1: multi-axis, encoders, homing, CAN/CAN-FD, STM32 HAL backend, ROS2, trajectory queues, closed-loop control, and firmware updates.

## Layout

```text
firmware/
  include/daft/              Platform-neutral C++ API and protocol types
  src/                       Protocol codec, config validation, state machine
  platforms/esp32_arduino/   PlatformIO ESP32-C6 backend
  tests/                     Host-side C++ tests
  platformio.ini             PlatformIO firmware build
  CMakeLists.txt             Host test build

python/
  daft_protocol/             Binary v2 codec, serial transport, client, config metadata
  apps/cli.py                CLI smoke and commissioning tool
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

The ESP32-C6 environment uses the Arduino framework through PlatformIO because the current hardware backend depends on FastAccelStepper and TMCStepper. The project is no longer an Arduino IDE sketch; the old `arduino/` sketch entrypoint has been removed.

Current pins are defined in `firmware/include/daft/config.hpp`:

```cpp
step=20, dir=19, enable=18, tmc_uart_rx=17, tmc_uart_tx=16
```

`enable` is active low. TMC2209 UART wiring remains:

```text
ESP32-C6 GPIO16 TX -> 1 kOhm -> TMC2209 PDN_UART
ESP32-C6 GPIO17 RX -----------> TMC2209 PDN_UART
GND --------------------------> TMC2209 GND
```

Note: this machine’s stock PlatformIO `espressif32@6.10.0` package does not support Arduino on ESP32-C6. `firmware/platformio.ini` pins the ESP32-C6 environment to `pioarduino/platform-espressif32`, which supports Arduino core 3.x for ESP32-C6.

## Host Tests

C++ core tests:

```powershell
cmake -G "MinGW Makefiles" -S firmware -B firmware\build-mingw
cmake --build firmware\build-mingw
ctest --test-dir firmware\build-mingw --output-on-failure
```

Python protocol tests:

```powershell
$env:PYTHONPATH='python'
python -m pytest -q python\tests
```

## Python Tools

Install dependencies:

```powershell
python -m pip install -r python\requirements.txt
```

Run the CLI:

```powershell
$env:PYTHONPATH='python'
python python\apps\cli.py scan
python python\apps\cli.py ping COM5
python python\apps\cli.py identity COM5
python python\apps\cli.py status COM5
python python\apps\cli.py mode COM5 position
python python\apps\cli.py move-rel COM5 --steps 1000 --speed 2000 --accel 5000
python python\apps\cli.py stop COM5
python python\apps\cli.py config get COM5 --field all
python python\apps\cli.py config stage COM5 run_current_ma 700
python python\apps\cli.py config apply COM5
python python\apps\cli.py config save COM5
```

Run the DearPyGUI commissioning app:

```powershell
$env:PYTHONPATH='python'
python python\apps\dpg_commissioning_gui.py
```

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

Core messages include `PING/PONG`, `GET_IDENTITY`, `GET_CAPABILITIES`, `GET_STATUS`, `GET_FAULTS`, `GET_COUNTERS`, `HEARTBEAT`, `SET_CONTROL_MODE`, `ESTOP`, `RAMP_STOP`, `CLEAR_FAULTS`, config query/stage/apply/save/reset, `GET_DRIVER_STATUS`, `MOVE_ABS`, `MOVE_REL`, `RUN_VEL`, and `SET_TELEM_RATE`.

Actuator states are explicit: `BOOTING`, `DISABLED`, `IDLE`, `MOVING_POSITION`, `RUNNING_VELOCITY`, `STOPPING`, `FAULTED`, `CONFIG_STAGED`, and `COMMISSIONING`.

Configuration changes are staged first, then applied, then optionally saved. ESP32 persistence uses two NVS slots with config version, payload length, CRC, generation counter, and a valid marker written last.
