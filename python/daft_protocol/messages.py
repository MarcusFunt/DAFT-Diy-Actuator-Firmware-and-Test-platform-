from __future__ import annotations

import struct
from enum import IntEnum

from .codec import Packet, ProtocolError


class MsgId(IntEnum):
    PING = 0x01
    PONG = 0x02
    GET_IDENTITY = 0x03
    IDENTITY = 0x04
    GET_CAPABILITIES = 0x05
    CAPABILITIES = 0x06
    GET_STATUS = 0x07
    STATUS = 0x08
    GET_FAULTS = 0x09
    FAULTS = 0x0A
    GET_COUNTERS = 0x0B
    COUNTERS = 0x0C
    HEARTBEAT = 0x0D
    SET_CONTROL_MODE = 0x10
    ESTOP = 0x11
    RAMP_STOP = 0x12
    CLEAR_FAULTS = 0x13
    CONFIG_QUERY = 0x20
    CONFIG_VALUE = 0x21
    CONFIG_STAGE_FIELD = 0x22
    CONFIG_APPLY = 0x23
    CONFIG_SAVE = 0x24
    CONFIG_RESET_DEFAULTS = 0x25
    REBOOT = 0x26
    GET_DRIVER_STATUS = 0x30
    DRIVER_STATUS = 0x31
    MOVE_ABS = 0x40
    MOVE_REL = 0x41
    RUN_VEL = 0x42
    SET_TELEM_RATE = 0x50
    ACK = 0x80
    ERROR = 0x81
    EVENT = 0x82


class ErrorCode(IntEnum):
    OK = 0
    ERR_BUSY = 1
    ERR_INVALID_FIELD = 2
    ERR_OUT_OF_RANGE = 3
    ERR_REQUIRES_REBOOT = 4
    ERR_UNSUPPORTED = 5
    ERR_FAULTED = 6
    ERR_BAD_STATE = 7
    ERR_BAD_CRC = 8
    ERR_BAD_PAYLOAD = 9
    ERR_BAD_VERSION = 10
    ERR_DUPLICATE_MISMATCH = 11
    ERR_STORAGE = 12


class State(IntEnum):
    BOOTING = 0
    DISABLED = 1
    IDLE = 2
    MOVING_POSITION = 3
    RUNNING_VELOCITY = 4
    STOPPING = 5
    FAULTED = 6
    CONFIG_STAGED = 7
    COMMISSIONING = 8


class ControlMode(IntEnum):
    DISABLED = 0
    POSITION = 1
    VELOCITY = 2
    COMMISSIONING = 3


class ConfigField(IntEnum):
    TELEMETRY_INTERVAL_MS = 1
    SOFT_LIMITS_ENABLED = 2
    SOFT_LIMIT_MIN_STEPS = 3
    SOFT_LIMIT_MAX_STEPS = 4
    DEFAULT_MAX_VELOCITY_SPS = 5
    DEFAULT_MAX_ACCEL_SPS2 = 6
    DEFAULT_STOP_DECEL_SPS2 = 7
    RUN_CURRENT_MA = 8
    HOLD_CURRENT_MA = 9
    MICROSTEPS = 10
    DRIVER_MODE = 11
    POSITION_OFFSET_STEPS = 12
    HOME_POSITION_STEPS = 13
    DIRECTION_INVERTED = 14
    HOST_TIMEOUT_MS = 15
    VELOCITY_TIMEOUT_ACTION = 16
    ENABLE_TIMEOUT_MS = 17
    MAX_ALLOWED_CURRENT_MA = 18
    MAX_ALLOWED_VELOCITY_SPS = 19
    MAX_ALLOWED_ACCEL_SPS2 = 20


class Mutability(IntEnum):
    RUNTIME_SAFE = 0
    REQUIRES_IDLE = 1
    REQUIRES_DRIVER_REINIT = 2
    REQUIRES_REBOOT = 3
    COMPILE_TIME_ONLY = 4


def _require_len(packet: Packet, length: int, name: str) -> None:
    if len(packet.payload) != length:
        raise ProtocolError(f"{name} payload must be {length} bytes")


def build_set_control_mode(mode: ControlMode | int) -> bytes:
    return struct.pack("<B", int(mode))


def build_ramp_stop(decel_sps2: int) -> bytes:
    return struct.pack("<I", decel_sps2)


def build_move(target_or_delta_steps: int, velocity_sps: int, accel_sps2: int) -> bytes:
    return struct.pack("<iII", target_or_delta_steps, velocity_sps, accel_sps2)


def build_run_velocity(velocity_sps: int, accel_sps2: int) -> bytes:
    return struct.pack("<iI", velocity_sps, accel_sps2)


def build_telem_rate(interval_ms: int) -> bytes:
    return struct.pack("<H", interval_ms)


def build_config_query(field: ConfigField | int) -> bytes:
    return struct.pack("<H", int(field))


def build_config_stage_field(field: ConfigField | int, value: int) -> bytes:
    return struct.pack("<Hi", int(field), value)


def parse_ack(packet: Packet) -> dict[str, object]:
    _require_len(packet, 2, "ACK")
    return {"acked_msg": MsgId(packet.payload[0]).name, "code": ErrorCode(packet.payload[1]).name}


def parse_error(packet: Packet) -> dict[str, object]:
    _require_len(packet, 2, "ERROR")
    msg_id = packet.payload[0]
    code = packet.payload[1]
    return {
        "rejected_msg": MsgId(msg_id).name if msg_id in MsgId._value2member_map_ else f"0x{msg_id:02X}",
        "code": ErrorCode(code).name if code in ErrorCode._value2member_map_ else f"ERR_{code}",
    }


def parse_identity(packet: Packet) -> dict[str, int | str]:
    _require_len(packet, 16, "IDENTITY")
    proto, major, minor, patch, board, axes, config_version = struct.unpack_from("<BBBBBBH", packet.payload, 0)
    magic, generation = struct.unpack_from("<II", packet.payload, 8)
    return {
        "protocol_version": proto,
        "firmware": f"{major}.{minor}.{patch}",
        "board_family": board,
        "axis_count": axes,
        "config_version": config_version,
        "magic": f"0x{magic:08X}",
        "config_generation": generation,
    }


def parse_capabilities(packet: Packet) -> dict[str, int | bool]:
    _require_len(packet, 16, "CAPABILITIES")
    driver, homing, encoder, persistence = struct.unpack_from("<BBBB", packet.payload, 0)
    max_payload, max_telem_hz = struct.unpack_from("<HH", packet.payload, 4)
    features, host_timeout_ms = struct.unpack_from("<II", packet.payload, 8)
    return {
        "driver": driver,
        "homing": bool(homing),
        "encoder": bool(encoder),
        "persistence": bool(persistence),
        "max_payload": max_payload,
        "max_telemetry_hz": max_telem_hz,
        "features": features,
        "host_timeout_ms": host_timeout_ms,
    }


def parse_status(packet: Packet) -> dict[str, int | str | bool]:
    _require_len(packet, 32, "STATUS")
    state, mode, enabled, staged = struct.unpack_from("<BBBB", packet.payload, 0)
    faults, position, target, speed, uptime_ms, generation, command_count = struct.unpack_from(
        "<IiiiIII", packet.payload, 4
    )
    return {
        "state": State(state).name if state in State._value2member_map_ else f"STATE_{state}",
        "control_mode": ControlMode(mode).name if mode in ControlMode._value2member_map_ else f"MODE_{mode}",
        "driver_enabled": bool(enabled),
        "config_staged": bool(staged),
        "faults": faults,
        "position_steps": position,
        "target_steps": target,
        "speed_sps": speed,
        "uptime_ms": uptime_ms,
        "config_generation": generation,
        "command_count": command_count,
    }


def parse_faults(packet: Packet) -> dict[str, int]:
    _require_len(packet, 4, "FAULTS")
    return {"faults": struct.unpack("<I", packet.payload)[0]}


def parse_counters(packet: Packet) -> dict[str, int]:
    _require_len(packet, 28, "COUNTERS")
    keys = ("crc_errors", "decode_errors", "dropped_packets", "duplicate_packets", "command_count", "rejected_commands", "uptime_ms")
    return dict(zip(keys, struct.unpack("<IIIIIII", packet.payload), strict=True))


def parse_config_value(packet: Packet) -> dict[str, int | str | bool]:
    _require_len(packet, 8, "CONFIG_VALUE")
    field_id, mutability, staged, value = struct.unpack("<HBBi", packet.payload)
    return {
        "field_id": field_id,
        "field": ConfigField(field_id).name if field_id in ConfigField._value2member_map_ else f"FIELD_{field_id}",
        "mutability": Mutability(mutability).name if mutability in Mutability._value2member_map_ else f"MUT_{mutability}",
        "config_staged": bool(staged),
        "value": value,
    }


def parse_driver_status(packet: Packet) -> dict[str, int | bool]:
    _require_len(packet, 16, "DRIVER_STATUS")
    configured, uart_ok, otpw, ot, current, microsteps = struct.unpack_from("<BBBBHH", packet.payload, 0)
    raw_status = struct.unpack_from("<I", packet.payload, 8)[0]
    return {
        "configured": bool(configured),
        "uart_ok": bool(uart_ok),
        "overtemp_warning": bool(otpw),
        "overtemp": bool(ot),
        "run_current_ma": current,
        "microsteps": microsteps,
        "raw_status": raw_status,
    }
