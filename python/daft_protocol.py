from __future__ import annotations

import struct
from dataclasses import dataclass


VERSION = 1
RAW_PACKET_SIZE = 32
PAYLOAD_SIZE = 23
CRC_OFFSET = 30

MSG_PING = 0x01
MSG_GET_STATUS = 0x02
MSG_SET_TELEM_RATE = 0x03

MSG_ENABLE = 0x10
MSG_DISABLE = 0x11
MSG_ESTOP = 0x12
MSG_STOP_RAMP = 0x13
MSG_ZERO_POSITION = 0x14

MSG_MOVE_ABS = 0x20
MSG_MOVE_REL = 0x21
MSG_RUN_VEL = 0x22
MSG_SET_SOFT_LIMITS = 0x23

MSG_DRIVER_CONFIG = 0x30
MSG_DRIVER_QUERY = 0x31

MSG_CLEAR_FAULTS = 0x40

MSG_ACK = 0x80
MSG_NACK = 0x81
MSG_STATUS = 0x82
MSG_EVENT = 0x83
MSG_DRIVER_STATUS = 0x84

DRIVER_MODE_STEALTHCHOP = 0
DRIVER_MODE_SPREADCYCLE = 1
SUPPORTED_MICROSTEPS = (1, 2, 4, 8, 16, 32, 64, 128, 256)

NACK_REASONS = {
    0: "no error",
    1: "bad CRC",
    2: "bad version",
    3: "unknown command",
    4: "bad payload length",
    5: "value out of range",
    6: "soft limit violation",
    7: "busy",
    8: "driver disabled",
    9: "fault latched",
    10: "queue full",
    11: "duplicate sequence mismatch",
}

COMMAND_NAMES = {
    MSG_PING: "PING",
    MSG_GET_STATUS: "GET_STATUS",
    MSG_SET_TELEM_RATE: "SET_TELEM_RATE",
    MSG_ENABLE: "ENABLE",
    MSG_DISABLE: "DISABLE",
    MSG_ESTOP: "ESTOP",
    MSG_STOP_RAMP: "STOP_RAMP",
    MSG_ZERO_POSITION: "ZERO_POSITION",
    MSG_MOVE_ABS: "MOVE_ABS",
    MSG_MOVE_REL: "MOVE_REL",
    MSG_RUN_VEL: "RUN_VEL",
    MSG_SET_SOFT_LIMITS: "SET_SOFT_LIMITS",
    MSG_DRIVER_CONFIG: "DRIVER_CONFIG",
    MSG_DRIVER_QUERY: "DRIVER_QUERY",
    MSG_CLEAR_FAULTS: "CLEAR_FAULTS",
    MSG_ACK: "ACK",
    MSG_NACK: "NACK",
    MSG_STATUS: "STATUS",
    MSG_EVENT: "EVENT",
    MSG_DRIVER_STATUS: "DRIVER_STATUS",
}

MOTION_STATES = {
    0: "idle",
    1: "moving",
    2: "velocity",
    3: "stopping",
    4: "fault",
}

EVENT_NAMES = {
    1: "move done",
    2: "limit hit",
    3: "estop",
    4: "fault",
}


class ProtocolError(ValueError):
    pass


@dataclass(frozen=True)
class Packet:
    msg_id: int
    flags: int
    seq: int
    payload: bytes


def command_name(msg_id: int) -> str:
    return COMMAND_NAMES.get(msg_id, f"0x{msg_id:02X}")


def reason_name(reason: int) -> str:
    return NACK_REASONS.get(reason, f"reason {reason}")


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def cobs_encode(data: bytes) -> bytes:
    out = bytearray()
    code_index = 0
    out.append(0)
    code = 1

    for byte in data:
        if byte == 0:
            out[code_index] = code
            code_index = len(out)
            out.append(0)
            code = 1
        else:
            out.append(byte)
            code += 1
            if code == 0xFF:
                out[code_index] = code
                code_index = len(out)
                out.append(0)
                code = 1

    out[code_index] = code
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    out = bytearray()
    index = 0

    while index < len(data):
        code = data[index]
        if code == 0:
            raise ProtocolError("zero byte inside COBS frame")
        index += 1

        block_end = index + code - 1
        if block_end > len(data):
            raise ProtocolError("truncated COBS block")

        out.extend(data[index:block_end])
        index = block_end

        if code < 0xFF and index < len(data):
            out.append(0)

    return bytes(out)


def encode_packet(msg_id: int, seq: int, payload: bytes = b"", flags: int = 0) -> bytes:
    if not 0 <= msg_id <= 0xFF:
        raise ValueError("msg_id must fit in u8")
    if not 0 <= flags <= 0xFF:
        raise ValueError("flags must fit in u8")
    if len(payload) > PAYLOAD_SIZE:
        raise ValueError(f"payload is limited to {PAYLOAD_SIZE} bytes")

    raw = bytearray(RAW_PACKET_SIZE)
    struct.pack_into("<BBBBH", raw, 0, VERSION, msg_id, flags, len(payload), seq & 0xFFFF)
    raw[6 : 6 + len(payload)] = payload
    crc = crc16_ccitt(bytes(raw[:CRC_OFFSET]))
    struct.pack_into("<H", raw, CRC_OFFSET, crc)
    return cobs_encode(bytes(raw)) + b"\x00"


def decode_packet(frame: bytes) -> Packet:
    raw = cobs_decode(frame)
    if len(raw) != RAW_PACKET_SIZE:
        raise ProtocolError(f"decoded packet has {len(raw)} bytes, expected {RAW_PACKET_SIZE}")

    expected_crc = struct.unpack_from("<H", raw, CRC_OFFSET)[0]
    actual_crc = crc16_ccitt(raw[:CRC_OFFSET])
    if actual_crc != expected_crc:
        raise ProtocolError("bad CRC")

    version, msg_id, flags, payload_len, seq = struct.unpack_from("<BBBBH", raw, 0)
    if version != VERSION:
        raise ProtocolError(f"unsupported protocol version {version}")
    if payload_len > PAYLOAD_SIZE:
        raise ProtocolError("payload length exceeds packet payload area")

    return Packet(msg_id=msg_id, flags=flags, seq=seq, payload=bytes(raw[6 : 6 + payload_len]))


def _require_range(name: str, value: int, low: int, high: int) -> None:
    if not low <= value <= high:
        raise ValueError(f"{name} must be in range {low}..{high}")


def payload_move(target_steps: int, max_speed_sps: int, accel_sps2: int, axis: int = 0, apply_tick: int = 0, move_flags: int = 0) -> bytes:
    _require_range("axis", axis, 0, 255)
    _require_range("target_steps", target_steps, -(2**31), 2**31 - 1)
    _require_range("max_speed_sps", max_speed_sps, 1, 2**32 - 1)
    _require_range("accel_sps2", accel_sps2, 1, 2**32 - 1)
    _require_range("apply_tick", apply_tick, 0, 2**32 - 1)

    payload = bytearray(PAYLOAD_SIZE)
    struct.pack_into("<BBH", payload, 0, axis, move_flags & 0xFF, 0)
    struct.pack_into("<iIII", payload, 4, target_steps, max_speed_sps, accel_sps2, apply_tick)
    return bytes(payload)


def payload_run_velocity(target_speed_sps: int, accel_sps2: int, axis: int = 0, apply_tick: int = 0, vel_flags: int = 0) -> bytes:
    _require_range("axis", axis, 0, 255)
    _require_range("target_speed_sps", target_speed_sps, -(2**31), 2**31 - 1)
    _require_range("accel_sps2", accel_sps2, 1, 2**32 - 1)
    _require_range("apply_tick", apply_tick, 0, 2**32 - 1)

    payload = bytearray(PAYLOAD_SIZE)
    struct.pack_into("<BBH", payload, 0, axis, vel_flags & 0xFF, 0)
    struct.pack_into("<iII", payload, 4, target_speed_sps, accel_sps2, apply_tick)
    return bytes(payload)


def payload_stop_ramp(decel_sps2: int, axis: int = 0, apply_tick: int = 0) -> bytes:
    _require_range("axis", axis, 0, 255)
    _require_range("decel_sps2", decel_sps2, 1, 2**32 - 1)
    _require_range("apply_tick", apply_tick, 0, 2**32 - 1)

    payload = bytearray(12)
    struct.pack_into("<BBHII", payload, 0, axis, 0, 0, decel_sps2, apply_tick)
    return bytes(payload)


def payload_set_soft_limits(enabled: bool, min_steps: int, max_steps: int, axis: int = 0) -> bytes:
    _require_range("axis", axis, 0, 255)
    _require_range("min_steps", min_steps, -(2**31), 2**31 - 1)
    _require_range("max_steps", max_steps, -(2**31), 2**31 - 1)
    if enabled and min_steps > max_steps:
        raise ValueError("min_steps must be less than or equal to max_steps when soft limits are enabled")

    payload = bytearray(12)
    flags = 0x01 if enabled else 0x00
    struct.pack_into("<BBHii", payload, 0, axis, flags, 0, min_steps, max_steps)
    return bytes(payload)


def payload_driver_config(
    run_current_ma: int,
    microsteps: int,
    hold_current_ma: int = 0,
    driver_mode: int = DRIVER_MODE_STEALTHCHOP,
    flags: int = 0,
    axis: int = 0,
) -> bytes:
    _require_range("axis", axis, 0, 255)
    _require_range("microsteps", microsteps, 1, 255)
    _require_range("driver_mode", driver_mode, 0, 1)
    _require_range("flags", flags, 0, 255)
    _require_range("run_current_ma", run_current_ma, 1, 65535)
    _require_range("hold_current_ma", hold_current_ma, 0, 65535)

    payload = bytearray(PAYLOAD_SIZE)
    struct.pack_into(
        "<BBBBHHH",
        payload,
        0,
        axis,
        microsteps,
        driver_mode,
        flags,
        run_current_ma,
        hold_current_ma,
        0,
    )
    return bytes(payload)


def payload_telem_rate(interval_ms: int) -> bytes:
    _require_range("interval_ms", interval_ms, 0, 65535)
    return struct.pack("<H", interval_ms)


def parse_ack(packet: Packet) -> tuple[int, int]:
    if len(packet.payload) < 2:
        raise ProtocolError("short ACK payload")
    return packet.payload[0], packet.payload[1]


def parse_nack(packet: Packet) -> tuple[int, int]:
    if len(packet.payload) < 2:
        raise ProtocolError("short NACK payload")
    return packet.payload[0], packet.payload[1]


def parse_status(packet: Packet) -> dict[str, int | str]:
    if len(packet.payload) != PAYLOAD_SIZE:
        raise ProtocolError("STATUS payload must be 23 bytes")

    axis, motion_state, fault_flags, position, target, speed, esp_tick_ms, rx_free, enabled, _ = struct.unpack(
        "<BBHiiiIBBB", packet.payload
    )
    return {
        "axis": axis,
        "motion_state": motion_state,
        "motion_state_name": MOTION_STATES.get(motion_state, f"state {motion_state}"),
        "fault_flags": fault_flags,
        "position_steps": position,
        "target_steps": target,
        "speed_sps": speed,
        "esp_tick_ms": esp_tick_ms,
        "rx_queue_free": rx_free,
        "driver_enabled": enabled,
    }


def parse_event(packet: Packet) -> tuple[int, int]:
    if len(packet.payload) < 2:
        raise ProtocolError("short EVENT payload")
    return packet.payload[0], packet.payload[1]
