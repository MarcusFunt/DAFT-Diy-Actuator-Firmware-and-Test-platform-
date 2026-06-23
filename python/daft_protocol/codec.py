from __future__ import annotations

import struct
from dataclasses import dataclass


VERSION = 2
MAX_PAYLOAD_SIZE = 128
HEADER_FORMAT = "<BBBHH"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)


class ProtocolError(ValueError):
    pass


@dataclass(frozen=True)
class Packet:
    msg_id: int
    flags: int = 0
    seq: int = 0
    payload: bytes = b""


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
    out = bytearray([0])
    code_index = 0
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
    if not 0 <= seq <= 0xFFFF:
        raise ValueError("seq must fit in u16")
    if len(payload) > MAX_PAYLOAD_SIZE:
        raise ValueError(f"payload exceeds {MAX_PAYLOAD_SIZE} bytes")

    raw = struct.pack(HEADER_FORMAT, VERSION, msg_id, flags, seq, len(payload)) + payload
    raw += struct.pack("<H", crc16_ccitt(raw))
    return cobs_encode(raw) + b"\x00"


def decode_packet(frame: bytes) -> Packet:
    raw = cobs_decode(frame)
    if len(raw) < HEADER_SIZE + 2:
        raise ProtocolError("short packet")

    version, msg_id, flags, seq, payload_len = struct.unpack_from(HEADER_FORMAT, raw, 0)
    if version != VERSION:
        raise ProtocolError(f"unsupported protocol version {version}")
    if payload_len > MAX_PAYLOAD_SIZE:
        raise ProtocolError("payload too large")
    if len(raw) != HEADER_SIZE + payload_len + 2:
        raise ProtocolError("packet length mismatch")

    expected = struct.unpack_from("<H", raw, HEADER_SIZE + payload_len)[0]
    actual = crc16_ccitt(raw[: HEADER_SIZE + payload_len])
    if actual != expected:
        raise ProtocolError("bad CRC")

    return Packet(msg_id=msg_id, flags=flags, seq=seq, payload=raw[HEADER_SIZE : HEADER_SIZE + payload_len])
