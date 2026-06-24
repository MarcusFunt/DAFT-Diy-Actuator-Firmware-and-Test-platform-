from __future__ import annotations

import random
import struct

import pytest

from daft_protocol.codec import ProtocolError, decode_packet, encode_packet
from daft_protocol.messages import MsgId, build_move, parse_status
from daft_protocol.transport_serial import SerialTransport


class FakePort:
    def __init__(self, chunks: list[bytes]):
        self.chunks = chunks
        self.is_open = True

    def read(self, size: int) -> bytes:
        return self.chunks.pop(0) if self.chunks else b""


def test_round_trip_packet() -> None:
    payload = build_move(1234, 2000, 3000)
    frame = encode_packet(MsgId.MOVE_ABS, 42, payload)
    assert frame.endswith(b"\x00")
    packet = decode_packet(frame[:-1])
    assert packet.msg_id == MsgId.MOVE_ABS
    assert packet.seq == 42
    assert packet.payload == payload


def test_crc_rejects_corruption() -> None:
    frame = bytearray(encode_packet(MsgId.PING, 1)[:-1])
    frame[2] ^= 0x55
    with pytest.raises(ProtocolError):
        decode_packet(bytes(frame))


def test_random_frames_do_not_crash() -> None:
    for _ in range(200):
        data = bytes(random.randrange(1, 256) for _ in range(random.randrange(1, 80)))
        try:
            decode_packet(data)
        except ProtocolError:
            pass


def test_parse_status_payload() -> None:
    payload = struct.pack("<BBBBIiiiIII", 2, 1, 1, 0, 0, 10, 20, 30, 1000, 3, 9)
    packet = type("PacketLike", (), {"payload": payload})()
    fields = parse_status(packet)
    assert fields["state"] == "IDLE"
    assert fields["control_mode"] == "POSITION"
    assert fields["position_steps"] == 10


def test_serial_transport_drains_back_to_back_packets() -> None:
    transport = SerialTransport("loopback")
    transport._port = FakePort(
        [
            encode_packet(MsgId.ACK, 7, b"\x01\x00")
            + encode_packet(MsgId.PONG, 7),
        ]
    )

    first = transport.read_packet(timeout=0.1)
    second = transport.read_packet(timeout=0.1)

    assert first.msg_id == MsgId.ACK
    assert second.msg_id == MsgId.PONG
    assert second.seq == 7


def test_serial_transport_recovers_after_corrupted_frame() -> None:
    transport = SerialTransport("loopback")
    transport._port = FakePort(
        [
            b"\x05\x01\x00" + encode_packet(MsgId.PONG, 9),
        ]
    )

    packet = transport.read_packet(timeout=0.1)

    assert packet.msg_id == MsgId.PONG
    assert packet.seq == 9
