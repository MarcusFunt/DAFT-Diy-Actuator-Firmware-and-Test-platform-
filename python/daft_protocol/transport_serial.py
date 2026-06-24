from __future__ import annotations

import time
from dataclasses import dataclass

import serial
from serial.tools import list_ports

from .codec import Packet, ProtocolError, decode_packet, encode_packet


DEFAULT_BAUD = 115200


def list_serial_ports() -> list[str]:
    return [port.device for port in list_ports.comports()]


@dataclass
class SerialTransport:
    port_name: str
    baud_rate: int = DEFAULT_BAUD
    timeout: float = 0.05
    write_timeout: float = 0.2
    open_delay: float = 0.6
    packet_timeout: float = 2.0

    def __post_init__(self) -> None:
        self._port: serial.Serial | None = None
        self._rx = bytearray()

    def __enter__(self) -> "SerialTransport":
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    @property
    def is_open(self) -> bool:
        return self._port is not None and self._port.is_open

    def open(self) -> None:
        self.close()
        self._port = serial.Serial(
            self.port_name,
            self.baud_rate,
            timeout=self.timeout,
            write_timeout=self.write_timeout,
            inter_byte_timeout=0.02,
        )
        self._rx.clear()
        if self.open_delay > 0:
            time.sleep(self.open_delay)
        self._port.reset_input_buffer()
        self._port.reset_output_buffer()

    def close(self) -> None:
        if self._port is not None:
            self._port.close()
        self._port = None

    def write_packet(self, msg_id: int, seq: int, payload: bytes = b"", flags: int = 0) -> None:
        if self._port is None:
            raise RuntimeError("serial port is not open")
        self._port.write(encode_packet(msg_id, seq, payload, flags))

    def _read_buffered_packet(self) -> Packet | None:
        while True:
            try:
                delimiter = self._rx.index(0)
            except ValueError:
                if len(self._rx) > 256:
                    self._rx.clear()
                    raise ProtocolError("RX buffer overflow")
                return None

            frame = bytes(self._rx[:delimiter])
            del self._rx[: delimiter + 1]
            if not frame:
                continue
            return decode_packet(frame)

    def read_packet(self, timeout: float | None = None) -> Packet:
        if self._port is None:
            raise RuntimeError("serial port is not open")

        deadline = time.monotonic() + (self.packet_timeout if timeout is None else timeout)
        while time.monotonic() < deadline:
            try:
                packet = self._read_buffered_packet()
            except ProtocolError:
                continue
            if packet is not None:
                return packet

            chunk = self._port.read(128)
            if not chunk:
                continue
            self._rx.extend(chunk)

        raise TimeoutError("timed out waiting for packet")
