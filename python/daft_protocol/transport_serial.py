from __future__ import annotations

import time
from dataclasses import dataclass

import serial
from serial.tools import list_ports

from .codec import Packet, ProtocolError, decode_packet, encode_packet


DEFAULT_BAUD = 115200


def list_serial_ports() -> list[str]:
    return [port.device for port in list_ports.comports()]


@dataclass(frozen=True)
class SerialTransportStats:
    packets_read: int = 0
    bytes_read: int = 0
    protocol_errors: int = 0
    rx_overflows: int = 0
    timeouts: int = 0
    last_protocol_error: str | None = None


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
        self._packets_read = 0
        self._bytes_read = 0
        self._protocol_errors = 0
        self._rx_overflows = 0
        self._timeouts = 0
        self._last_protocol_error: str | None = None

    def __enter__(self) -> "SerialTransport":
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    @property
    def is_open(self) -> bool:
        return self._port is not None and self._port.is_open

    @property
    def stats(self) -> SerialTransportStats:
        return SerialTransportStats(
            packets_read=self._packets_read,
            bytes_read=self._bytes_read,
            protocol_errors=self._protocol_errors,
            rx_overflows=self._rx_overflows,
            timeouts=self._timeouts,
            last_protocol_error=self._last_protocol_error,
        )

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
                    self._rx_overflows += 1
                    raise ProtocolError("RX buffer overflow")
                return None

            frame = bytes(self._rx[:delimiter])
            del self._rx[: delimiter + 1]
            if not frame:
                continue
            packet = decode_packet(frame)
            self._packets_read += 1
            return packet

    def read_packet(self, timeout: float | None = None) -> Packet:
        if self._port is None:
            raise RuntimeError("serial port is not open")

        deadline = time.monotonic() + (self.packet_timeout if timeout is None else timeout)
        while time.monotonic() < deadline:
            try:
                packet = self._read_buffered_packet()
            except ProtocolError as exc:
                self._protocol_errors += 1
                self._last_protocol_error = str(exc)
                continue
            if packet is not None:
                return packet

            chunk = self._port.read(128)
            if not chunk:
                continue
            self._bytes_read += len(chunk)
            self._rx.extend(chunk)

        self._timeouts += 1
        raise TimeoutError("timed out waiting for packet")
