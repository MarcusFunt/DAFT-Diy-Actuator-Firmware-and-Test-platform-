from __future__ import annotations

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
    timeout: float = 0.2

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
            write_timeout=self.timeout,
            inter_byte_timeout=0.02,
        )
        self._rx.clear()

    def close(self) -> None:
        if self._port is not None:
            self._port.close()
        self._port = None

    def write_packet(self, msg_id: int, seq: int, payload: bytes = b"", flags: int = 0) -> None:
        if self._port is None:
            raise RuntimeError("serial port is not open")
        self._port.write(encode_packet(msg_id, seq, payload, flags))

    def read_packet(self, timeout_packets: int = 256) -> Packet:
        if self._port is None:
            raise RuntimeError("serial port is not open")

        empty_reads = 0
        while empty_reads < timeout_packets:
            chunk = self._port.read(128)
            if not chunk:
                empty_reads += 1
                continue
            self._rx.extend(chunk)
            while True:
                try:
                    delimiter = self._rx.index(0)
                except ValueError:
                    if len(self._rx) > 256:
                        self._rx.clear()
                        raise ProtocolError("RX buffer overflow")
                    break

                frame = bytes(self._rx[:delimiter])
                del self._rx[: delimiter + 1]
                if not frame:
                    continue
                return decode_packet(frame)

        raise TimeoutError("timed out waiting for packet")
