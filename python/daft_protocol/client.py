from __future__ import annotations

from dataclasses import dataclass, field

from .codec import Packet, ProtocolError
from .messages import (
    ControlMode,
    ErrorCode,
    MsgId,
    build_config_query,
    build_config_stage_field,
    build_move,
    build_ramp_stop,
    build_run_velocity,
    build_set_control_mode,
    build_telem_rate,
    parse_ack,
    parse_capabilities,
    parse_config_value,
    parse_counters,
    parse_driver_status,
    parse_error,
    parse_event,
    parse_faults,
    parse_identity,
    parse_status,
)


TELEMETRY_FLAG = 1 << 2


class DeviceError(RuntimeError):
    def __init__(self, packet: Packet):
        self.fields = parse_error(packet)
        super().__init__(f"{self.fields['rejected_msg']}: {self.fields['code']}")


@dataclass
class DaftClient:
    transport: object
    next_seq: int = 1
    telemetry: list[dict[str, object]] = field(default_factory=list)
    events: list[dict[str, object]] = field(default_factory=list)

    def _seq(self) -> int:
        seq = self.next_seq
        self.next_seq = (self.next_seq + 1) & 0xFFFF
        if self.next_seq == 0:
            self.next_seq = 1
        return seq

    def request(self, msg: MsgId | int, payload: bytes = b"", expect: MsgId | int | None = None) -> Packet:
        seq = self._seq()
        self.transport.write_packet(int(msg), seq, payload)
        expected = int(expect) if expect is not None else None

        while True:
            packet = self.transport.read_packet()
            if packet.msg_id == int(MsgId.STATUS) and (packet.flags & TELEMETRY_FLAG):
                self.telemetry.append(parse_status(packet))
                continue
            if packet.msg_id == int(MsgId.EVENT):
                self.events.append(parse_event(packet))
                continue
            if packet.seq != seq:
                continue
            if packet.msg_id == int(MsgId.ERROR):
                raise DeviceError(packet)
            if expected is None:
                return packet
            if packet.msg_id == expected:
                return packet
            raise ProtocolError(f"unexpected response 0x{packet.msg_id:02X} for seq {seq}")

    def acked(self, msg: MsgId | int, payload: bytes = b"") -> dict[str, object]:
        packet = self.request(msg, payload, MsgId.ACK)
        return parse_ack(packet)

    def ping(self) -> bool:
        seq = self._seq()
        self.transport.write_packet(int(MsgId.PING), seq, b"")
        saw_ack = False
        while True:
            packet = self.transport.read_packet()
            if packet.msg_id == int(MsgId.STATUS) and (packet.flags & TELEMETRY_FLAG):
                self.telemetry.append(parse_status(packet))
                continue
            if packet.msg_id == int(MsgId.EVENT):
                self.events.append(parse_event(packet))
                continue
            if packet.seq != seq:
                if packet.msg_id == int(MsgId.STATUS):
                    self.telemetry.append(parse_status(packet))
                continue
            if packet.msg_id == int(MsgId.ERROR):
                raise DeviceError(packet)
            if packet.msg_id == int(MsgId.ACK):
                saw_ack = True
                continue
            if packet.msg_id == int(MsgId.PONG):
                return saw_ack

    def identity(self) -> dict[str, int | str | bool]:
        return parse_identity(self.request(MsgId.GET_IDENTITY, expect=MsgId.IDENTITY))

    def capabilities(self) -> dict[str, int | bool]:
        return parse_capabilities(self.request(MsgId.GET_CAPABILITIES, expect=MsgId.CAPABILITIES))

    def status(self) -> dict[str, int | str | bool]:
        return parse_status(self.request(MsgId.GET_STATUS, expect=MsgId.STATUS))

    def faults(self) -> dict[str, int]:
        return parse_faults(self.request(MsgId.GET_FAULTS, expect=MsgId.FAULTS))

    def counters(self) -> dict[str, int]:
        return parse_counters(self.request(MsgId.GET_COUNTERS, expect=MsgId.COUNTERS))

    def driver_status(self) -> dict[str, int | bool]:
        return parse_driver_status(self.request(MsgId.GET_DRIVER_STATUS, expect=MsgId.DRIVER_STATUS))

    def set_mode(self, mode: ControlMode | int) -> dict[str, object]:
        return self.acked(MsgId.SET_CONTROL_MODE, build_set_control_mode(mode))

    def set_telemetry_rate(self, interval_ms: int) -> dict[str, object]:
        return self.acked(MsgId.SET_TELEM_RATE, build_telem_rate(interval_ms))

    def ramp_stop(self, decel_sps2: int) -> dict[str, object]:
        return self.acked(MsgId.RAMP_STOP, build_ramp_stop(decel_sps2))

    def estop(self) -> dict[str, object]:
        return self.acked(MsgId.ESTOP)

    def clear_faults(self) -> dict[str, object]:
        return self.acked(MsgId.CLEAR_FAULTS)

    def move_abs(self, target_steps: int, velocity_sps: int, accel_sps2: int) -> dict[str, object]:
        return self.acked(MsgId.MOVE_ABS, build_move(target_steps, velocity_sps, accel_sps2))

    def move_rel(self, delta_steps: int, velocity_sps: int, accel_sps2: int) -> dict[str, object]:
        return self.acked(MsgId.MOVE_REL, build_move(delta_steps, velocity_sps, accel_sps2))

    def run_velocity(self, velocity_sps: int, accel_sps2: int) -> dict[str, object]:
        return self.acked(MsgId.RUN_VEL, build_run_velocity(velocity_sps, accel_sps2))

    def config_get(self, field_id: int) -> dict[str, int | str | bool]:
        return parse_config_value(self.request(MsgId.CONFIG_QUERY, build_config_query(field_id), MsgId.CONFIG_VALUE))

    def config_stage(self, field_id: int, value: int) -> dict[str, object]:
        return self.acked(MsgId.CONFIG_STAGE_FIELD, build_config_stage_field(field_id, value))

    def config_apply(self) -> dict[str, object]:
        return self.acked(MsgId.CONFIG_APPLY)

    def config_save(self) -> dict[str, object]:
        return self.acked(MsgId.CONFIG_SAVE)

    def config_reset(self) -> dict[str, object]:
        return self.acked(MsgId.CONFIG_RESET_DEFAULTS)
