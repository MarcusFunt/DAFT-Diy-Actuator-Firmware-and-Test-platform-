from __future__ import annotations

import argparse
import json
import sys

from daft_protocol import CONFIG_FIELDS, ControlMode, DaftClient, SerialTransport, field_by_name, list_serial_ports


def print_json(value: object) -> None:
    print(json.dumps(value, indent=2, sort_keys=True))


def with_client(port: str, fn):
    with SerialTransport(port) as transport:
        client = DaftClient(transport)
        return fn(client)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="daft", description="DAFT Binary v2 CLI")
    sub = parser.add_subparsers(dest="cmd", required=True)
    sub.add_parser("scan")

    for name in ("ping", "identity", "status", "faults", "counters", "driver-status", "stop", "estop", "clear-faults"):
        p = sub.add_parser(name)
        p.add_argument("port")

    p = sub.add_parser("mode")
    p.add_argument("port")
    p.add_argument("mode", choices=("disabled", "position", "velocity", "commissioning"))

    p = sub.add_parser("move-rel")
    p.add_argument("port")
    p.add_argument("--steps", type=int, required=True)
    p.add_argument("--speed", type=int, required=True)
    p.add_argument("--accel", type=int, required=True)

    p = sub.add_parser("move-abs")
    p.add_argument("port")
    p.add_argument("--target", type=int, required=True)
    p.add_argument("--speed", type=int, required=True)
    p.add_argument("--accel", type=int, required=True)

    p = sub.add_parser("run-vel")
    p.add_argument("port")
    p.add_argument("--speed", type=int, required=True)
    p.add_argument("--accel", type=int, required=True)

    p = sub.add_parser("telemetry")
    p.add_argument("port")
    p.add_argument("--interval-ms", type=int, required=True)

    config = sub.add_parser("config")
    config_sub = config.add_subparsers(dest="config_cmd", required=True)
    p = config_sub.add_parser("fields")
    p = config_sub.add_parser("get")
    p.add_argument("port")
    p.add_argument("--field", default="all")
    p = config_sub.add_parser("stage")
    p.add_argument("port")
    p.add_argument("field")
    p.add_argument("value", type=int)
    for name in ("apply", "save", "reset"):
        p = config_sub.add_parser(name)
        p.add_argument("port")

    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    if args.cmd == "scan":
        print_json({"ports": list_serial_ports()})
        return 0

    if args.cmd == "ping":
        print_json(with_client(args.port, lambda c: {"ok": c.ping()}))
    elif args.cmd == "identity":
        print_json(with_client(args.port, lambda c: c.identity()))
    elif args.cmd == "status":
        print_json(with_client(args.port, lambda c: c.status()))
    elif args.cmd == "faults":
        print_json(with_client(args.port, lambda c: c.faults()))
    elif args.cmd == "counters":
        print_json(with_client(args.port, lambda c: c.counters()))
    elif args.cmd == "driver-status":
        print_json(with_client(args.port, lambda c: c.driver_status()))
    elif args.cmd == "stop":
        print_json(with_client(args.port, lambda c: c.ramp_stop(1500)))
    elif args.cmd == "estop":
        print_json(with_client(args.port, lambda c: c.estop()))
    elif args.cmd == "clear-faults":
        print_json(with_client(args.port, lambda c: c.clear_faults()))
    elif args.cmd == "mode":
        modes = {
            "disabled": ControlMode.DISABLED,
            "position": ControlMode.POSITION,
            "velocity": ControlMode.VELOCITY,
            "commissioning": ControlMode.COMMISSIONING,
        }
        print_json(with_client(args.port, lambda c: c.set_mode(modes[args.mode])))
    elif args.cmd == "move-rel":
        print_json(with_client(args.port, lambda c: c.move_rel(args.steps, args.speed, args.accel)))
    elif args.cmd == "move-abs":
        print_json(with_client(args.port, lambda c: c.move_abs(args.target, args.speed, args.accel)))
    elif args.cmd == "run-vel":
        print_json(with_client(args.port, lambda c: c.run_velocity(args.speed, args.accel)))
    elif args.cmd == "telemetry":
        print_json(with_client(args.port, lambda c: c.set_telemetry_rate(args.interval_ms)))
    elif args.cmd == "config":
        if args.config_cmd == "fields":
            print_json([info.__dict__ | {"field": int(info.field)} for info in CONFIG_FIELDS])
        elif args.config_cmd == "get":
            def get_config(client: DaftClient):
                if args.field == "all":
                    return [client.config_get(int(info.field)) for info in CONFIG_FIELDS]
                return client.config_get(int(field_by_name(args.field)))
            print_json(with_client(args.port, get_config))
        elif args.config_cmd == "stage":
            print_json(with_client(args.port, lambda c: c.config_stage(int(field_by_name(args.field)), args.value)))
        elif args.config_cmd == "apply":
            print_json(with_client(args.port, lambda c: c.config_apply()))
        elif args.config_cmd == "save":
            print_json(with_client(args.port, lambda c: c.config_save()))
        elif args.config_cmd == "reset":
            print_json(with_client(args.port, lambda c: c.config_reset()))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
