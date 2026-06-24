from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import sys
import time

from daft_protocol import CONFIG_FIELDS, ConfigField, ControlMode, DaftClient, MsgId, SerialTransport, field_by_name, list_serial_ports


HOST_TIMEOUT_FAULT = 1 << 9


def print_json(value: object) -> None:
    print(json.dumps(value, indent=2, sort_keys=True))


def with_client(port: str, fn):
    with SerialTransport(port) as transport:
        client = DaftClient(transport)
        return fn(client)


def _bench_default_log() -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path("hardware_logs") / f"bench_{stamp}.jsonl"


def _write_jsonl(handle, event: str, **fields) -> None:
    payload = {"t": time.time(), "event": event, **fields}
    handle.write(json.dumps(payload, sort_keys=True) + "\n")
    handle.flush()


def _bench_open(port: str) -> tuple[SerialTransport, DaftClient]:
    transport = SerialTransport(port, packet_timeout=3.0)
    transport.open()
    return transport, DaftClient(transport)


def _bench_reopen(transport: SerialTransport, port: str, delay_s: float = 2.0) -> tuple[SerialTransport, DaftClient]:
    transport.close()
    time.sleep(delay_s)
    deadline = time.monotonic() + 10.0
    last_error: Exception | None = None
    while time.monotonic() < deadline:
        try:
            return _bench_open(port)
        except Exception as exc:
            last_error = exc
            time.sleep(0.5)
    raise RuntimeError(f"could not reopen {port} after reboot: {last_error}")


def _bench_wait_idle(client: DaftClient, log_handle, label: str, timeout_s: float) -> dict[str, int | str | bool]:
    deadline = time.monotonic() + timeout_s
    last_status: dict[str, int | str | bool] | None = None
    while time.monotonic() < deadline:
        last_status = client.status()
        _write_jsonl(log_handle, "status", label=label, status=last_status)
        if last_status["state"] in ("IDLE", "DISABLED", "FAULTED"):
            return last_status
        time.sleep(0.05)
    raise TimeoutError(f"{label} did not settle; last status={last_status}")


def run_bench(args) -> dict[str, object]:
    log_path = Path(args.log_jsonl) if args.log_jsonl else _bench_default_log()
    log_path.parent.mkdir(parents=True, exist_ok=True)

    transport: SerialTransport | None = None
    client: DaftClient | None = None
    original_host_timeout: int | None = None
    final_status: dict[str, object] | None = None

    with log_path.open("w", encoding="utf-8") as log_handle:
        try:
            transport, client = _bench_open(args.port)
            _write_jsonl(log_handle, "start", port=args.port, args=vars(args))

            identity = client.identity()
            status = client.status()
            driver = client.driver_status()
            counters = client.counters()
            faults = client.faults()
            _write_jsonl(log_handle, "snapshot", label="initial", identity=identity, status=status, driver=driver, counters=counters, faults=faults)
            if not driver["configured"] or not driver["uart_ok"]:
                raise RuntimeError(f"driver is not healthy: {driver}")
            if status["faults"]:
                client.clear_faults()

            original_host_timeout = int(client.config_get(int(ConfigField.HOST_TIMEOUT_MS))["value"])
            client.set_telemetry_rate(args.telemetry_interval_ms)
            client.set_mode(ControlMode.POSITION)

            settle_timeout = max(5.0, abs(args.steps) / max(args.speed, 1) + 3.0)
            for cycle in range(args.cycles):
                client.move_rel(args.steps, args.speed, args.accel)
                _bench_wait_idle(client, log_handle, f"move_positive_{cycle + 1}", settle_timeout)
                client.move_rel(-args.steps, args.speed, args.accel)
                _bench_wait_idle(client, log_handle, f"move_negative_{cycle + 1}", settle_timeout)

            client.set_mode(ControlMode.VELOCITY)
            client.run_velocity(args.speed, args.accel)
            time.sleep(args.velocity_run_s)
            client.ramp_stop(args.accel)
            _bench_wait_idle(client, log_handle, "velocity_positive_stop", max(3.0, args.velocity_run_s + 2.0))
            client.run_velocity(-args.speed, args.accel)
            time.sleep(args.velocity_run_s)
            client.ramp_stop(args.accel)
            _bench_wait_idle(client, log_handle, "velocity_negative_stop", max(3.0, args.velocity_run_s + 2.0))

            test_timeout_ms = max(100, min(args.host_timeout_ms, 1000))
            client.config_stage(int(ConfigField.HOST_TIMEOUT_MS), test_timeout_ms)
            client.config_apply()
            client.config_save()
            client.acked(MsgId.REBOOT)
            transport, client = _bench_reopen(transport, args.port)
            persisted_timeout = int(client.config_get(int(ConfigField.HOST_TIMEOUT_MS))["value"])
            _write_jsonl(log_handle, "config_persisted", host_timeout_ms=persisted_timeout)
            if persisted_timeout != test_timeout_ms:
                raise RuntimeError(f"host timeout did not persist: {persisted_timeout} != {test_timeout_ms}")

            client.clear_faults()
            client.set_mode(ControlMode.VELOCITY)
            client.run_velocity(args.speed, args.accel)
            time.sleep((test_timeout_ms / 1000.0) + 0.4)
            timeout_status = client.status()
            _write_jsonl(log_handle, "status", label="after_host_timeout", status=timeout_status)
            if not (int(timeout_status["faults"]) & HOST_TIMEOUT_FAULT):
                raise RuntimeError(f"host timeout fault did not set: {timeout_status}")
            client.ramp_stop(args.accel)
            _bench_wait_idle(client, log_handle, "timeout_stop", 3.0)
            client.clear_faults()

            client.config_stage(int(ConfigField.HOST_TIMEOUT_MS), original_host_timeout)
            client.config_apply()
            client.config_save()
            client.acked(MsgId.REBOOT)
            transport, client = _bench_reopen(transport, args.port)
            restored_timeout = int(client.config_get(int(ConfigField.HOST_TIMEOUT_MS))["value"])
            _write_jsonl(log_handle, "config_restored", host_timeout_ms=restored_timeout)
            if restored_timeout != original_host_timeout:
                raise RuntimeError(f"host timeout restore failed: {restored_timeout} != {original_host_timeout}")

            client.clear_faults()
            client.set_mode(ControlMode.DISABLED)
            final_status = client.status()
            final_faults = client.faults()
            final_counters = client.counters()
            _write_jsonl(log_handle, "snapshot", label="final", status=final_status, faults=final_faults, counters=final_counters, transport_stats=transport.stats.__dict__)
            if final_status["state"] != "DISABLED" or final_status["faults"] != 0:
                raise RuntimeError(f"final state is not clean: {final_status}")

            return {
                "ok": True,
                "log_jsonl": str(log_path),
                "final_status": final_status,
                "transport_stats": transport.stats.__dict__,
                "telemetry_packets_seen": len(client.telemetry),
            }
        finally:
            if client is not None:
                try:
                    client.ramp_stop(args.accel)
                except Exception as exc:
                    _write_jsonl(log_handle, "cleanup_error", action="ramp_stop", error=str(exc))
                try:
                    client.clear_faults()
                except Exception as exc:
                    _write_jsonl(log_handle, "cleanup_error", action="clear_faults", error=str(exc))
                if original_host_timeout is not None:
                    try:
                        current_timeout = int(client.config_get(int(ConfigField.HOST_TIMEOUT_MS))["value"])
                        if current_timeout != original_host_timeout:
                            client.config_stage(int(ConfigField.HOST_TIMEOUT_MS), original_host_timeout)
                            client.config_apply()
                            client.config_save()
                    except Exception as exc:
                        _write_jsonl(log_handle, "cleanup_error", action="restore_host_timeout", error=str(exc))
                try:
                    client.set_mode(ControlMode.DISABLED)
                except Exception as exc:
                    _write_jsonl(log_handle, "cleanup_error", action="disable", error=str(exc))
            if transport is not None:
                _write_jsonl(log_handle, "transport_stats", stats=transport.stats.__dict__)
                transport.close()


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

    p = sub.add_parser("bench")
    p.add_argument("port")
    p.add_argument("--cycles", type=int, default=2)
    p.add_argument("--steps", type=int, default=1000)
    p.add_argument("--speed", type=int, default=1000)
    p.add_argument("--accel", type=int, default=2000)
    p.add_argument("--telemetry-interval-ms", type=int, default=100)
    p.add_argument("--velocity-run-s", type=float, default=0.8)
    p.add_argument("--host-timeout-ms", type=int, default=250)
    p.add_argument("--log-jsonl")

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
    elif args.cmd == "bench":
        print_json(run_bench(args))
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
